#include "board.hpp"
#include "301/CO_driver.h"
#include "adc.h"
#include "cordic_math.hpp"
#include "fdcan.h"
#include "main.h"
#include "roller/drivers/rollercan_sensing.hpp"
#include "spi.h"
#include "stm32g4xx_ll_adc.h"
#include "tim.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#ifdef ROLLER_BENCH
#include "bench.hpp"
#endif

namespace roller::platform {
namespace {
constexpr std::uint32_t cpu_hz = 160000000;
constexpr std::uint32_t encoder_turnaround_cycles = 32; // 200 ns; SSC minimum is 130 ns.
void delay_cycles(std::uint32_t cycles) {
    const auto start = DWT->CYCCNT;
    while (static_cast<std::uint32_t>(DWT->CYCCNT - start) < cycles) {
        __NOP();
    }
}
} // namespace
void Board::gate_off() noexcept {
    // PB2 disables all three DRV8311 half bridges independently of timer/MOE.
    GPIOB->BSRR = static_cast<std::uint32_t>(GPIO_PIN_2) << 16;
    __DMB();
}
Result Board::on_start() noexcept {
    gate_off();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    if (!initialize_cordic()) {
        return std::unexpected(Error::hardware);
    }
#ifdef DEBUG
    __HAL_DBGMCU_FREEZE_IWDG();
#endif
    if (SystemCoreClock != cpu_hz || htim1.Init.Period != 4000 || hspi1.Init.DataSize != SPI_DATASIZE_16BIT ||
        hfdcan1.Init.FrameFormat != FDCAN_FRAME_CLASSIC) {
        return std::unexpected(Error::configuration);
    }
    if (HAL_TIM_Base_Start(&htim7) != HAL_OK) {
        return std::unexpected(Error::hardware);
    }
    HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_SET);
    HAL_Delay(5); // DRV wake and CSA settling, with half bridges disabled.
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) {
        return std::unexpected(Error::hardware);
    }
    // Regular ADC sequence is sampled before the injected loop starts. EOC is
    // observed directly for each rank so DR is consumed before the next rank.
    for (int rank = 0; rank < 2; ++rank) {
        if (HAL_ADC_Start(&hadc1) != HAL_OK) {
            return std::unexpected(Error::hardware);
        }
        const auto begin = DWT->CYCCNT;
        while (!__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_EOC)) {
            if (DWT->CYCCNT - begin > 800000) {
                return std::unexpected(Error::timeout);
            }
        }
        const auto raw = static_cast<std::uint16_t>(HAL_ADC_GetValue(&hadc1));
        if (rank == 0) {
            temperature_ = static_cast<float>(raw);
        } else {
            if (raw < 500 || raw > 3000) {
                return std::unexpected(Error::hardware);
            }
            vdda_ =
                static_cast<float>(__LL_ADC_CALC_VREFANALOG_VOLTAGE(raw, LL_ADC_RESOLUTION_12B)) / 1000.0F;
        }
    }
    (void)HAL_ADC_Stop(&hadc1);
    temperature_ = static_cast<float>(__LL_ADC_CALC_TEMPERATURE(static_cast<std::uint32_t>(vdda_ * 1000.0F),
                                                                static_cast<std::uint32_t>(temperature_),
                                                                LL_ADC_RESOLUTION_12B));
    if (vdda_ < 2.9F || vdda_ > 3.6F) {
        return std::unexpected(Error::hardware);
    }
    __HAL_SPI_ENABLE(&hspi1);
    // Reading STAT acknowledges the encoder's power-on reset indication.
    HAL_GPIO_WritePin(ENC_CS_GPIO_Port, ENC_CS_Pin, GPIO_PIN_RESET);
    delay_cycles(100);
    auto command = exchange(0x8001);
    const auto moder = GPIOA->MODER;
    MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE7, 0);
    delay_cycles(encoder_turnaround_cycles);
    auto status = exchange(0xFFFF);
    auto safety = exchange(0xFFFF);
    HAL_GPIO_WritePin(ENC_CS_GPIO_Port, ENC_CS_Pin, GPIO_PIN_SET);
    GPIOA->MODER = moder;
    if (!command || !status || !safety) {
        return std::unexpected(Error::hardware);
    }
    delay_cycles(100);
    if (!angle()) {
        return std::unexpected(Error::hardware);
    }
    return {};
}
Result Board::on_stop() noexcept {
    stop_sampling();
    gate_off();
    HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CAN_STB_GPIO_Port, CAN_STB_Pin, GPIO_PIN_SET);
    return {};
}
Result Board::start_sampling() noexcept {
    gate_off();
#ifdef ROLLER_BENCH
    bench::sampling_restart();
#endif
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 2000);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 2000);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 2000);
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    (void)HAL_TIM_GenerateEvent(&htim1, TIM_EVENTSOURCE_UPDATE);
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOC | ADC_FLAG_JEOS | ADC_FLAG_JQOVF);
    if (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK) {
        return std::unexpected(Error::hardware);
    }
    sampling_.store(true, std::memory_order_release);
    for (auto channel : {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4}) {
        if (HAL_TIM_PWM_Start(&htim1, channel) != HAL_OK) {
            stop_sampling();
            return std::unexpected(Error::hardware);
        }
    }
    channels_.sampling_epoch.fetch_add(1, std::memory_order_release);
    return {};
}
void Board::stop_sampling() noexcept {
    gate_off();
    sampling_.store(false, std::memory_order_release);
    (void)HAL_ADCEx_InjectedStop_IT(&hadc1);
    for (auto channel : {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4}) {
        (void)HAL_TIM_PWM_Stop(&htim1, channel);
    }
}
std::expected<std::uint16_t, drivers::EncoderError> Board::exchange(std::uint16_t value) noexcept {
    const auto start = DWT->CYCCNT;
    while (!(SPI1->SR & SPI_SR_TXE)) {
        if (DWT->CYCCNT - start > 1600) {
            return std::unexpected(drivers::EncoderError::timeout);
        }
    }
    // STM32 SPI requires a half-word access in 16-bit mode. GCC's may_alias
    // type expresses this intentional alternate view of the CMSIS register.
    using SpiWord [[gnu::may_alias]] = std::uint16_t;
    auto* const data = reinterpret_cast<volatile SpiWord*>(&SPI1->DR);
    *data = value;
    while (!(SPI1->SR & SPI_SR_RXNE)) {
        if (DWT->CYCCNT - start > 1600) {
            return std::unexpected(drivers::EncoderError::timeout);
        }
    }
    const auto result = *data;
    while (SPI1->SR & SPI_SR_BSY) {
        if (DWT->CYCCNT - start > 1600) {
            return std::unexpected(drivers::EncoderError::timeout);
        }
    }
    return result;
}
std::expected<float, drivers::EncoderError> Board::angle() noexcept {
    HAL_GPIO_WritePin(ENC_CS_GPIO_Port, ENC_CS_Pin, GPIO_PIN_RESET);
    delay_cycles(60);
    auto sent = exchange(0x8021);
    const auto mode = GPIOA->MODER;
    MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE7, 0); // DATA turnaround: MOSI high impedance.
    delay_cycles(encoder_turnaround_cycles);
    auto value = exchange(0xFFFF);
    auto safety = exchange(0xFFFF);
    HAL_GPIO_WritePin(ENC_CS_GPIO_Port, ENC_CS_Pin, GPIO_PIN_SET);
    GPIOA->MODER = mode;
    if (!sent || !value || !safety) {
        __HAL_SPI_DISABLE(&hspi1);
        __HAL_SPI_CLEAR_OVRFLAG(&hspi1);
        __HAL_SPI_ENABLE(&hspi1);
        return std::unexpected(drivers::EncoderError::timeout);
    }
    return drivers::decode_angle(*value, *safety);
}
void Board::acknowledge_encoder() noexcept {
    gate_off();
    HAL_GPIO_WritePin(ENC_CS_GPIO_Port, ENC_CS_Pin, GPIO_PIN_RESET);
    delay_cycles(60);
    (void)exchange(0x8001);
    const auto mode = GPIOA->MODER;
    MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE7, 0);
    delay_cycles(encoder_turnaround_cycles);
    (void)exchange(0xFFFF);
    (void)exchange(0xFFFF);
    HAL_GPIO_WritePin(ENC_CS_GPIO_Port, ENC_CS_Pin, GPIO_PIN_SET);
    GPIOA->MODER = mode;
    delay_cycles(100);
    // The next normal angle read must still pass CRC/status; acknowledgement
    // does not clear the software drive fault or bypass validation.
}
drive::Sample Board::sample() noexcept {
    const std::array<std::uint16_t, 3> raw{static_cast<std::uint16_t>(ADC1->JDR1),
                                           static_cast<std::uint16_t>(ADC1->JDR2),
                                           static_cast<std::uint16_t>(ADC1->JDR3)};
    if (offset_samples_ < 1024) {
        for (std::size_t i = 0; i < 3; ++i) {
            offset_sum_[i] += raw[i];
            offset_min_[i] = std::min(offset_min_[i], raw[i]);
            offset_max_[i] = std::max(offset_max_[i], raw[i]);
        }
        if (++offset_samples_ == 1024) {
            offsets_valid_ = true;
            for (std::size_t i = 0; i < 3; ++i) {
                offsets_[i] = static_cast<float>(offset_sum_[i]) / 1024.0F;
                if (offsets_[i] < 1700 || offsets_[i] > 2400 || offset_max_[i] - offset_min_[i] > 100) {
                    offsets_valid_ = false;
                }
            }
        }
    }
    drive::Sample s{};
    // GAIN floating: 1 V/A. Verified with a stationary positive alpha vector:
    // A sources current (ADC above offset), B/C sink current (below offset).
    const auto scale = vdda_ / 4095.0F;
    s.current = {(raw[0] - offsets_[0]) * scale, (raw[1] - offsets_[1]) * scale,
                 (raw[2] - offsets_[2]) * scale};
    s.bus_voltage = drivers::bus_voltage(static_cast<std::uint16_t>(ADC1->JDR4), vdda_);
    s.mcu_temperature = temperature_;
    const auto position = angle();
    s.encoder_ok = position.has_value();
    s.angle = position.value_or(0);
    s.driver_ok = HAL_GPIO_ReadPin(DRV_FAULT_GPIO_Port, DRV_FAULT_Pin) == GPIO_PIN_SET;
    s.adc_ok = std::all_of(raw.begin(), raw.end(), [](auto x) { return x > 20 && x < 4075; }) &&
               !(ADC1->ISR & ADC_ISR_JQOVF) && housekeeping();
    s.current_calibrated = offsets_valid_;
    if (offset_samples_ == 1024 && !offsets_valid_) {
        s.adc_ok = false;
    }
    return s;
}
bool Board::housekeeping() noexcept {
    // One regular rank at a time, as configured by CubeMX's discontinuous mode.
    // Poll at the following injected IRQ instead of blocking the current loop.
    ++housekeeping_ticks_;
    if (regular_stage_ && __HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_EOC)) {
        const auto raw = static_cast<std::uint16_t>(ADC1->DR);
        if (regular_stage_ == 1) {
            raw_temperature_ = raw;
            regular_stage_ = 2;
            LL_ADC_REG_StartConversion(ADC1);
        } else {
            regular_stage_ = 0;
            if (raw < 500 || raw > 3000) {
                return false;
            }
            const auto mv = __LL_ADC_CALC_VREFANALOG_VOLTAGE(raw, LL_ADC_RESOLUTION_12B);
            if (mv < 2900 || mv > 3600) {
                return false;
            }
            vdda_ = static_cast<float>(mv) / 1000.0F;
            temperature_ =
                static_cast<float>(__LL_ADC_CALC_TEMPERATURE(mv, raw_temperature_, LL_ADC_RESOLUTION_12B));
        }
    }
    if (regular_stage_ && housekeeping_ticks_ - regular_started_ > 4) {
        return false;
    }
    if (!regular_stage_ && housekeeping_ticks_ % 2000 == 0) {
        __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR);
        regular_stage_ = 1;
        regular_started_ = housekeeping_ticks_;
        LL_ADC_REG_StartConversion(ADC1);
    }
    return true;
}
void Board::apply(drive::Output output) noexcept {
    if (!std::isfinite(output.duty.a) || !std::isfinite(output.duty.b) || !std::isfinite(output.duty.c)) {
        gate_off();
        channels_.asynchronous_fault.store(static_cast<std::uint32_t>(drive::Fault::configuration));
        return;
    }
    if (!output.enable || channels_.inhibit.load(std::memory_order_acquire) || !offsets_valid_) {
        gate_off();
        return;
    }
    const auto ticks = [](float duty) {
        return static_cast<std::uint32_t>(std::clamp(duty, 0.1F, 0.9F) * 4000.0F);
    };
    // A -> TIM1_CH3, B -> CH2, C -> CH1: verified against the schematic.
    TIM1->CCR3 = ticks(output.duty.a);
    TIM1->CCR2 = ticks(output.duty.b);
    TIM1->CCR1 = ticks(output.duty.c);
    __DMB();
    // An EXTI fault may preempt above. Recheck input and latch before enabling.
    const auto irq = __get_PRIMASK();
    __disable_irq();
    if (!channels_.asynchronous_fault.load(std::memory_order_acquire) &&
        !channels_.inhibit.load(std::memory_order_acquire) && (GPIOB->IDR & DRV_FAULT_Pin)) {
        GPIOB->BSRR = PWM_ENABLE_Pin;
    }
    __set_PRIMASK(irq);
}
void Board::fault_irq() noexcept {
    gate_off();
    if (sampling()) {
        channels_.asynchronous_fault.store(static_cast<std::uint32_t>(drive::Fault::driver),
                                           std::memory_order_release);
    }
}
void Board::can_irq() noexcept {
    // Hardware FIFO depth is 3. Bound each ISR; further arrivals pend another IRQ.
    for (unsigned i = 0; i < 3 && HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0; ++i) {
        FDCAN_RxHeaderTypeDef header{};
        std::array<std::uint8_t, 64> data{};
        if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &header, data.data()) != HAL_OK) {
            break;
        }
        const auto size = header.DataLength;
        if (header.IdType != FDCAN_STANDARD_ID || header.RxFrameType != FDCAN_DATA_FRAME ||
            header.FDFormat != FDCAN_CLASSIC_CAN || size > 8) {
            continue;
        }
        RollerCanFrame frame{};
        frame.ident = static_cast<std::uint16_t>(header.Identifier);
        frame.size = static_cast<std::uint8_t>(size);
        std::memcpy(frame.data, data.data(), size);
        if (!can_rx_.push(frame)) {
            rx_errors_.store(CO_CAN_ERRRX_OVERFLOW);
            gate_off();
            channels_.asynchronous_fault.store(static_cast<std::uint32_t>(drive::Fault::communication));
        }
    }
}
RollerCanTransport Board::can_transport() noexcept {
    return {this,
            [](void*) {
                FDCAN_FilterTypeDef filter{};
                filter.IdType = FDCAN_STANDARD_ID;
                filter.FilterIndex = 0;
                filter.FilterType = FDCAN_FILTER_MASK;
                filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
                filter.FilterID1 = 0;
                filter.FilterID2 = 0;
                if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK ||
                    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE,
                                                 FDCAN_REJECT_REMOTE) != HAL_OK ||
                    HAL_FDCAN_ActivateNotification(&hfdcan1,
                                                   FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                                                       FDCAN_IT_RX_FIFO0_MESSAGE_LOST | FDCAN_IT_BUS_OFF,
                                                   0) != HAL_OK ||
                    HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
                    return false;
                }
                HAL_GPIO_WritePin(CAN_STB_GPIO_Port, CAN_STB_Pin, GPIO_PIN_RESET);
                return true;
            },
            [](void*) {
                HAL_GPIO_WritePin(CAN_STB_GPIO_Port, CAN_STB_Pin, GPIO_PIN_SET);
                if (hfdcan1.State == HAL_FDCAN_STATE_BUSY) {
                    (void)HAL_FDCAN_Stop(&hfdcan1);
                }
            },
            [](void* context, const RollerCanFrame* f) {
                auto& self = *static_cast<Board*>(context);
                if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0) {
                    return false;
                }
                FDCAN_TxHeaderTypeDef h{};
                h.Identifier = f->ident;
                h.IdType = FDCAN_STANDARD_ID;
                h.TxFrameType = FDCAN_DATA_FRAME;
                h.DataLength = static_cast<std::uint32_t>(f->size);
                h.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
                h.BitRateSwitch = FDCAN_BRS_OFF;
                h.FDFormat = FDCAN_CLASSIC_CAN;
                h.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
                self.synchronous_tx_mask_ &= hfdcan1.Instance->TXBRP;
                if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &h, f->data) != HAL_OK) {
                    return false;
                }
                const auto slot = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan1);
                self.synchronous_tx_mask_ &= ~slot;
                if (f->synchronous) {
                    self.synchronous_tx_mask_ |= slot;
                }
                return true;
            },
            [](void* context) -> std::uint16_t {
                auto& self = *static_cast<Board*>(context);
                FDCAN_ProtocolStatusTypeDef status{};
                if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &status) != HAL_OK) {
                    return CO_CAN_ERRTX_BUS_OFF;
                }
                std::uint16_t errors = static_cast<std::uint16_t>(self.rx_errors_.load());
                if (status.BusOff) {
                    errors |= CO_CAN_ERRTX_BUS_OFF;
                    self.gate_off();
                    self.channels_.asynchronous_fault.store(
                        static_cast<std::uint32_t>(drive::Fault::communication));
                    if (++self.bus_off_ticks_ >= 500) {
                        self.bus_off_ticks_ = 0;
                        (void)HAL_FDCAN_AbortTxRequest(&hfdcan1, hfdcan1.Instance->TXBRP);
                        if (HAL_FDCAN_Stop(&hfdcan1) == HAL_OK) {
                            (void)HAL_FDCAN_Start(&hfdcan1);
                        }
                    }
                } else {
                    self.bus_off_ticks_ = 0;
                }
                if (status.ErrorPassive) {
                    errors |= CO_CAN_ERRTX_PASSIVE | CO_CAN_ERRRX_PASSIVE;
                }
                if (status.Warning) {
                    errors |= CO_CAN_ERRTX_WARNING | CO_CAN_ERRRX_WARNING;
                }
                return errors;
            },
            [](void* context) {
                auto& self = *static_cast<Board*>(context);
                const auto pending = hfdcan1.Instance->TXBRP & self.synchronous_tx_mask_;
                if (pending) {
                    (void)HAL_FDCAN_AbortTxRequest(&hfdcan1, pending);
                }
                self.synchronous_tx_mask_ = 0;
            }};
}
} // namespace roller::platform
