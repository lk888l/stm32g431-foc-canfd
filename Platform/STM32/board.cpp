#include "board.hpp"
#include "301/CO_driver.h"
#include "adc.h"
#include "cordic_math.hpp"
#include "fdcan.h"
#include "fdcan_timing.hpp"
#include "main.h"
#include "pwm_dma.hpp"
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
} // namespace
Board* Board::encoder_owner_{};
void Board::gate_off() noexcept {
    // PB2 disables all three DRV8311 half bridges independently of timer/MOE.
    GPIOB->BSRR = static_cast<std::uint32_t>(GPIO_PIN_2) << 16;
    __DMB();
}
Result Board::on_start() noexcept {
    gate_off();
    // Stream sequential hot-path instructions across the four Flash wait
    // states; keep the instruction cache available for branches.
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    encoder_owner_ = this;
    if (!initialize_cordic() || !initialize_pwm_dma() || !encoder_.initialize()) {
        return std::unexpected(Error::hardware);
    }
#ifdef DEBUG
    __HAL_DBGMCU_FREEZE_IWDG();
#endif
    if (SystemCoreClock != cpu_hz || htim1.Init.Period != 4000 || hspi1.Init.DataSize != SPI_DATASIZE_16BIT ||
        hfdcan1.Init.FrameFormat != FDCAN_FRAME_FD_BRS) {
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
    // Injected conversions have no native DMA request. Only slow internal
    // channels use them; all four fast external channels use regular DMA.
    if (HAL_ADCEx_InjectedStart(&hadc1) != HAL_OK) {
        return std::unexpected(Error::hardware);
    }
    housekeeping_pending_ = true;
    const auto begin = DWT->CYCCNT;
    while (!__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_JEOS)) {
        if (DWT->CYCCNT - begin > 800000) {
            return std::unexpected(Error::timeout);
        }
    }
    const auto sensing_ok = housekeeping();
    (void)HAL_ADCEx_InjectedStop(&hadc1);
    // Startup reads also transfer with DMA; only this pre-RTOS path waits.
    if (!sensing_ok || !read_encoder(0x8001) || !read_encoder(0x8021)) {
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
    if (!start_pwm_dma()) {
        stop_sampling();
        return std::unexpected(Error::hardware);
    }
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    (void)HAL_TIM_GenerateEvent(&htim1, TIM_EVENTSOURCE_UPDATE);
    housekeeping_pending_ = false;
    housekeeping_ok_ = true;
    sample_pending_ = false;
    reset_frame_ = false;
    encoder_cycle_started_ = false;
    __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
    HAL_NVIC_ClearPendingIRQ(TIM1_UP_TIM16_IRQn);
    if (HAL_ADC_Start_DMA(&hadc1, reinterpret_cast<std::uint32_t*>(adc_dma_.data()),
                          static_cast<std::uint32_t>(adc_dma_.size())) != HAL_OK) {
        stop_sampling();
        return std::unexpected(Error::hardware);
    }
    // DMA is armed before the timer can produce the first sampling trigger.
    sampling_.store(true, std::memory_order_release);
    // Prime the first encoder frame before the timer's first natural update.
    encoder_trigger();
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
    for (auto channel : {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4}) {
        const auto result = HAL_TIM_PWM_Start(&htim1, channel);
        if (result != HAL_OK) {
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
    // Prevent a partially completed ADC/SPI callback from resuming on restart.
    const auto irq = __get_PRIMASK();
    __disable_irq();
    __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_UPDATE);
    for (auto channel : {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4}) {
        (void)HAL_TIM_PWM_Stop(&htim1, channel);
    }
    sample_pending_ = false;
    encoder_cycle_started_ = false;
    encoder_.abort();
    stop_pwm_dma();
    HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
    HAL_NVIC_DisableIRQ(ADC1_2_IRQn);
    __set_PRIMASK(irq);
    // HAL ADC stop timeouts require the low-priority HAL tick to run.
    (void)HAL_ADC_Stop_DMA(&hadc1);
    (void)HAL_ADCEx_InjectedStop(&hadc1);
    HAL_NVIC_ClearPendingIRQ(DMA1_Channel1_IRQn);
    HAL_NVIC_ClearPendingIRQ(ADC1_2_IRQn);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
}
bool Board::read_encoder(std::uint16_t command) noexcept {
    reading_status_ = command == 0x8001;
    encoder_finished_.store(false, std::memory_order_relaxed);
    if (!encoder_.start(command, encoder_complete)) {
        return false;
    }
    const auto start = DWT->CYCCNT;
    while (!encoder_finished_.load(std::memory_order_acquire)) {
        if (DWT->CYCCNT - start > 16000) {
            encoder_.abort();
            return false;
        }
    }
    return encoder_ok_;
}
void Board::encoder_complete(std::uint16_t value, std::uint16_t safety, bool ok) noexcept {
    auto& self = *encoder_owner_;
    if (self.acknowledging_) {
        self.acknowledging_ = false;
        if (ok && self.encoder_.start(0x8021, encoder_complete)) {
            return;
        }
        ok = false;
    }
    const auto decoded = drivers::decode_angle(value, safety);
    // A STAT read at startup only acknowledges reset; the subsequent angle
    // transaction still checks the full angle CRC and safety/status word.
    self.encoder_ok_ = ok && (self.reading_status_ || decoded.has_value());
    self.encoder_angle_ = decoded.value_or(0);
    self.encoder_finished_.store(true, std::memory_order_release);
    if (self.sample_pending_) {
        self.sample_pending_ = false;
        self.sample_ready_();
    }
}
void Board::acknowledge_encoder() noexcept {
    gate_off();
    acknowledge_requested_ = true;
}
void Board::encoder_trigger() noexcept {
    if (!sampling()) {
        return;
    }
    if (sample_pending_ || encoder_.busy() || encoder_cycle_started_) {
        gate_off();
        sample_pending_ = false;
        encoder_.abort();
        channels_.asynchronous_fault.store(static_cast<std::uint32_t>(drive::Fault::timing));
    }
    reading_status_ = false;
    encoder_ok_ = false;
    encoder_finished_.store(false, std::memory_order_relaxed);
    acknowledging_ = acknowledge_requested_;
    reset_frame_ = acknowledging_;
    acknowledge_requested_ = false;
    encoder_cycle_started_ = true;
    if (!encoder_.start(acknowledging_ ? 0x8001 : 0x8021, encoder_complete)) {
        gate_off();
        encoder_finished_.store(true, std::memory_order_release);
        channels_.asynchronous_fault.store(static_cast<std::uint32_t>(drive::Fault::timing));
    }
}
bool Board::begin_sample(unsigned half, void (*ready)()) noexcept {
    const auto remaining = DMA1_Channel1->CNDTR;
    // Only consume a complete half while DMA owns the other half. A delayed
    // IRQ must never mix ranks from two PWM periods.
    const bool stable = half == 0 ? remaining > 0 && remaining <= 4 : remaining == 0 || remaining > 4;
    if (!stable || sample_pending_ || !encoder_cycle_started_) {
        gate_off();
        sample_pending_ = false;
        encoder_.abort();
        encoder_ok_ = false;
        channels_.asynchronous_fault.store(static_cast<std::uint32_t>(drive::Fault::timing));
        return false;
    }
    encoder_cycle_started_ = false;
    __DMB();
    const volatile auto* source = adc_dma_.data() + half * 4;
    adc_frame_ = {source[0], source[1], source[2], source[3]};
    __DMB();
    const auto after_copy = DMA1_Channel1->CNDTR;
    if (half == 0 ? !(after_copy > 0 && after_copy <= 4) : !(after_copy == 0 || after_copy > 4)) {
        gate_off();
        encoder_ok_ = false;
        return false;
    }
    sample_ready_ = ready;
    sample_pending_ = true;
    // Prepare ADC-derived values while the SSC transaction runs. Equal IRQ
    // priorities keep completion from consuming a partially prepared sample.
    housekeeping_ok_ = housekeeping();
    prepared_sample_ = current_sample();
    if (encoder_finished_.load(std::memory_order_acquire)) {
        sample_pending_ = false;
        ready();
    }
    return true;
}
drive::Sample Board::sample() noexcept {
    auto result = prepared_sample_;
    result.encoder_ok = encoder_ok_;
    result.angle = encoder_angle_;
    result.driver_ok = (GPIOB->IDR & DRV_FAULT_Pin) != 0;
    result.adc_ok = result.adc_ok && !(ADC1->ISR & (ADC_ISR_OVR | ADC_ISR_JQOVF)) && pwm_dma_ok();
    return result;
}
drive::Sample Board::current_sample() noexcept {
    const std::array<std::uint16_t, 3> raw{adc_frame_[0], adc_frame_[1], adc_frame_[2]};
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
    s.bus_voltage = drivers::bus_voltage(adc_frame_[3], vdda_);
    s.mcu_temperature = temperature_;
    s.encoder_ok = encoder_ok_;
    s.angle = encoder_angle_;
    s.driver_ok = HAL_GPIO_ReadPin(DRV_FAULT_GPIO_Port, DRV_FAULT_Pin) == GPIO_PIN_SET;
    s.adc_ok = std::all_of(raw.begin(), raw.end(), [](auto x) { return x > 20 && x < 4075; }) &&
               !(ADC1->ISR & (ADC_ISR_OVR | ADC_ISR_JQOVF)) && housekeeping_ok_ && pwm_dma_ok();
    s.current_calibrated = offsets_valid_;
    if (offset_samples_ == 1024 && !offsets_valid_) {
        s.adc_ok = false;
    }
    return s;
}
bool Board::housekeeping() noexcept {
    ++housekeeping_ticks_;
    if (housekeeping_pending_ && __HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_JEOS)) {
        const auto raw_temperature = static_cast<std::uint16_t>(ADC1->JDR1);
        const auto raw_reference = static_cast<std::uint16_t>(ADC1->JDR2);
        __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOC | ADC_FLAG_JEOS);
        housekeeping_pending_ = false;
        if (raw_reference < 500 || raw_reference > 3000) {
            return false;
        }
        const auto mv = __LL_ADC_CALC_VREFANALOG_VOLTAGE(raw_reference, LL_ADC_RESOLUTION_12B);
        if (mv < 2900 || mv > 3600) {
            return false;
        }
        vdda_ = static_cast<float>(mv) / 1000.0F;
        temperature_ =
            static_cast<float>(__LL_ADC_CALC_TEMPERATURE(mv, raw_temperature, LL_ADC_RESOLUTION_12B));
    }
    if (housekeeping_pending_ && housekeeping_ticks_ - housekeeping_started_ > 4) {
        return false;
    }
    if (!housekeeping_pending_ && housekeeping_ticks_ % 2000 == 0) {
        // At this point the fast regular sequence is finished. These two slow
        // internal conversions finish well before the next 50-us trigger.
        __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOC | ADC_FLAG_JEOS | ADC_FLAG_JQOVF);
        housekeeping_pending_ = true;
        housekeeping_started_ = housekeeping_ticks_;
        LL_ADC_INJ_StartConversion(ADC1);
    }
    return true;
}
void Board::apply(drive::Output output) noexcept {
    if (!std::isfinite(output.duty.a) || !std::isfinite(output.duty.b) || !std::isfinite(output.duty.c)) {
        gate_off();
        channels_.asynchronous_fault.store(static_cast<std::uint32_t>(drive::Fault::configuration));
        return;
    }
    if (!output.enable || channels_.inhibit.load(std::memory_order_acquire) || !offsets_valid_ ||
        reset_frame_) {
        gate_off();
        if (!write_pwm_dma({2000, 2000, 2000})) {
            channels_.asynchronous_fault.store(static_cast<std::uint32_t>(drive::Fault::timing));
        }
        return;
    }
    const auto ticks = [](float duty) {
        return static_cast<std::uint16_t>(std::clamp(duty, 0.1F, 0.9F) * 4000.0F);
    };
    // A -> TIM1_CH3, B -> CH2, C -> CH1: verified against the schematic.
    if (!write_pwm_dma({ticks(output.duty.c), ticks(output.duty.b), ticks(output.duty.a)})) {
        gate_off();
        channels_.asynchronous_fault.store(static_cast<std::uint32_t>(drive::Fault::timing));
        return;
    }
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
        RollerCanFrame frame{};
        if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &header, frame.data) != HAL_OK) {
            break;
        }
        // G4 HAL exposes the encoded 0..15 DLC here, not the byte count.
        const auto size = roller_can_dlc_to_length(static_cast<std::uint8_t>(header.DataLength));
        const bool fd = header.FDFormat == FDCAN_FD_CAN;
        if (header.IdType != FDCAN_STANDARD_ID || header.RxFrameType != FDCAN_DATA_FRAME || size > 64 ||
            (!fd && size > 8)) {
            continue;
        }
        frame.ident = static_cast<std::uint16_t>(header.Identifier);
        frame.size = size;
        frame.fd = fd;
        frame.brs = header.BitRateSwitch == FDCAN_BRS_ON;
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
                if (f->ident > 0x7FF || f->size > 64 || (!f->fd && (f->size > 8 || f->brs)) ||
                    roller_can_wire_size(f->size) != f->size || HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0) {
                    return false;
                }
                FDCAN_TxHeaderTypeDef h{};
                h.Identifier = f->ident;
                h.IdType = FDCAN_STANDARD_ID;
                h.TxFrameType = FDCAN_DATA_FRAME;
                h.DataLength = roller_can_length_to_dlc(f->size);
                h.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
                h.BitRateSwitch = f->brs ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
                h.FDFormat = f->fd ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN;
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
            },
            [](void*, std::uint32_t nominal, std::uint32_t data, bool brs) {
                const auto n = can_nominal_timing(nominal), d = can_data_timing(data);
                if (!n || !d || !brs || hfdcan1.State == HAL_FDCAN_STATE_BUSY ||
                    HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN) != 160000000U) {
                    return false;
                }
                hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
                hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
                hfdcan1.Init.NominalPrescaler = n->prescaler;
                hfdcan1.Init.NominalTimeSeg1 = n->segment1;
                hfdcan1.Init.NominalTimeSeg2 = n->segment2;
                hfdcan1.Init.NominalSyncJumpWidth = n->sjw;
                hfdcan1.Init.DataPrescaler = d->prescaler;
                hfdcan1.Init.DataTimeSeg1 = d->segment1;
                hfdcan1.Init.DataTimeSeg2 = d->segment2;
                hfdcan1.Init.DataSyncJumpWidth = d->sjw;
                // M_CAN permits TDC only with data prescaler 1 or 2. The 1M
                // setting uses prescaler 5 and must sample without TDC.
                const bool tdc = can_tdc_enabled(*d);
                // SSP follows the measured loop delay plus the data sample point.
                const auto offset = tdc ? d->prescaler * (1U + d->segment1) : 0U;
                if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK ||
                    HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, offset, 0) != HAL_OK) {
                    return false;
                }
                return (tdc ? HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1)
                            : HAL_FDCAN_DisableTxDelayCompensation(&hfdcan1)) == HAL_OK;
            }};
}
} // namespace roller::platform
