#include "board.hpp"
#include "301/CO_driver.h"
#include "adc.h"
#include "cordic_math.hpp"
#include "fdcan.h"
#include "fdcan_timing.hpp"
#include "main.h"
#include "pwm.hpp"
#include "roller/drivers/current_window.hpp"
#include "roller/drivers/rollercan_sensing.hpp"
#include "spi.h"
#include "stm32g4xx_ll_adc.h"
#include "tim.h"
#include <algorithm>
#include <cmath>
#ifdef ROLLER_BENCH
#include "bench.hpp"
#endif
extern "C" {
volatile roller::platform::HardwareDiagnostics roller_hardware;
roller::platform::HardwareDiagnostics roller_hardware_snapshot;
volatile std::uint32_t roller_hardware_request{}, roller_hardware_ack{};
extern std::uint32_t roller_pwm_deferred;
}
namespace roller::platform {
#ifdef ROLLER_NO_MOTION
namespace {
bool can_loopback_test() noexcept {
    // Internal loopback: checks message RAM, FIFO and FD/BRS without bus traffic.
    const auto original_mode = hfdcan1.Init.Mode;
    hfdcan1.Init.Mode = FDCAN_MODE_INTERNAL_LOOPBACK;
    FDCAN_FilterTypeDef filter{};
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0x123;
    filter.FilterID2 = 0x7FF;
    bool ok = HAL_FDCAN_Init(&hfdcan1) == HAL_OK && HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) == HAL_OK &&
              HAL_FDCAN_Start(&hfdcan1) == HAL_OK;
    FDCAN_TxHeaderTypeDef tx{};
    tx.Identifier = 0x123;
    tx.IdType = FDCAN_STANDARD_ID;
    tx.TxFrameType = FDCAN_DATA_FRAME;
    tx.DataLength = FDCAN_DLC_BYTES_64;
    tx.FDFormat = FDCAN_FD_CAN;
    tx.BitRateSwitch = FDCAN_BRS_ON;
    std::array<std::uint8_t, 64> data{}, received{};
    for (unsigned i = 0; i < data.size(); ++i) {
        data[i] = static_cast<std::uint8_t>(i ^ 0xA5U);
    }
    if (ok) {
        ok = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx, data.data()) == HAL_OK;
    }
    const auto start = DWT->CYCCNT;
    while (ok && HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) == 0) {
        if (DWT->CYCCNT - start > 1600000) {
            ok = false;
        }
    }
    FDCAN_RxHeaderTypeDef rx{};
    if (ok) {
        ok = HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx, received.data()) == HAL_OK &&
             rx.Identifier == 0x123 && rx.FDFormat == FDCAN_FD_CAN && rx.BitRateSwitch == FDCAN_BRS_ON &&
             rx.DataLength == FDCAN_DLC_BYTES_64 && data == received;
    }
    (void)HAL_FDCAN_Stop(&hfdcan1);
    hfdcan1.Init.Mode = original_mode;
    (void)HAL_FDCAN_Init(&hfdcan1);
    return ok;
}
} // namespace
#endif
Board* Board::encoder_owner_{};
void Board::gate_off() noexcept {
    DRV_ENABLE_GPIO_Port->BSRR = static_cast<std::uint32_t>(DRV_ENABLE_Pin) << 16;
    CLEAR_BIT(TIM1->CCER, TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E);
    __DMB();
}
Result Board::on_start() noexcept {
    gate_off();
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
#ifdef DEBUG
    __HAL_DBGMCU_FREEZE_IWDG();
#endif
    encoder_owner_ = this;
    if (!initialize_cordic() || !encoder_.initialize()) {
        return std::unexpected(Error::hardware);
    }
    if (SystemCoreClock != 160000000 || htim1.Init.Period != 4000) {
        return std::unexpected(Error::configuration);
    }
    if (HAL_TIM_Base_Start(&htim7) != HAL_OK) {
        return std::unexpected(Error::hardware);
    }
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK ||
        HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) {
        return std::unexpected(Error::hardware);
    }
#ifdef ROLLER_NO_MOTION
    roller_hardware.no_motion = 1;
    roller_hardware.can_loopback = can_loopback_test() ? 1U : 2U;
#else
    // Wake only for current-zero acquisition; CH1-3 remain disconnected, all INH low.
    HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_SET);
    wake_cycles_ = DWT->CYCCNT;
#endif
    HAL_Delay(40); // MT6701 power-up maximum 32 ms; no motor actuation.
    return {};
}
Result Board::on_stop() noexcept {
    stop_sampling();
    return {};
}
Result Board::start_sampling() noexcept {
#ifdef ROLLER_BENCH
    bench::sampling_restart();
#endif
    if (!start_pwm()) {
        return std::unexpected(Error::hardware);
    }
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
    active_duty_ = {};
    pending_duty_ = {};
    duty_pending_ = false;
    request_pending_ = false;
    enable_pending_ = false;
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    (void)HAL_TIM_GenerateEvent(&htim1, TIM_EVENTSOURCE_UPDATE);
    __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
    HAL_NVIC_ClearPendingIRQ(TIM1_UP_TIM16_IRQn);
    if (HAL_ADC_Start(&hadc2) != HAL_OK ||
        HAL_ADCEx_MultiModeStart_DMA(&hadc1, adc_dma_.data(), 6) != HAL_OK) {
        stop_sampling();
        return std::unexpected(Error::hardware);
    }
    sampling_.store(true, std::memory_order_release);
    encoder_trigger();
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
    // CH4 drives only the ADC trigger. CH1-3 are enabled only by apply().
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK) {
        stop_sampling();
        return std::unexpected(Error::hardware);
    }
    channels_.sampling_epoch.fetch_add(1, std::memory_order_release);
    return {};
}
void Board::stop_sampling() noexcept {
    gate_off();
    sampling_.store(false, std::memory_order_release);
    const auto irq = __get_PRIMASK();
    __disable_irq();
    __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_UPDATE);
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
    encoder_.abort();
    stop_pwm();
    __set_PRIMASK(irq);
    (void)HAL_ADCEx_MultiModeStop_DMA(&hadc1);
    (void)HAL_ADC_Stop(&hadc2);
    (void)HAL_ADCEx_InjectedStop(&hadc1);
    housekeeping_pending_ = false;
}
void Board::encoder_complete(std::uint32_t frame, bool ok) noexcept {
    auto& self = *encoder_owner_;
    const auto decoded = drivers::decode_mt6701(frame);
    self.encoder_ok_ = ok && decoded.has_value();
    roller_hardware.encoder_raw = frame;
    if (self.encoder_ok_) {
        self.encoder_angle_ = decoded->angle;
        self.encoder_counts_ = decoded->counts;
        roller_hardware.encoder_status = decoded->status;
        roller_hardware.encoder_frames = roller_hardware.encoder_frames + 1;
    } else {
        roller_hardware.encoder_errors = roller_hardware.encoder_errors + 1;
    }
}
void Board::encoder_trigger() noexcept {
    if (!sampling()) {
        return;
    }
    if (duty_pending_) {
        active_duty_ = pending_duty_;
        duty_pending_ = false;
#ifndef ROLLER_NO_MOTION
        if (enable_pending_ && !channels_.inhibit.load(std::memory_order_acquire)) {
            SET_BIT(TIM1->CCER, TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E);
        }
#endif
        enable_pending_ = false;
    }
#ifndef ROLLER_NO_MOTION
    if (request_pending_ && !channels_.inhibit.load(std::memory_order_acquire) &&
        (DRV_ENABLE_GPIO_Port->ODR & DRV_ENABLE_Pin) && write_pwm(requested_duty_)) {
        pending_duty_ = requested_duty_;
        duty_pending_ = true;
        enable_pending_ = true;
        request_pending_ = false;
    }
#endif
    if (encoder_.busy()) {
        encoder_.abort();
        roller_hardware.missed_frames = roller_hardware.missed_frames + 1;
    }
    encoder_ok_ = false;
    (void)encoder_.start(encoder_complete);
}
bool Board::begin_sample(unsigned half, void (*ready)()) noexcept {
    const auto stable = [half](std::uint32_t n) { return half == 0 ? n > 0 && n <= 3 : n == 0 || n > 3; };
    if (!stable(DMA1_Channel1->CNDTR)) {
        roller_hardware.missed_frames = roller_hardware.missed_frames + 1;
        return false; // Keep the previous output; never consume a torn frame.
    }
    const volatile auto* source = adc_dma_.data() + half * 3;
    __DMB();
    const std::uint32_t ab = source[0], cv = source[1], nv = source[2];
    __DMB();
    if (!stable(DMA1_Channel1->CNDTR)) {
        return false;
    }
    adc_frame_ = {static_cast<std::uint16_t>(ab), static_cast<std::uint16_t>(ab >> 16),
                  static_cast<std::uint16_t>(cv), static_cast<std::uint16_t>(cv >> 16),
                  static_cast<std::uint16_t>(nv)};
    (void)housekeeping();
    prepared_sample_ = current_sample();
    ready();
    return true;
}
drive::Sample Board::sample() noexcept {
    return prepared_sample_;
}
drive::Sample Board::current_sample() noexcept {
    const std::array<std::uint16_t, 3> raw{adc_frame_[0], adc_frame_[1], adc_frame_[2]};
    const bool awake = (DRV_ENABLE_GPIO_Port->ODR & DRV_ENABLE_Pin) != 0;
    if (!awake) {
        driver_settled_ = false;
    } else if (!driver_settled_ && DWT->CYCCNT - wake_cycles_ >= 800000) {
        driver_settled_ = true; // Latch: CYCCNT wraps every 26.8 s at 160 MHz.
    }
    const bool settled = awake && driver_settled_;
    const auto errors = (ADC1->ISR | ADC2->ISR) & (ADC_ISR_OVR | ADC_ISR_JQOVF);
#ifndef ROLLER_NO_MOTION
    current_zero_.update(raw, settled && !errors && (TIM1->CCER & 0x111U) == 0, vdda_);
#endif
    const auto& offsets = current_zero_.means();
    const auto scale = vdda_ / (4095.0F * drivers::current_volts_per_amp);
    drive::Sample s{};
    if (current_zero_.valid() && settled) {
        s.current = {(raw[0] - offsets[0]) * scale, (raw[1] - offsets[1]) * scale,
                     (raw[2] - offsets[2]) * scale};
    }
    s.bus_voltage = drivers::bus_voltage(adc_frame_[3], vdda_);
    s.angle = encoder_angle_;
    s.encoder_counts = encoder_counts_;
    s.encoder_ok = encoder_ok_;
    s.driver_ok = true; // nFAULT is not connected in the supplied v2 pin table.
    s.mcu_temperature = temperature_;
    s.current_calibrated = current_zero_.valid();
    s.current_zero_failed = current_zero_.failed();
    s.power_ready = settled && (TIM1->CCER & 0x111U) == 0x111U;
    const auto duty = active_duty_;
#ifdef ROLLER_BENCH
    sampled_duty_ = duty;
#endif
    s.current_mask = 0;
    // 2.2 us covers fastest-slew propagation max 1.05 us + CSA settling 1 us.
    // Rank 1 holds at 25.11 us, rank 2 at 26.04 us. Test each phase's low interval.
    for (unsigned i = 0; i < 3; ++i) {
        if (settled && current_zero_.valid() && raw[i] > 20 && raw[i] < 4075 &&
            drivers::current_window_valid(duty[i], i == 2 ? 1U : 0U)) {
            s.current_mask = static_cast<std::uint8_t>(s.current_mask | (1U << i));
        }
    }
    if (errors) {
        roller_hardware.adc_errors = roller_hardware.adc_errors + 1;
        ADC1->ISR = ADC_ISR_OVR | ADC_ISR_JQOVF;
        ADC2->ISR = ADC_ISR_OVR | ADC_ISR_JQOVF;
        s.current_mask = 0;
    }
    s.adc_ok = errors == 0;
    s.current_angle_delay = 30.0e-6F;  // sensor <5 us + current sample at ~25 us.
    s.voltage_angle_delay = 130.0e-6F; // queued PWM average at 125 us + sensor delay.
    s.applied_voltage_valid = true;
    if (awake && (TIM1->CCER & TIM_CCER_CC1E)) {
        s.applied_voltage = control::clarke({duty[0] * s.bus_voltage / 4000.0F,
                                             duty[1] * s.bus_voltage / 4000.0F,
                                             duty[2] * s.bus_voltage / 4000.0F});
    }
    roller_hardware.sequence = roller_hardware.sequence + 1;
    __DMB();
    roller_hardware.frames = roller_hardware.frames + 1;
    roller_hardware.adc_a = raw[0];
    roller_hardware.adc_b = raw[1];
    roller_hardware.adc_c = raw[2];
    roller_hardware.adc_bus = adc_frame_[3];
    roller_hardware.adc_ntc = adc_frame_[4];
    roller_hardware.angle = s.angle;
    roller_hardware.bus = s.bus_voltage;
    roller_hardware.vdda = vdda_;
    roller_hardware.temperature = temperature_;
    roller_hardware.current_mask = s.current_mask;
    roller_hardware.offset_a = offsets[0];
    roller_hardware.offset_b = offsets[1];
    roller_hardware.offset_c = offsets[2];
    roller_hardware.offsets_valid = current_zero_.valid();
    roller_hardware.zero_state = static_cast<std::uint32_t>(current_zero_.state());
    roller_hardware.zero_reason = static_cast<std::uint32_t>(current_zero_.reason());
    roller_hardware.zero_span = current_zero_.span();
    roller_hardware.zero_elapsed = current_zero_.elapsed();
    roller_hardware.driver_awake = awake;
    roller_hardware.pwm_deferred = roller_pwm_deferred;
    __DMB();
    roller_hardware.sequence = roller_hardware.sequence + 1;
    return s;
}
bool Board::housekeeping() noexcept {
    ++housekeeping_ticks_;
    if (housekeeping_pending_ && __HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_JEOS)) {
        const auto raw_t = static_cast<std::uint16_t>(ADC1->JDR1);
        const auto raw_v = static_cast<std::uint16_t>(ADC1->JDR2);
        __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOC | ADC_FLAG_JEOS);
        housekeeping_pending_ = false;
        if (raw_v > 500 && raw_v < 3000) {
            const auto mv = __LL_ADC_CALC_VREFANALOG_VOLTAGE(raw_v, LL_ADC_RESOLUTION_12B);
            if (mv >= 2900 && mv <= 3600) {
                vdda_ = static_cast<float>(mv) / 1000.0F;
                temperature_ =
                    static_cast<float>(__LL_ADC_CALC_TEMPERATURE(mv, raw_t, LL_ADC_RESOLUTION_12B));
            }
        }
    }
    if (housekeeping_pending_ && housekeeping_ticks_ - housekeeping_started_ > 4) {
        housekeeping_pending_ = false; // Retry, retain the last valid VDDA/temperature.
    }
    if (!housekeeping_pending_ && housekeeping_ticks_ % 2000 == 1) {
        housekeeping_pending_ = true;
        housekeeping_started_ = housekeeping_ticks_;
        __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOC | ADC_FLAG_JEOS | ADC_FLAG_JQOVF);
        LL_ADC_INJ_StartConversion(ADC1);
    }
    return true;
}
void Board::apply(drive::Output output) noexcept {
#ifdef ROLLER_NO_MOTION
    (void)output;
    gate_off();
#else
    if (output.stop || current_zero_.failed() || channels_.inhibit.load(std::memory_order_acquire)) {
        current_zero_.fail(drivers::ZeroReason::cancelled);
        request_pending_ = false;
        enable_pending_ = false;
        gate_off();
        return;
    }
    if (!current_zero_.valid()) {
        return; // Bounded startup acquisition. Explicit stop always takes precedence.
    }
    if (!output.enable || channels_.inhibit.load(std::memory_order_acquire)) {
        request_pending_ = false;
        enable_pending_ = false;
        gate_off();
        return;
    }
    if (!std::isfinite(output.duty.a) || !std::isfinite(output.duty.b) || !std::isfinite(output.duty.c)) {
        return;
    }
    if (!(DRV_ENABLE_GPIO_Port->ODR & DRV_ENABLE_Pin)) {
        HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_SET);
        wake_cycles_ = DWT->CYCCNT;
        driver_settled_ = false;
        return;
    }
    if (!driver_settled_) {
        return;
    }
    const auto ticks = [](float d) {
        return static_cast<std::uint16_t>(std::clamp(d, 0.0F, 1.0F) * 4000.0F);
    };
    const std::array<std::uint16_t, 3> abc{ticks(output.duty.a), ticks(output.duty.b), ticks(output.duty.c)};
    // TIM1 now preempts this ISR. Publish all three phases atomically; its
    // underflow ISR alone writes the preload registers and tracks their latch.
    const auto mask = __get_PRIMASK();
    __disable_irq();
    requested_duty_ = abc;
    request_pending_ = true;
    __set_PRIMASK(mask);
#endif
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
            channels_.communication_errors.fetch_add(1, std::memory_order_relaxed);
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

                return true;
            },
            [](void*) {
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
                    self.channels_.communication_errors.fetch_add(1, std::memory_order_relaxed);
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
