#pragma once
#include "co_transport.h"
#include "roller/core/channels.hpp"
#include "roller/core/module.hpp"
#include "roller/core/storage.hpp"
#include "roller/drivers/mt6701.hpp"
#include "ssi_dma.hpp"
#include <atomic>
#include <expected>
namespace roller::platform {
struct HardwareDiagnostics {
    std::uint32_t sequence{}, frames{}, encoder_frames{}, encoder_errors{}, encoder_raw{}, encoder_status{};
    std::uint32_t adc_a{}, adc_b{}, adc_c{}, adc_bus{}, adc_ntc{}, current_mask{}, adc_errors{},
        missed_frames{};
    float angle{}, bus{}, vdda{}, temperature{}, offset_a{}, offset_b{}, offset_c{};
    std::uint32_t offsets_valid{}, driver_awake{}, no_motion{}, pwm_deferred{}, warnings{}, can_loopback{};
};
class Board final : public Module {
  public:
    explicit Board(Channels& channels) noexcept : channels_(channels) {}
    std::string_view name() const noexcept override {
        return "stm32-board-v2";
    }
    Result start_sampling() noexcept;
    void stop_sampling() noexcept;
    drive::Sample sample() noexcept;
    bool begin_sample(unsigned half, void (*ready)()) noexcept;
    void encoder_trigger() noexcept;
    void apply(drive::Output output) noexcept;
    static void gate_off() noexcept;
    RollerCanTransport can_transport() noexcept;
    bool receive(RollerCanFrame& frame) noexcept {
        return can_rx_.pop(frame);
    }
    void can_irq() noexcept;
    bool sampling() const noexcept {
        return sampling_.load(std::memory_order_acquire);
    }

  private:
    Result on_start() noexcept override;
    Result on_stop() noexcept override;
    static void encoder_complete(std::uint32_t frame, bool ok) noexcept;
    bool housekeeping() noexcept;
    drive::Sample current_sample() noexcept;
    static Board* encoder_owner_;
    SsiDma encoder_;
    alignas(4) std::array<std::uint32_t, 6> adc_dma_{};
    std::array<std::uint16_t, 5> adc_frame_{};
    drive::Sample prepared_sample_{};
    bool encoder_ok_{};
    float encoder_angle_{};
    bool housekeeping_pending_{};
    Channels& channels_;
    SpscQueue<RollerCanFrame, 32> can_rx_;
    std::atomic<bool> sampling_{};
    std::atomic<std::uint32_t> rx_errors_{};
    float vdda_{3.3F}, temperature_{25};
    std::array<std::uint32_t, 3> offset_sum_{};
    std::array<float, 3> offsets_{2048, 2048, 2048};
    std::uint32_t offset_samples_{};
    bool offsets_valid_{}, duty_pending_{}, enable_pending_{};
    std::array<std::uint16_t, 3> active_duty_{}, pending_duty_{};
    std::uint32_t housekeeping_ticks_{}, housekeeping_started_{}, wake_cycles_{};
    std::uint16_t bus_off_ticks_{};
    std::uint32_t synchronous_tx_mask_{};
};
class FlashStore final : public ParameterStore {
  public:
    explicit FlashStore(Board& board) noexcept : board_(board) {}
    bool read(std::span<std::byte> data) noexcept override;
    bool write(std::span<const std::byte> data) noexcept override;

  private:
    Board& board_;
};
} // namespace roller::platform
extern "C" {
extern volatile roller::platform::HardwareDiagnostics roller_hardware;
extern roller::platform::HardwareDiagnostics roller_hardware_snapshot;
extern volatile std::uint32_t roller_hardware_request, roller_hardware_ack;
}
