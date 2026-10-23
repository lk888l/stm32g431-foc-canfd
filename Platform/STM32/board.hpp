#pragma once
#include "co_transport.h"
#include "roller/core/channels.hpp"
#include "roller/core/module.hpp"
#include "roller/core/storage.hpp"
#include "roller/drivers/tli5012.hpp"
#include "ssc_dma.hpp"
#include <atomic>
#include <expected>

namespace roller::platform {
class Board final : public Module {
  public:
    explicit Board(Channels& channels) noexcept : channels_(channels) {}
    [[nodiscard]] std::string_view name() const noexcept override {
        return "stm32-board";
    }
    [[nodiscard]] Result start_sampling() noexcept;
    void stop_sampling() noexcept;
    [[nodiscard]] drive::Sample sample() noexcept;
    [[nodiscard]] bool begin_sample(unsigned half, void (*ready)()) noexcept;
    void encoder_trigger() noexcept;
    void apply(drive::Output output) noexcept;
    static void gate_off() noexcept;
    [[nodiscard]] RollerCanTransport can_transport() noexcept;
    [[nodiscard]] bool receive(RollerCanFrame& frame) noexcept {
        return can_rx_.pop(frame);
    }
    void can_irq() noexcept;
    void fault_irq() noexcept;
    void acknowledge_encoder() noexcept;
    [[nodiscard]] bool sampling() const noexcept {
        return sampling_.load(std::memory_order_acquire);
    }

  private:
    Result on_start() noexcept override;
    Result on_stop() noexcept override;
    bool read_encoder(std::uint16_t command) noexcept;
    static void encoder_complete(std::uint16_t value, std::uint16_t safety, bool ok) noexcept;
    bool housekeeping() noexcept;
    [[nodiscard]] drive::Sample current_sample() noexcept;
    static Board* encoder_owner_;
    SscDma encoder_;
    alignas(4) std::array<std::uint16_t, 8> adc_dma_{};
    std::array<std::uint16_t, 4> adc_frame_{};
    drive::Sample prepared_sample_{};
    void (*sample_ready_)(){};
    std::atomic<bool> encoder_finished_{};
    bool encoder_ok_{}, sample_pending_{}, acknowledge_requested_{}, acknowledging_{}, reset_frame_{},
        reading_status_{}, encoder_cycle_started_{};
    float encoder_angle_{};
    bool housekeeping_ok_{true}, housekeeping_pending_{};
    Channels& channels_;
    // Bound SRAM use with full 64-byte FD frames; overflow remains a latched fault.
    SpscQueue<RollerCanFrame, 32> can_rx_;
    std::atomic<bool> sampling_{};
    std::atomic<std::uint32_t> rx_errors_{};
    float vdda_{3.3F}, temperature_{25};
    std::array<std::uint32_t, 3> offset_sum_{};
    std::array<std::uint16_t, 3> offset_min_{4095, 4095, 4095}, offset_max_{};
    std::array<float, 3> offsets_{2048, 2048, 2048};
    std::uint32_t offset_samples_{};
    bool offsets_valid_{};
    std::uint32_t housekeeping_ticks_{}, housekeeping_started_{};
    std::uint16_t bus_off_ticks_{};
    std::uint32_t synchronous_tx_mask_{}; // CAN task owns hardware TX bookkeeping.
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
