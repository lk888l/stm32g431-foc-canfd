#pragma once
#include "co_transport.h"
#include "roller/core/channels.hpp"
#include "roller/core/module.hpp"
#include "roller/core/storage.hpp"
#include "roller/drivers/tli5012.hpp"
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
    std::expected<std::uint16_t, drivers::EncoderError> exchange(std::uint16_t value) noexcept;
    std::expected<float, drivers::EncoderError> angle() noexcept;
    bool housekeeping() noexcept;
    Channels& channels_;
    SpscQueue<RollerCanFrame, 64> can_rx_;
    std::atomic<bool> sampling_{};
    std::atomic<std::uint32_t> rx_errors_{};
    float vdda_{3.3F}, temperature_{25};
    std::array<std::uint32_t, 3> offset_sum_{};
    std::array<std::uint16_t, 3> offset_min_{4095, 4095, 4095}, offset_max_{};
    std::array<float, 3> offsets_{2048, 2048, 2048};
    std::uint32_t offset_samples_{};
    bool offsets_valid_{};
    std::uint32_t housekeeping_ticks_{}, regular_started_{};
    std::uint16_t raw_temperature_{};
    std::uint8_t regular_stage_{};
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
