#pragma once
#include <atomic>
#include <cstdint>

extern "C" void DMA1_Channel2_IRQHandler(void);
extern "C" void DMA1_Channel3_IRQHandler(void);

namespace roller::platform {
// Owns SPI1 and DMA1 channels 2/3. Each SSC frame has a command phase,
// a DATA turnaround, and a two-word response phase under the same CS.
class SscDma final {
  public:
    using Completion = void (*)(std::uint16_t value, std::uint16_t safety, bool ok);
    SscDma() noexcept = default;
    SscDma(const SscDma&) = delete;
    SscDma& operator=(const SscDma&) = delete;
    [[nodiscard]] bool initialize() noexcept;
    [[nodiscard]] bool start(std::uint16_t command, Completion done) noexcept;
    // Cancellation never invokes the completion callback. The owner can safely
    // stop sampling or abandon a missed frame without reentering control code.
    void abort() noexcept;
    [[nodiscard]] bool busy() const noexcept {
        return phase_.load(std::memory_order_acquire) != Phase::idle;
    }

  private:
    friend void ::DMA1_Channel2_IRQHandler(void);
    friend void ::DMA1_Channel3_IRQHandler(void);
    enum class Phase : std::uint8_t { idle, command, response };
    void arm(volatile std::uint16_t* receive, const std::uint16_t* transmit, std::uint16_t count) noexcept;
    void receive_irq() noexcept;
    void transmit_irq() noexcept;
    void finish(bool ok) noexcept;
    void release(bool reset_spi) noexcept;
    std::atomic<Phase> phase_{Phase::idle};
    Completion done_{};
    alignas(4) std::uint16_t command_{};
    alignas(4) volatile std::uint16_t discard_{};
    alignas(4) volatile std::uint16_t response_[2]{};
    alignas(4) const std::uint16_t dummy_[2]{0xFFFF, 0xFFFF};
    std::uint32_t mosi_mode_{}, spi_cr1_{}, spi_cr2_{}, last_cs_high_{};
    bool initialized_{};
};
} // namespace roller::platform
