#pragma once
#include <atomic>
#include <cstdint>
extern "C" void DMA1_Channel2_IRQHandler(void);
extern "C" void DMA1_Channel3_IRQHandler(void);
namespace roller::platform {
class SsiDma final {
  public:
    using Completion = void (*)(std::uint32_t frame, bool ok);
    bool initialize() noexcept;
    bool start(Completion done) noexcept;
    void abort() noexcept;
    bool busy() const noexcept {
        return busy_.load(std::memory_order_acquire);
    }

  private:
    friend void ::DMA1_Channel2_IRQHandler(void);
    friend void ::DMA1_Channel3_IRQHandler(void);
    void irq() noexcept;
    void finish(bool ok) noexcept;
    void release(bool reset) noexcept;
    std::atomic<bool> busy_{};
    Completion done_{};
    alignas(4) volatile std::uint8_t response_[3]{};
    alignas(4) const std::uint8_t dummy_[3]{0, 0, 0};
    std::uint32_t cr1_{}, cr2_{}, last_cs_high_{};
    bool initialized_{};
};
} // namespace roller::platform
