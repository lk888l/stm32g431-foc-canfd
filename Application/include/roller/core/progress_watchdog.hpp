#pragma once
#include <cstdint>
namespace roller {
class ProgressWatchdog {
  public:
    bool observe(std::uint32_t samples, std::uint32_t can, std::uint32_t epoch) noexcept {
        // A deliberate ADC restart after flash maintenance starts a new window.
        // The following observation must still see progress from both actors.
        const bool healthy = epoch != epoch_ || (samples != samples_ && can != can_);
        samples_ = samples;
        can_ = can;
        epoch_ = epoch;
        return healthy;
    }

  private:
    std::uint32_t samples_{}, can_{}, epoch_{};
};
} // namespace roller
