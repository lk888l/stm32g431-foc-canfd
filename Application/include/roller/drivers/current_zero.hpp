#pragma once
#include <algorithm>
#include <array>
#include <cstdint>

namespace roller::drivers {
enum class ZeroState : std::uint8_t { collecting, complete, failed };
enum class ZeroReason : std::uint8_t { none, range, noise, mismatch, unavailable, cancelled };

// Startup only, with PWM disconnected and the CSA awake. A stable but floating
// input must not become a plausible zero merely by subtracting its own mean.
class CurrentZero {
  public:
    static constexpr std::uint32_t window = 1024, timeout = 20000;
    void update(const std::array<std::uint16_t, 3>& raw, bool available, float vdda = 3.3F) noexcept {
        if (state_ != ZeroState::collecting) {
            return;
        }
        if (++elapsed_ >= timeout) {
            fail(reason_ == ZeroReason::none ? ZeroReason::unavailable : reason_);
            return;
        }
        if (!available) {
            clear_window();
            return;
        }
        for (unsigned i = 0; i < 3; ++i) {
            sum_[i] += raw[i];
            minimum_[i] = std::min(minimum_[i], raw[i]);
            maximum_[i] = std::max(maximum_[i], raw[i]);
        }
        if (++count_ != window) {
            return;
        }
        reason_ = ZeroReason::none;
        span_ = 0;
        for (unsigned i = 0; i < 3; ++i) {
            means_[i] = static_cast<float>(sum_[i]) / static_cast<float>(window);
            span_ = std::max(span_, static_cast<std::uint32_t>(maximum_[i] - minimum_[i]));
        }
        // DRV8316C VREF is 2.8 V..AVDD (~3.3 V), so zero is about 1.4..1.65 V.
        // Include reference/ADC tolerances, not voltages near the positive rail.
        if (*std::min_element(means_.begin(), means_.end()) * vdda / 4095.0F < 1.25F ||
            *std::max_element(means_.begin(), means_.end()) * vdda / 4095.0F > 1.85F) {
            reason_ = ZeroReason::range;
        } else if (span_ > 32) {
            reason_ = ZeroReason::noise;
        } else if (*std::max_element(means_.begin(), means_.end()) -
                       *std::min_element(means_.begin(), means_.end()) >
                   80) {
            reason_ = ZeroReason::mismatch;
        }
        if (reason_ == ZeroReason::none) {
            state_ = ZeroState::complete;
        } else if (++rejected_ >= 3) {
            fail(reason_);
        } else {
            clear_window();
        }
    }
    void fail(ZeroReason reason) noexcept {
        if (state_ == ZeroState::collecting) {
            state_ = ZeroState::failed;
            reason_ = reason;
        }
    }
    bool valid() const noexcept {
        return state_ == ZeroState::complete;
    }
    bool failed() const noexcept {
        return state_ == ZeroState::failed;
    }
    ZeroState state() const noexcept {
        return state_;
    }
    ZeroReason reason() const noexcept {
        return reason_;
    }
    const std::array<float, 3>& means() const noexcept {
        return means_;
    }
    std::uint32_t span() const noexcept {
        return span_;
    }
    std::uint32_t elapsed() const noexcept {
        return elapsed_;
    }

  private:
    void clear_window() noexcept {
        count_ = 0;
        sum_ = {};
        minimum_.fill(4095);
        maximum_ = {};
    }
    ZeroState state_{};
    ZeroReason reason_{};
    std::uint32_t elapsed_{}, count_{}, rejected_{}, span_{};
    std::array<std::uint32_t, 3> sum_{};
    std::array<std::uint16_t, 3> minimum_{4095, 4095, 4095}, maximum_{};
    std::array<float, 3> means_{2048, 2048, 2048};
};
} // namespace roller::drivers
