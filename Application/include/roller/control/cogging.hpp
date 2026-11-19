#pragma once
#include "roller/control/cogging_map.hpp"
#include "roller/control/math.hpp"

namespace roller::control {
inline float cogging_current(float raw_angle, float offset, unsigned poles, int direction) noexcept {
    if (!std::isfinite(raw_angle) || !std::isfinite(offset) || poles != cogging::pole_pairs ||
        direction != cogging::direction || std::abs(wrap_signed(offset - cogging::reference_offset)) > 0.1F) {
        return 0;
    }
    const float phase = wrap(static_cast<float>(direction) * raw_angle) * (1024.0F / tau);
    const auto index = static_cast<unsigned>(phase);
    const float fraction = phase - static_cast<float>(index);
    const float a = static_cast<float>(cogging::current_100ua[index & 1023U]);
    const float b = static_cast<float>(cogging::current_100ua[(index + 1U) & 1023U]);
    return (a + fraction * (b - a)) * (0.0001F * cogging::gain);
}
} // namespace roller::control
