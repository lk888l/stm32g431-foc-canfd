#pragma once
#include <array>
#include <cstdint>
namespace roller::platform {
bool start_pwm() noexcept;
void stop_pwm() noexcept;
// Atomic A/B/C preload submission. False means defer and retain the previous triple.
bool write_pwm(std::array<std::uint16_t, 3> abc) noexcept;
} // namespace roller::platform
