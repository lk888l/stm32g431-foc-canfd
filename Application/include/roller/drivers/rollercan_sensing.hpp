#pragma once
#include <cstdint>

namespace roller::drivers {
// simple-foc v2.0: R_DV1=47k, R_DV2=4.7k; DRV8316CT GAIN tied to GND.
inline constexpr float bus_divider = 11.0F;
inline constexpr float current_volts_per_amp = 0.15F;
constexpr float bus_voltage(std::uint16_t raw, float vdda) noexcept {
    return static_cast<float>(raw) * vdda / 4095.0F * bus_divider;
}
} // namespace roller::drivers
