#pragma once
#include <cstdint>

namespace roller::drivers {
// Unit-RollerCAN V1.0 schematic: R18=12 kOhm, R19=2.2 kOhm.
inline constexpr float bus_divider = (12000.0F + 2200.0F) / 2200.0F;
constexpr float bus_voltage(std::uint16_t raw, float vdda) noexcept {
    return static_cast<float>(raw) * vdda / 4095.0F * bus_divider;
}
} // namespace roller::drivers
