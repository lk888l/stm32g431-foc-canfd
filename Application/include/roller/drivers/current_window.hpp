#pragma once
#include <array>
#include <cstdint>
namespace roller::drivers {
// TIM1=160 MHz, ARR=4000, trigger=3920. ADC=40 MHz, sample=24.5
// cycles, conversion=12.5. SLEW tied to AVDD: propagation <=1.05 us,
// CSA settling 1 us typical; add 0.15 us allowance. Scope validation remains needed.
constexpr bool current_window_valid(std::uint16_t compare, unsigned rank) noexcept {
    const std::uint32_t start = 3920U + rank * 148U;
    const std::uint32_t end = start + 98U;
    return compare + 352U < start && compare < 8000U - end;
}
constexpr bool pwm_update_window(std::uint32_t count, bool down, bool update_pending) noexcept {
    return down && count >= 320U && !update_pending;
}
} // namespace roller::drivers
