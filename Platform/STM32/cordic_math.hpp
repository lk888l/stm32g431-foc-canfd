#pragma once
#include "roller/control/math.hpp"
namespace roller::platform {
bool initialize_cordic() noexcept;
// Sole runtime owner: ADC ISR. Input is finite and in [0, 2*pi].
control::SinCos cordic_sincos(float angle) noexcept;
} // namespace roller::platform
