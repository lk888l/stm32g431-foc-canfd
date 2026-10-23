#pragma once
#include "roller/control/math.hpp"
namespace roller::platform {
bool initialize_cordic() noexcept;
// Sole runtime owner: control ISR. Input is finite and in [0, 2*pi]. Uses
// DMA1 channels 5/6 with a bounded wait because the result is needed now.
control::SinCos cordic_sincos(float angle) noexcept;
} // namespace roller::platform
