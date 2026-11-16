#include "cordic_math.hpp"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_cordic.h"
#include <bit>
#include <limits>
extern "C" {
float roller_cordic_max_error{};
std::uint32_t roller_cordic_calls{}, roller_cordic_errors{};
}
namespace roller::platform {
namespace {
bool initialized{};
}
control::SinCos cordic_sincos(float angle) noexcept {
    if (!initialized || !std::isfinite(angle)) {
        return control::software_sincos(angle);
    }
    angle = control::wrap_signed(angle);
    const auto q31 = static_cast<std::int32_t>(angle * (2147483648.0F / control::pi));
    CORDIC->WDATA = std::bit_cast<std::uint32_t>(q31);
    CORDIC->WDATA = 0x7FFFFFFFU;
    const auto begin = DWT->CYCCNT;
    while (!(CORDIC->CSR & CORDIC_CSR_RRDY)) {
        if (DWT->CYCCNT - begin > 160) {
            ++roller_cordic_errors;
            initialized = false;
            return control::software_sincos(angle); // Preserve control using libm if accelerator fails.
        }
    }
    const auto cosine = std::bit_cast<std::int32_t>(static_cast<std::uint32_t>(CORDIC->RDATA));
    const auto sine = std::bit_cast<std::int32_t>(static_cast<std::uint32_t>(CORDIC->RDATA));
    ++roller_cordic_calls;
    return {static_cast<float>(sine) / 2147483648.0F, static_cast<float>(cosine) / 2147483648.0F};
}
bool initialize_cordic() noexcept {
    __HAL_RCC_CORDIC_FORCE_RESET();
    __HAL_RCC_CORDIC_RELEASE_RESET();
    LL_CORDIC_Config(CORDIC, LL_CORDIC_FUNCTION_COSINE, LL_CORDIC_PRECISION_6CYCLES, LL_CORDIC_SCALE_0,
                     LL_CORDIC_NBWRITE_2, LL_CORDIC_NBREAD_2, LL_CORDIC_INSIZE_32BITS,
                     LL_CORDIC_OUTSIZE_32BITS);
    initialized = true;
    roller_cordic_max_error = 0;
    for (unsigned i = 0; i <= 256; ++i) {
        const float angle = control::tau * static_cast<float>(i) / 256.0F;
        const auto actual = cordic_sincos(angle), expected = control::software_sincos(angle);
        const auto error =
            std::max(std::abs(actual.sine - expected.sine), std::abs(actual.cosine - expected.cosine));
        roller_cordic_max_error = std::max(roller_cordic_max_error, error);
        if (!std::isfinite(error) || error > 2e-6F) {
            initialized = false;
            return false;
        }
    }
    return true;
}
} // namespace roller::platform
