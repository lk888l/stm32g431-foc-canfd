#include "cordic_math.hpp"
#include "stm32g4xx_ll_cordic.h"
#include <bit>
#include <limits>

extern "C" {
float roller_cordic_max_error{};
}
namespace roller::platform {
control::SinCos cordic_sincos(float angle) noexcept {
    constexpr float scale = 2147483648.0F / control::pi;
    constexpr float inverse = 1.0F / 2147483648.0F;
    const auto invalid = [] {
        const auto nan = std::numeric_limits<float>::quiet_NaN();
        return control::SinCos{nan, nan};
    };
    if (!std::isfinite(angle) || angle < 0 || angle > control::tau) {
        return invalid();
    }
    if (angle >= control::pi) {
        angle -= control::tau;
    }
    const auto q31 = static_cast<std::int32_t>(angle * scale);
    LL_CORDIC_WriteData(CORDIC, std::bit_cast<std::uint32_t>(q31));
    LL_CORDIC_WriteData(CORDIC, 0x7FFFFFFFU);
    const auto start = DWT->CYCCNT;
    while (!LL_CORDIC_IsActiveFlag_RRDY(CORDIC)) {
        if (DWT->CYCCNT - start > 160) {
            return invalid();
        }
    }
    const auto cosine = std::bit_cast<std::int32_t>(LL_CORDIC_ReadData(CORDIC));
    const auto sine = std::bit_cast<std::int32_t>(LL_CORDIC_ReadData(CORDIC));
    return {static_cast<float>(sine) * inverse, static_cast<float>(cosine) * inverse};
}
bool initialize_cordic() noexcept {
    // CubeMX enables/reset-initializes the peripheral; algorithm mode belongs
    // to this adapter. Six cycles = 24 iterations, both results in Q1.31.
    LL_CORDIC_Config(CORDIC, LL_CORDIC_FUNCTION_COSINE, LL_CORDIC_PRECISION_6CYCLES, LL_CORDIC_SCALE_0,
                     LL_CORDIC_NBWRITE_2, LL_CORDIC_NBREAD_2, LL_CORDIC_INSIZE_32BITS,
                     LL_CORDIC_OUTSIZE_32BITS);
    for (unsigned i = 0; i <= 256; ++i) {
        const auto angle = control::tau * static_cast<float>(i) / 256.0F;
        const auto actual = cordic_sincos(angle), expected = control::software_sincos(angle);
        const auto error =
            std::max(std::abs(actual.sine - expected.sine), std::abs(actual.cosine - expected.cosine));
        if (!std::isfinite(error) || error > 2e-6F) {
            return false;
        }
        roller_cordic_max_error = std::max(roller_cordic_max_error, error);
    }
    return true;
}
} // namespace roller::platform
