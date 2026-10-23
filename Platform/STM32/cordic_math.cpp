#include "cordic_math.hpp"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_cordic.h"
#include <array>
#include <bit>
#include <limits>

extern "C" {
float roller_cordic_max_error{};
std::uint32_t roller_cordic_dma_count{}, roller_cordic_dma_errors{};
}
namespace roller::platform {
namespace {
constexpr auto dma_read_configuration = DMA_CCR_MINC | DMA_CCR_PSIZE_1 | DMA_CCR_MSIZE_1 | DMA_CCR_PL_0;
constexpr auto dma_write_configuration = dma_read_configuration | DMA_CCR_DIR;
constexpr std::uint32_t dma_timeout_cycles = 800; // 5 us at 160 MHz, including bus arbitration.
alignas(4) std::array<std::uint32_t, 2> arguments{}, results{};
bool initialized{};

void stop_dma() noexcept {
    CLEAR_BIT(CORDIC->CSR, CORDIC_CSR_DMAREN | CORDIC_CSR_DMAWEN);
    CLEAR_BIT(DMA1_Channel5->CCR, DMA_CCR_EN);
    CLEAR_BIT(DMA1_Channel6->CCR, DMA_CCR_EN);
    __DSB();
}
} // namespace

control::SinCos cordic_sincos(float angle) noexcept {
    constexpr float scale = 2147483648.0F / control::pi;
    constexpr float inverse = 1.0F / 2147483648.0F;
    const auto invalid = [] {
        const auto nan = std::numeric_limits<float>::quiet_NaN();
        return control::SinCos{nan, nan};
    };
    if (!initialized || !std::isfinite(angle) || angle < 0 || angle > control::tau) {
        return invalid();
    }
    if (angle >= control::pi) {
        angle -= control::tau;
    }
    const auto q31 = static_cast<std::int32_t>(angle * scale);
    arguments = {std::bit_cast<std::uint32_t>(q31), 0x7FFFFFFFU};
    DMA1->IFCR = DMA_IFCR_CGIF5 | DMA_IFCR_CGIF6;
    DMA1_Channel5->CNDTR = static_cast<std::uint32_t>(results.size());
    DMA1_Channel6->CNDTR = static_cast<std::uint32_t>(arguments.size());
    __DMB();
    // Arm result reception before supplying the inputs. Both channels stop
    // after two words; their persistent buffers outlive the entire transfer.
    SET_BIT(DMA1_Channel5->CCR, DMA_CCR_EN);
    SET_BIT(DMA1_Channel6->CCR, DMA_CCR_EN);
    SET_BIT(CORDIC->CSR, CORDIC_CSR_DMAREN | CORDIC_CSR_DMAWEN);
    const auto start = DWT->CYCCNT;
    std::uint32_t remaining = dma_timeout_cycles;
    for (;;) {
        const auto flags = DMA1->ISR;
        if ((flags & (DMA_ISR_TEIF5 | DMA_ISR_TEIF6)) || DWT->CYCCNT - start > dma_timeout_cycles ||
            --remaining == 0) {
            stop_dma();
            // A partial operation may leave the CORDIC input/result phases
            // incomplete. Require reinitialization instead of reusing it.
            initialized = false;
            ++roller_cordic_dma_errors;
            return invalid();
        }
        if ((flags & (DMA_ISR_TCIF5 | DMA_ISR_TCIF6)) == (DMA_ISR_TCIF5 | DMA_ISR_TCIF6) &&
            DMA1_Channel5->CNDTR == 0 && DMA1_Channel6->CNDTR == 0) {
            break;
        }
    }
    stop_dma();
    __DMB();
    const auto cosine = std::bit_cast<std::int32_t>(results[0]);
    const auto sine = std::bit_cast<std::int32_t>(results[1]);
    ++roller_cordic_dma_count;
    return {static_cast<float>(sine) * inverse, static_cast<float>(cosine) * inverse};
}
bool initialize_cordic() noexcept {
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    stop_dma();
    __HAL_RCC_CORDIC_FORCE_RESET();
    __HAL_RCC_CORDIC_RELEASE_RESET();
    DMA1->IFCR = DMA_IFCR_CGIF5 | DMA_IFCR_CGIF6;
    DMAMUX1_Channel4->CCR = DMA_REQUEST_CORDIC_READ;
    DMAMUX1_Channel5->CCR = DMA_REQUEST_CORDIC_WRITE;
    DMA1_Channel5->CCR = dma_read_configuration;
    DMA1_Channel5->CPAR = reinterpret_cast<std::uint32_t>(&CORDIC->RDATA);
    DMA1_Channel5->CMAR = reinterpret_cast<std::uint32_t>(results.data());
    DMA1_Channel6->CCR = dma_write_configuration;
    DMA1_Channel6->CPAR = reinterpret_cast<std::uint32_t>(&CORDIC->WDATA);
    DMA1_Channel6->CMAR = reinterpret_cast<std::uint32_t>(arguments.data());
    // CubeMX enables/reset-initializes the peripheral; algorithm mode belongs
    // to this adapter. Six cycles = 24 iterations, both results in Q1.31.
    LL_CORDIC_Config(CORDIC, LL_CORDIC_FUNCTION_COSINE, LL_CORDIC_PRECISION_6CYCLES, LL_CORDIC_SCALE_0,
                     LL_CORDIC_NBWRITE_2, LL_CORDIC_NBREAD_2, LL_CORDIC_INSIZE_32BITS,
                     LL_CORDIC_OUTSIZE_32BITS);
    initialized = true;
    roller_cordic_max_error = 0;
    for (unsigned i = 0; i <= 256; ++i) {
        const auto angle = control::tau * static_cast<float>(i) / 256.0F;
        const auto actual = cordic_sincos(angle), expected = control::software_sincos(angle);
        const auto error =
            std::max(std::abs(actual.sine - expected.sine), std::abs(actual.cosine - expected.cosine));
        if (!std::isfinite(error) || error > 2e-6F) {
            initialized = false;
            return false;
        }
        roller_cordic_max_error = std::max(roller_cordic_max_error, error);
    }
    return true;
}
} // namespace roller::platform
