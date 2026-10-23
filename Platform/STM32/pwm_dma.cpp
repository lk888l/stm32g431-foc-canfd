#include "pwm_dma.hpp"
#include "stm32g4xx_hal.h"

extern "C" {
std::uint32_t roller_pwm_dma_count{}, roller_pwm_dma_errors{};
}
namespace roller::platform {
namespace {
constexpr auto dma_configuration =
    DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0 | DMA_CCR_PL_1;
constexpr std::uint32_t transfer_timeout_cycles = 320; // 2 us at 160 MHz.
alignas(4) std::array<std::uint16_t, 3> compare_values{2000, 2000, 2000};
bool initialized{}, running{}, pending{}, failed{};

void stop_transfer() noexcept {
    CLEAR_BIT(TIM1->DIER, TIM_DIER_COMDE);
    CLEAR_BIT(DMA1_Channel4->CCR, DMA_CCR_EN);
    __DSB();
}

bool fail_transfer() noexcept {
    stop_transfer();
    if (!failed) {
        ++roller_pwm_dma_errors;
    }
    failed = true;
    return false;
}
} // namespace

bool initialize_pwm_dma() noexcept {
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    initialized = true;
    stop_pwm_dma();
    // COM changes no output configuration while CCPC is disabled. Its DMA
    // burst fills only the three duty preloads; CCR4 keeps the ADC trigger.
    CLEAR_BIT(TIM1->CR2, TIM_CR2_CCPC);
    TIM1->DCR = TIM_DMABASE_CCR1 | TIM_DMABURSTLENGTH_3TRANSFERS;
    DMAMUX1_Channel3->CCR = DMA_REQUEST_TIM1_COM;
    DMA1_Channel4->CCR = dma_configuration;
    DMA1_Channel4->CPAR = reinterpret_cast<std::uint32_t>(&TIM1->DMAR);
    DMA1_Channel4->CMAR = reinterpret_cast<std::uint32_t>(compare_values.data());
    DMA1_Channel4->CNDTR = 0;
    return true;
}

bool start_pwm_dma() noexcept {
    if (!initialized) {
        return false;
    }
    stop_pwm_dma();
    running = true;
    return write_pwm_dma({2000, 2000, 2000});
}

void stop_pwm_dma() noexcept {
    if (!initialized) {
        return;
    }
    running = false;
    stop_transfer();
    DMA1->IFCR = DMA_IFCR_CGIF4;
    TIM1->SR = ~TIM_SR_COMIF;
    pending = false;
    failed = false;
}

bool write_pwm_dma(std::array<std::uint16_t, 3> cba) noexcept {
    if (!running || !pwm_dma_ok()) {
        return false;
    }
    if (pending && (!(DMA1->ISR & DMA_ISR_TCIF4) || DMA1_Channel4->CNDTR != 0)) {
        return fail_transfer();
    }
    CLEAR_BIT(TIM1->DIER, TIM_DIER_COMDE);
    CLEAR_BIT(DMA1_Channel4->CCR, DMA_CCR_EN);
    compare_values = cba;
    DMA1->IFCR = DMA_IFCR_CGIF4;
    TIM1->SR = ~TIM_SR_COMIF;
    DMA1_Channel4->CNDTR = static_cast<std::uint32_t>(compare_values.size());
    __DMB();
    pending = true;
    SET_BIT(DMA1_Channel4->CCR, DMA_CCR_EN);
    SET_BIT(TIM1->DIER, TIM_DIER_COMDE);
    // A software COM event avoids the extra PWM period of latency incurred
    // when an update event starts the burst after latching its preloads.
    TIM1->EGR = TIM_EGR_COMG;
    const auto start = DWT->CYCCNT;
    // Wait only for the three DMA transfers, never for a timer update. Board
    // can enable PB2 after success knowing all preloads reached the timer.
    std::uint32_t remaining = transfer_timeout_cycles;
    for (;;) {
        const auto flags = DMA1->ISR;
        if (flags & DMA_ISR_TEIF4) {
            return fail_transfer();
        }
        if ((flags & DMA_ISR_TCIF4) && DMA1_Channel4->CNDTR == 0) {
            stop_transfer();
            ++roller_pwm_dma_count;
            // Preserve TC until the next submission verifies completion.
            return true;
        }
        if (DWT->CYCCNT - start > transfer_timeout_cycles || --remaining == 0) {
            return fail_transfer();
        }
    }
}

bool pwm_dma_ok() noexcept {
    if (initialized && (DMA1->ISR & DMA_ISR_TEIF4)) {
        return fail_transfer();
    }
    return !failed;
}
} // namespace roller::platform
