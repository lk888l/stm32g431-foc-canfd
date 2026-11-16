#include "ssi_dma.hpp"
#include "main.h"
#include "spi.h"
struct SsiDmaStats {
    volatile std::uint32_t frames{}, errors{}, max_cycles{}, start{};
};
extern "C" {
SsiDmaStats roller_ssi_dma_stats;
}
namespace roller::platform {
namespace {
SsiDma* owner{};
void delay(std::uint32_t cycles) {
    const auto begin = DWT->CYCCNT;
    while (DWT->CYCCNT - begin < cycles) {
        __NOP();
    }
}
void stop_dma() {
    CLEAR_BIT(SPI3->CR2, SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
    CLEAR_BIT(DMA1_Channel2->CCR, DMA_CCR_EN);
    CLEAR_BIT(DMA1_Channel3->CCR, DMA_CCR_EN);
    __DSB();
    DMA1->IFCR = DMA_IFCR_CGIF2 | DMA_IFCR_CGIF3;
}
} // namespace
bool SsiDma::initialize() noexcept {
    if ((owner && owner != this) || hspi3.Instance != SPI3 || hspi3.Init.DataSize != SPI_DATASIZE_8BIT ||
        hspi3.Init.CLKPolarity != SPI_POLARITY_LOW || hspi3.Init.CLKPhase != SPI_PHASE_2EDGE) {
        return false;
    }
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    stop_dma();
    DMAMUX1_Channel1->CCR = DMA_REQUEST_SPI3_RX;
    DMAMUX1_Channel2->CCR = DMA_REQUEST_SPI3_TX;
    DMA1_Channel2->CPAR = reinterpret_cast<std::uint32_t>(&SPI3->DR);
    DMA1_Channel3->CPAR = reinterpret_cast<std::uint32_t>(&SPI3->DR);
    DMA1_Channel2->CCR = DMA_CCR_MINC | DMA_CCR_PL_1 | DMA_CCR_TCIE | DMA_CCR_TEIE;
    DMA1_Channel3->CCR = DMA_CCR_MINC | DMA_CCR_PL_1 | DMA_CCR_DIR | DMA_CCR_TEIE;
    cr1_ = SPI3->CR1 & ~SPI_CR1_SPE;
    cr2_ = (SPI3->CR2 & ~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN)) | SPI_CR2_FRXTH;
    owner = this;
    initialized_ = true;
    release(true);
    NVIC_SetPriority(DMA1_Channel2_IRQn, 2);
    NVIC_SetPriority(DMA1_Channel3_IRQn, 2);
    NVIC_EnableIRQ(DMA1_Channel2_IRQn);
    NVIC_EnableIRQ(DMA1_Channel3_IRQn);
    return true;
}
bool SsiDma::start(Completion done) noexcept {
    if (!initialized_ || !done || busy()) {
        return false;
    }
    // 24 clocks under one CS, no command and no DATA turnaround.
    const auto elapsed = DWT->CYCCNT - last_cs_high_;
    if (elapsed < 160) {
        delay(160 - elapsed);
    }
    done_ = done;
    busy_.store(true, std::memory_order_release);
    roller_ssi_dma_stats.start = DWT->CYCCNT;
    ENC_CS_GPIO_Port->BSRR = static_cast<std::uint32_t>(ENC_CS_Pin) << 16;
    delay(32);
    DMA1_Channel2->CMAR = reinterpret_cast<std::uint32_t>(response_);
    DMA1_Channel3->CMAR = reinterpret_cast<std::uint32_t>(dummy_);
    DMA1_Channel2->CNDTR = 3;
    DMA1_Channel3->CNDTR = 3;
    __DMB();
    SET_BIT(DMA1_Channel2->CCR, DMA_CCR_EN);
    SET_BIT(SPI3->CR2, SPI_CR2_RXDMAEN);
    SET_BIT(DMA1_Channel3->CCR, DMA_CCR_EN);
    SET_BIT(SPI3->CR2, SPI_CR2_TXDMAEN);
    return true;
}
void SsiDma::irq() noexcept {
    const auto flags = DMA1->ISR;
    if (!busy()) {
        stop_dma();
        return;
    }
    if (flags & (DMA_ISR_TEIF2 | DMA_ISR_TEIF3)) {
        finish(false);
        return;
    }
    if (!(flags & DMA_ISR_TCIF2)) {
        return;
    }
    const auto begin = DWT->CYCCNT;
    while (SPI3->SR & SPI_SR_BSY) {
        if (DWT->CYCCNT - begin > 160) {
            finish(false);
            return;
        }
    }
    finish(DMA1_Channel2->CNDTR == 0 && DMA1_Channel3->CNDTR == 0 &&
           !(SPI3->SR & (SPI_SR_OVR | SPI_SR_MODF | SPI_SR_FRE)));
}
void SsiDma::release(bool reset) noexcept {
    stop_dma();
    if (reset) {
        __HAL_RCC_SPI3_FORCE_RESET();
        __DSB();
        __HAL_RCC_SPI3_RELEASE_RESET();
        SPI3->CR1 = cr1_;
        SPI3->CR2 = cr2_;
    }
    delay(16); // >= half a 5-MHz clock after the last falling edge.
    ENC_CS_GPIO_Port->BSRR = ENC_CS_Pin;
    last_cs_high_ = DWT->CYCCNT;
    SET_BIT(SPI3->CR1, SPI_CR1_SPE);
    done_ = nullptr;
    busy_.store(false, std::memory_order_release);
}
void SsiDma::finish(bool ok) noexcept {
    const auto done = done_;
    __DMB();
    const std::uint32_t frame = (static_cast<std::uint32_t>(response_[0]) << 16) |
                                (static_cast<std::uint32_t>(response_[1]) << 8) | response_[2];
    const auto elapsed = DWT->CYCCNT - roller_ssi_dma_stats.start;
    roller_ssi_dma_stats.frames = roller_ssi_dma_stats.frames + 1;
    if (!ok) {
        roller_ssi_dma_stats.errors = roller_ssi_dma_stats.errors + 1;
    }
    if (elapsed > roller_ssi_dma_stats.max_cycles) {
        roller_ssi_dma_stats.max_cycles = elapsed;
    }
    release(!ok);
    if (done) {
        done(frame, ok);
    }
}
void SsiDma::abort() noexcept {
    if (!initialized_) {
        return;
    }
    const auto mask = __get_PRIMASK();
    __disable_irq();
    release(true);
    NVIC_ClearPendingIRQ(DMA1_Channel2_IRQn);
    NVIC_ClearPendingIRQ(DMA1_Channel3_IRQn);
    __set_PRIMASK(mask);
}
} // namespace roller::platform
extern "C" void DMA1_Channel2_IRQHandler(void) {
    if (roller::platform::owner) {
        roller::platform::owner->irq();
    } else {
        DMA1->IFCR = DMA_IFCR_CGIF2;
    }
}
extern "C" void DMA1_Channel3_IRQHandler(void) {
    if (roller::platform::owner) {
        roller::platform::owner->irq();
    } else {
        DMA1->IFCR = DMA_IFCR_CGIF3;
    }
}
