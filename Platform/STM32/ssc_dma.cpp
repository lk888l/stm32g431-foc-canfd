#include "ssc_dma.hpp"
#include "main.h"
#include "spi.h"

// SWD-only diagnostics. All durations include peripheral transfer time and
// interrupt latency, and use the same 160-MHz DWT counter as the control loop.
struct SscDmaStats {
    volatile std::uint32_t frames{}, errors{}, max_total{}, max_command{}, max_response{};
    volatile std::uint32_t frame_start{}, command_start{}, response_start{};
};
extern "C" {
SscDmaStats roller_ssc_dma_stats;
}

namespace roller::platform {
namespace {
SscDma* owner{};
constexpr std::uint32_t dma_flags = DMA_IFCR_CGIF2 | DMA_IFCR_CGIF3;
constexpr std::uint32_t spi_errors = SPI_SR_OVR | SPI_SR_MODF | SPI_SR_FRE;
constexpr std::uint32_t dma_word = DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0 | DMA_CCR_MINC | DMA_CCR_PL_1;

void maximum(volatile std::uint32_t& field, std::uint32_t value) noexcept {
    if (value > field) {
        field = value;
    }
}

void record_frame(bool ok, bool response) noexcept {
    auto& stats = roller_ssc_dma_stats;
    const auto now = DWT->CYCCNT;
    stats.frames = stats.frames + 1;
    if (!ok) {
        stats.errors = stats.errors + 1;
    }
    maximum(stats.max_total, now - stats.frame_start);
    if (response) {
        maximum(stats.max_response, now - stats.response_start);
    } else {
        maximum(stats.max_command, now - stats.command_start);
    }
}

void delay_cycles(std::uint32_t cycles) noexcept {
    const auto begin = DWT->CYCCNT;
    while (static_cast<std::uint32_t>(DWT->CYCCNT - begin) < cycles) {
        __NOP();
    }
}

bool wait_idle() noexcept {
    // RX TC occurs at the end of a word. Only the peripheral's trailing BSY
    // deassertion is awaited here; the words themselves are transferred by DMA.
    const auto begin = DWT->CYCCNT;
    while (SPI1->SR & SPI_SR_BSY) {
        if (static_cast<std::uint32_t>(DWT->CYCCNT - begin) > 160) {
            return false;
        }
    }
    return true;
}

void stop_dma() noexcept {
    CLEAR_BIT(SPI1->CR2, SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
    CLEAR_BIT(DMA1_Channel2->CCR, DMA_CCR_EN);
    CLEAR_BIT(DMA1_Channel3->CCR, DMA_CCR_EN);
    __DSB();
    DMA1->IFCR = dma_flags;
}
} // namespace

bool SscDma::initialize() noexcept {
    if ((owner && owner != this) || busy() || hspi1.Instance != SPI1 ||
        hspi1.Init.DataSize != SPI_DATASIZE_16BIT || hspi1.Init.Mode != SPI_MODE_MASTER ||
        hspi1.Init.Direction != SPI_DIRECTION_2LINES || hspi1.Init.CLKPolarity != SPI_POLARITY_LOW ||
        hspi1.Init.CLKPhase != SPI_PHASE_2EDGE) {
        return false;
    }
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_SPI1_CLK_ENABLE();
    SET_BIT(CoreDebug->DEMCR, CoreDebug_DEMCR_TRCENA_Msk);
    SET_BIT(DWT->CTRL, DWT_CTRL_CYCCNTENA_Msk);
    const auto irq = __get_PRIMASK();
    __disable_irq();
    stop_dma();
    DMAMUX1_Channel1->CCR = DMA_REQUEST_SPI1_RX;
    DMAMUX1_Channel2->CCR = DMA_REQUEST_SPI1_TX;
    DMAMUX1_ChannelStatus->CFR = DMAMUX_CFR_CSOF1 | DMAMUX_CFR_CSOF2;
    DMA1_Channel2->CPAR = reinterpret_cast<std::uint32_t>(&SPI1->DR);
    DMA1_Channel2->CCR = dma_word | DMA_CCR_TCIE | DMA_CCR_TEIE;
    DMA1_Channel3->CPAR = reinterpret_cast<std::uint32_t>(&SPI1->DR);
    DMA1_Channel3->CCR = dma_word | DMA_CCR_DIR | DMA_CCR_TEIE;
    mosi_mode_ = GPIOA->MODER & GPIO_MODER_MODE7;
    spi_cr1_ = SPI1->CR1 & ~SPI_CR1_SPE;
    spi_cr2_ = SPI1->CR2 & ~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
    owner = this;
    initialized_ = true;
    release(true);
    NVIC_SetPriority(DMA1_Channel2_IRQn, 2);
    NVIC_SetPriority(DMA1_Channel3_IRQn, 2);
    NVIC_EnableIRQ(DMA1_Channel2_IRQn);
    NVIC_EnableIRQ(DMA1_Channel3_IRQn);
    __set_PRIMASK(irq);
    return true;
}

bool SscDma::start(std::uint16_t command, Completion done) noexcept {
    if (!initialized_ || !done) {
        return false;
    }
    const auto irq = __get_PRIMASK();
    __disable_irq();
    if (busy()) {
        __set_PRIMASK(irq);
        return false;
    }
    roller_ssc_dma_stats.frame_start = DWT->CYCCNT;
    roller_ssc_dma_stats.command_start = roller_ssc_dma_stats.frame_start;
    roller_ssc_dma_stats.response_start = 0;
    // Enforce the SSC inter-frame minimum even for back-to-back startup reads.
    const auto elapsed = static_cast<std::uint32_t>(DWT->CYCCNT - last_cs_high_);
    if (elapsed < 100) {
        delay_cycles(100 - elapsed);
    }
    command_ = command;
    done_ = done;
    phase_.store(Phase::command, std::memory_order_release);
    ENC_CS_GPIO_Port->BSRR = static_cast<std::uint32_t>(ENC_CS_Pin) << 16;
    delay_cycles(60);
    arm(&discard_, &command_, 1);
    __set_PRIMASK(irq);
    return true;
}

void SscDma::arm(volatile std::uint16_t* receive, const std::uint16_t* transmit,
                 std::uint16_t count) noexcept {
    // The previous RX completion or release() already disabled both channels
    // and cleared their flags. Width, direction and peripheral address remain
    // fixed across both phases and across frames; only buffers/counts change.
    DMA1_Channel2->CMAR = reinterpret_cast<std::uint32_t>(receive);
    DMA1_Channel2->CNDTR = count;
    DMA1_Channel3->CMAR = reinterpret_cast<std::uint32_t>(transmit);
    DMA1_Channel3->CNDTR = count;
    __DMB();
    // Arm RX before TX can produce the first clock edge.
    SET_BIT(DMA1_Channel2->CCR, DMA_CCR_EN);
    SET_BIT(SPI1->CR2, SPI_CR2_RXDMAEN);
    SET_BIT(DMA1_Channel3->CCR, DMA_CCR_EN);
    SET_BIT(SPI1->CR2, SPI_CR2_TXDMAEN);
}

void SscDma::receive_irq() noexcept {
    const auto flags = DMA1->ISR;
    if (!busy()) {
        DMA1->IFCR = DMA_IFCR_CGIF2;
        return;
    }
    if (flags & (DMA_ISR_TEIF2 | DMA_ISR_TEIF3)) {
        finish(false);
        return;
    }
    if (!(flags & DMA_ISR_TCIF2)) {
        return;
    }
    const bool transferred = DMA1_Channel2->CNDTR == 0 && DMA1_Channel3->CNDTR == 0;
    stop_dma();
    if (!transferred || !wait_idle() || (SPI1->SR & (spi_errors | SPI_SR_FTLVL | SPI_SR_FRLVL))) {
        finish(false);
        return;
    }
    __DMB();
    if (phase_.load(std::memory_order_relaxed) == Phase::command) {
        const auto now = DWT->CYCCNT;
        maximum(roller_ssc_dma_stats.max_command, now - roller_ssc_dma_stats.command_start);
        roller_ssc_dma_stats.response_start = now;
        MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE7, 0); // Release the shared SSC DATA wire.
        delay_cycles(32);                              // 200 ns at 160 MHz; encoder requires at least 130 ns.
        phase_.store(Phase::response, std::memory_order_release);
        arm(response_, dummy_, 2);
        return;
    }
    finish(true);
}

void SscDma::transmit_irq() noexcept {
    if (DMA1->ISR & DMA_ISR_TEIF3) {
        if (busy()) {
            finish(false);
        } else {
            DMA1->IFCR = DMA_IFCR_CGIF3;
        }
    }
}

void SscDma::release(bool reset_spi) noexcept {
    if (reset_spi) {
        stop_dma();
        NVIC_ClearPendingIRQ(DMA1_Channel2_IRQn);
        NVIC_ClearPendingIRQ(DMA1_Channel3_IRQn);
        // Reset also flushes a queued TX word and terminates a stuck BSY before
        // CS/PA7 are restored. No failed frame can clock into the next frame.
        __HAL_RCC_SPI1_FORCE_RESET();
        __DSB();
        __HAL_RCC_SPI1_RELEASE_RESET();
        SPI1->CR1 = spi_cr1_;
        SPI1->CR2 = spi_cr2_;
    }
    // On success receive_irq() has already stopped DMA and observed BSY=0.
    ENC_CS_GPIO_Port->BSRR = ENC_CS_Pin;
    MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE7, mosi_mode_);
    last_cs_high_ = DWT->CYCCNT;
    SET_BIT(SPI1->CR1, SPI_CR1_SPE);
    done_ = nullptr;
    phase_.store(Phase::idle, std::memory_order_release);
}

void SscDma::finish(bool ok) noexcept {
    const auto done = done_;
    __DMB();
    const std::uint16_t value = ok ? response_[0] : 0;
    const std::uint16_t safety = ok ? response_[1] : 0;
    record_frame(ok, phase_.load(std::memory_order_relaxed) == Phase::response);
    release(!ok);
    if (done) {
        done(value, safety, ok);
    }
}

void SscDma::abort() noexcept {
    if (!initialized_) {
        return;
    }
    const auto irq = __get_PRIMASK();
    __disable_irq();
    if (busy()) {
        record_frame(false, phase_.load(std::memory_order_relaxed) == Phase::response);
    }
    release(true);
    __set_PRIMASK(irq);
}
} // namespace roller::platform

extern "C" void DMA1_Channel2_IRQHandler(void) {
    if (roller::platform::owner) {
        roller::platform::owner->receive_irq();
    } else {
        DMA1->IFCR = DMA_IFCR_CGIF2;
    }
}

extern "C" void DMA1_Channel3_IRQHandler(void) {
    if (roller::platform::owner) {
        roller::platform::owner->transmit_irq();
    } else {
        DMA1->IFCR = DMA_IFCR_CGIF3;
    }
}
