#include "pwm.hpp"
#include "roller/drivers/current_window.hpp"
#include "stm32g4xx_hal.h"
extern "C" {
std::uint32_t roller_pwm_updates{}, roller_pwm_deferred{};
}
namespace roller::platform {
namespace {
bool running{};
}
bool start_pwm() noexcept {
    running = true;
    return true;
}
void stop_pwm() noexcept {
    running = false;
}
bool write_pwm(std::array<std::uint16_t, 3> abc) noexcept {
    if (!running) {
        return false;
    }
    const auto mask = __get_PRIMASK();
    __disable_irq();
    // Called at underflow after the previous preload has latched. The new
    // triple has a full PWM period before its latch, independent of ADC WCET.
    const auto count = TIM1->CNT;
    if (!drivers::pwm_update_window(count, (TIM1->CR1 & TIM_CR1_DIR) != 0, (TIM1->SR & TIM_SR_UIF) != 0)) {
        ++roller_pwm_deferred;
        __set_PRIMASK(mask);
        return false;
    }
    // UDIS prevents a partial triple even in the presence of bus stalls.
    SET_BIT(TIM1->CR1, TIM_CR1_UDIS);
    TIM1->CCR1 = abc[0];
    TIM1->CCR2 = abc[1];
    TIM1->CCR3 = abc[2];
    __DSB();
    CLEAR_BIT(TIM1->CR1, TIM_CR1_UDIS);
    ++roller_pwm_updates;
    __set_PRIMASK(mask);
    return true;
}
} // namespace roller::platform
