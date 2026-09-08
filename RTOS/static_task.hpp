#pragma once
#include "FreeRTOS.h"
#include "roller/core/module.hpp"
#include "task.h"
#include <array>
namespace roller::rtos {
inline void wait_period(TickType_t& wake, TickType_t period) noexcept {
    if (xTaskDelayUntil(&wake, period) == pdFALSE) {
        // Skip missed releases after bounded flash maintenance. Catch-up loops
        // would publish duplicate commands and falsely report stalled actors.
        wake = xTaskGetTickCount();
        (void)xTaskDelayUntil(&wake, period);
    }
}
// Static task storage; the wrapper is deliberately non-movable and non-copyable.
// Firmware tasks live until reset and never refer to a stack-local owner.
template <std::size_t Words> class StaticTask {
  public:
    StaticTask() = default;
    StaticTask(const StaticTask&) = delete;
    StaticTask& operator=(const StaticTask&) = delete;
    [[nodiscard]] Result start(TaskFunction_t entry, void* context, const char* name,
                               UBaseType_t priority) noexcept {
        if (handle_) {
            return std::unexpected(Error::invalid_state);
        }
        handle_ = xTaskCreateStatic(entry, name, Words, context, priority, stack_.data(), &tcb_);
        if (!handle_) {
            return std::unexpected(Error::capacity);
        }
        return {};
    }
    [[nodiscard]] UBaseType_t free_stack_words() const noexcept {
        return uxTaskGetStackHighWaterMark(handle_);
    }

  private:
    std::array<StackType_t, Words> stack_{};
    StaticTask_t tcb_{};
    TaskHandle_t handle_{};
};
} // namespace roller::rtos
