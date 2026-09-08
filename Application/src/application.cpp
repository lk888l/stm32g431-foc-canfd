#include "adc.h"
#include "board.hpp"
#include "cordic_math.hpp"
#include "fdcan.h"
#include "iwdg.h"
#include "main.h"
#include "node.hpp"
#include "roller/app_bridge.h"
#include "roller/core/channels.hpp"
#include "roller/core/progress_watchdog.hpp"
#include "static_task.hpp"
#include "tim.h"
#include <algorithm>
#ifdef ROLLER_BENCH
#include "bench.hpp"
#endif

namespace {
roller::Channels channels;
roller::platform::Board board{channels};
roller::platform::FlashStore storage{board};
#ifdef ROLLER_BENCH
RollerCanTransport can_transport = roller::bench::transport();
#else
RollerCanTransport can_transport = board.can_transport();
#endif
roller::protocol::Node node{can_transport, channels, storage};
roller::drive::Controller controller;
roller::ModuleManager<2> modules;
roller::rtos::StaticTask<1024> can_task;
roller::rtos::StaticTask<256> supervisor_task;
std::uint32_t previous_sample_cycle{};
std::uint16_t previous_controlword{};
struct DebugStatus {
    volatile std::uint32_t panic{}, can_stack_free{}, supervisor_stack_free{}, sample_count{},
        max_isr_cycles{};
    const char* volatile reason{};
};
} // namespace
// Stable SWD-visible symbol for first board bring-up.
extern "C" {
DebugStatus roller_debug{};
}
extern "C" void roller_panic(const char* reason) {
    __disable_irq();
    roller::platform::Board::gate_off();
    GPIOB->BSRR = static_cast<std::uint32_t>(GPIO_PIN_1) << 16;
    roller_debug.panic = 1;
    roller_debug.reason = reason;
    for (;;) {
        __NOP(); // IWDG resets the MCU; debugger may inspect roller_debug.
    }
}
extern "C" void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* adc) {
    if (adc->Instance != ADC1 || !board.sampling()) {
        return;
    }
    const auto start = DWT->CYCCNT;
    const bool active = (GPIOB->ODR & PWM_ENABLE_Pin) != 0;
    if (previous_sample_cycle && start - previous_sample_cycle > 12000 && active) {
        controller.trip(roller::drive::Fault::timing);
    }
    previous_sample_cycle = start;
    if (auto fault = channels.asynchronous_fault.exchange(0, std::memory_order_acq_rel); fault) {
        controller.trip(static_cast<roller::drive::Fault>(fault));
    }
    roller::drive::ValidatedCommand command;
    if (channels.commands.latest(command)) {
        if ((command.get().controlword & 0x80) && !(previous_controlword & 0x80)) {
            board.acknowledge_encoder();
        }
        previous_controlword = command.get().controlword;
        controller.command(command);
    }
    const auto sample = board.sample();
#ifdef ROLLER_BENCH
    const auto sampled = DWT->CYCCNT;
#endif
    const auto output = controller.step(sample);
#ifdef ROLLER_BENCH
    const auto controlled = DWT->CYCCNT;
#endif
    board.apply(output);
#ifdef ROLLER_BENCH
    roller::bench::record(start, sampled, controlled, DWT->CYCCNT);
#endif
    const auto count = channels.sample_count.fetch_add(1, std::memory_order_relaxed) + 1;
    if (count % 20 == 0) {
        (void)channels.telemetry.push(controller.telemetry());
    }
    const auto elapsed = DWT->CYCCNT - start;
    if (elapsed > channels.isr_max_cycles.load(std::memory_order_relaxed)) {
        channels.isr_max_cycles.store(elapsed, std::memory_order_relaxed);
    }
    if (elapsed > 7200) {
        board.gate_off();
        controller.trip(roller::drive::Fault::timing);
    }
}
extern "C" void HAL_ADC_ErrorCallback(ADC_HandleTypeDef*) {
    board.gate_off();
    channels.asynchronous_fault.store(static_cast<std::uint32_t>(roller::drive::Fault::configuration));
}
extern "C" void HAL_GPIO_EXTI_Callback(std::uint16_t pin) {
    if (pin == DRV_FAULT_Pin) {
        board.fault_irq();
    }
}
extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef*, std::uint32_t flags) {
    if (flags & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) {
        board.gate_off();
        channels.asynchronous_fault.store(static_cast<std::uint32_t>(roller::drive::Fault::communication));
    }
    if (flags & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) {
        board.can_irq();
    }
}
extern "C" void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef*, std::uint32_t flags) {
    if (flags & FDCAN_IT_BUS_OFF) {
        board.gate_off();
        channels.asynchronous_fault.store(static_cast<std::uint32_t>(roller::drive::Fault::communication));
    }
}
extern "C" void vApplicationGetIdleTaskMemory(StaticTask_t** task, StackType_t** stack,
                                              configSTACK_DEPTH_TYPE* size) {
    static StaticTask_t tcb;
    static StackType_t memory[configMINIMAL_STACK_SIZE];
    *task = &tcb;
    *stack = memory;
    *size = configMINIMAL_STACK_SIZE;
}
extern "C" void vApplicationStackOverflowHook(TaskHandle_t, char*) {
    roller_panic("stack overflow");
}
extern "C" void roller_app_start() {
    controller = roller::drive::Controller{node.load_settings(), roller::platform::cordic_sincos};
    if (!modules.add(board) || !modules.add(node) || !modules.start()) {
        roller_panic("module initialization");
    }
    if (!board.start_sampling()) {
        roller_panic("sampling initialization");
    }
    channels.inhibit.store(false, std::memory_order_release);
    if (!can_task.start(
            [](void*) {
                auto wake = xTaskGetTickCount();
                auto previous = DWT->CYCCNT;
                for (;;) {
                    // Batch is bounded so a busy bus cannot monopolize this task.
                    RollerCanFrame frame{};
#ifdef ROLLER_BENCH
                    if (roller::bench::receive(frame))
                        node.receive(frame);
#else
                    for (unsigned i = 0; i < 64 && board.receive(frame); ++i) {
                        node.receive(frame);
                    }
#endif
                    const auto now = DWT->CYCCNT;
                    node.process((now - previous) / (SystemCoreClock / 1000000U));
#ifdef ROLLER_BENCH
                    roller::bench::publish(node.telemetry(), channels.isr_max_cycles.load(),
                                           channels.sampling_epoch.load());
#endif
                    previous = now;
                    if (node.reset_requested()) {
                        board.gate_off();
                        NVIC_SystemReset();
                    }
                    roller::rtos::wait_period(wake, pdMS_TO_TICKS(1));
                }
            },
            nullptr, "canopen", 3)) {
        roller_panic("CAN task creation");
    }
    if (!supervisor_task.start(
            [](void*) {
                auto wake = xTaskGetTickCount();
                roller::ProgressWatchdog progress;
                for (;;) {
                    roller::rtos::wait_period(wake, pdMS_TO_TICKS(10));
                    const auto samples = channels.sample_count.load(std::memory_order_relaxed);
                    const auto can = channels.can_task_ticks.load(std::memory_order_relaxed);
                    const auto epoch = channels.sampling_epoch.load(std::memory_order_acquire);
                    if (!progress.observe(samples, can, epoch) || channels.inhibit.load()) {
                        channels.inhibit.store(true);
                        board.gate_off();
                        channels.asynchronous_fault.store(
                            static_cast<std::uint32_t>(roller::drive::Fault::timing));
                    } else if (HAL_IWDG_Refresh(&hiwdg) != HAL_OK) {
                        roller_panic("watchdog refresh");
                    }
                    if (HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin) == GPIO_PIN_RESET) {
                        board.gate_off();
                        channels.asynchronous_fault.store(
                            static_cast<std::uint32_t>(roller::drive::Fault::driver));
                    }
                    roller_debug.sample_count = samples;
                    roller_debug.max_isr_cycles = channels.isr_max_cycles.load();
                    roller_debug.can_stack_free = can_task.free_stack_words();
                    roller_debug.supervisor_stack_free = supervisor_task.free_stack_words();
                }
            },
            nullptr, "supervisor", 2)) {
        roller_panic("supervisor creation");
    }
    vTaskStartScheduler();
    roller_panic("scheduler returned");
}
