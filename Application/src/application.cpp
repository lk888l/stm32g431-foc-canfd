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
#include <cstring>
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
std::uint32_t frame_start_cycle{};
void control_ready();
roller::drive::ValidatedCommand incoming_command;
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
    DRV_ENABLE_GPIO_Port->BSRR = static_cast<std::uint32_t>(DRV_ENABLE_Pin) << 16;
    roller_debug.panic = 1;
    roller_debug.reason = reason;
    for (;;) {
        __NOP(); // IWDG resets the MCU; debugger may inspect roller_debug.
    }
}
namespace {
void begin_frame(unsigned half) {
    if (!board.sampling()) {
        return;
    }
    const auto start = DWT->CYCCNT;
    frame_start_cycle = start;
    (void)board.begin_sample(half, control_ready);
}
void control_ready() {
    if (!board.sampling()) {
        return;
    }
    // ADC DMA completes after the SSI transfer. Missing data is predicted;
    // this ISR never waits for an encoder transaction.
    const auto start = frame_start_cycle;
    if (auto fault = channels.asynchronous_fault.exchange(0, std::memory_order_acq_rel); fault) {
        controller.trip(static_cast<roller::drive::Fault>(fault));
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
    roller_hardware.warnings = controller.telemetry().warnings;
}
} // namespace
extern "C" void roller_app_encoder_trigger() {
    if (!board.sampling()) {
        return;
    }
    if (auto fault = channels.asynchronous_fault.exchange(0, std::memory_order_acq_rel); fault) {
        controller.trip(static_cast<roller::drive::Fault>(fault));
    }
    if (channels.commands.latest(incoming_command)) {
        controller.command(incoming_command);
    }
    board.encoder_trigger();
}
extern "C" void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* adc) {
    if (adc->Instance == ADC1) {
        begin_frame(0);
    }
}
extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* adc) {
    if (adc->Instance == ADC1) {
        begin_frame(1);
    }
}
extern "C" void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* adc) {
    // Report and allow the overwritten-data DMA stream to recover next cycle.
    roller_hardware.adc_errors = roller_hardware.adc_errors + 1;
    __HAL_ADC_CLEAR_FLAG(adc, ADC_FLAG_OVR | ADC_FLAG_JQOVF);
}
extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef*, std::uint32_t flags) {
    if (flags & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) {
        channels.communication_errors.fetch_add(1);
    }
    if (flags & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) {
        board.can_irq();
    }
}
extern "C" void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef*, std::uint32_t flags) {
    if (flags & FDCAN_IT_BUS_OFF) {
        channels.communication_errors.fetch_add(1);
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
                    const auto request = roller_hardware_request;
                    if (request != roller_hardware_ack) {
                        const auto mask = __get_PRIMASK();
                        __disable_irq();
                        std::memcpy(
                            &roller_hardware_snapshot,
                            const_cast<const roller::platform::HardwareDiagnostics*>(&roller_hardware),
                            sizeof(roller_hardware_snapshot));
                        __DMB();
                        roller_hardware_ack = request;
                        __set_PRIMASK(mask);
                    }
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
                    static unsigned stalled{};
                    const bool progressing = progress.observe(samples, can, epoch);
                    stalled = progressing ? 0U : stalled + 1U;
                    if (stalled >= 20 || channels.inhibit.load()) {
                        channels.inhibit.store(true);
                        board.gate_off();
                        channels.asynchronous_fault.store(
                            static_cast<std::uint32_t>(roller::drive::Fault::timing));
                    } else if (HAL_IWDG_Refresh(&hiwdg) != HAL_OK) {
                        roller_panic("watchdog refresh");
                    }
                    static unsigned led_ticks{};
                    if (++led_ticks == 50) {
                        led_ticks = 0;
                        HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
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
