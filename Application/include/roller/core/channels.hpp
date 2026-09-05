#pragma once
#include "roller/core/spsc_queue.hpp"
#include "roller/drive/controller.hpp"
#include <atomic>
namespace roller {
struct Channels {
    SpscQueue<drive::Command, 4> commands;
    SpscQueue<drive::Telemetry, 4> telemetry;
    std::atomic<std::uint32_t> asynchronous_fault{};
    std::atomic<std::uint32_t> sample_count{}, can_task_ticks{}, isr_max_cycles{};
    std::atomic<bool> inhibit{true};
};
} // namespace roller
