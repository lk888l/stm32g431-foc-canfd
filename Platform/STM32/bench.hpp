#pragma once
#include "co_transport.h"
#include "roller/drive/controller.hpp"
#include <atomic>

namespace roller::bench {
struct Profile {
    std::atomic<std::uint32_t> samples{}, sample_max{}, control_max{}, apply_max{}, total_max{};
    std::atomic<std::uint32_t> interval_min{0xFFFFFFFFU}, interval_max{}, last_start{};
};
extern Profile profile;
RollerCanTransport transport() noexcept;
bool receive(RollerCanFrame& frame) noexcept;
void publish(const drive::Telemetry& telemetry, std::uint32_t isr_max, std::uint32_t epoch) noexcept;
void sampling_restart() noexcept;
void record(std::uint32_t begin, std::uint32_t sampled, std::uint32_t controlled,
            std::uint32_t applied) noexcept;
} // namespace roller::bench
