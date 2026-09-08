#include "bench.hpp"
#include "main.h"
#include <cstring>

namespace roller::bench {
Profile profile;
struct Inbox {
    volatile std::uint32_t sequence{};
    volatile std::uint32_t cob_id{}, length{}, low{}, high{};
    volatile std::uint32_t accepted{};
};
struct Response {
    volatile std::uint32_t sequence{};
    std::uint32_t cob_id{}, length{}, low{}, high{};
};
struct Snapshot {
    const std::uint32_t magic = 0x524C4142, version = 2;
    volatile std::uint32_t sequence{};
    std::uint32_t samples{}, status{}, fault{}, calibration{}, pole_pairs{};
    std::int32_t direction{};
    float bus{}, temperature{}, position{}, velocity{}, id{}, iq{}, ia{}, ib{}, ic{}, current_limit{}, kt{},
        offset{};
    std::uint32_t outputs{}, sample_max{}, control_max{}, apply_max{}, total_max{}, interval_min{},
        interval_max{};
    std::uint32_t isr_max{}, sampling_epoch{};
    float cordic_max_error{};
};
static_assert(sizeof(Inbox) == 24 && sizeof(Response) == 20 && sizeof(Snapshot) == 124);
} // namespace roller::bench
extern "C" {
extern float roller_cordic_max_error;
roller::bench::Inbox roller_bench_rx;
roller::bench::Response roller_bench_tx;
roller::bench::Snapshot roller_bench_state;
volatile std::uint32_t roller_bench_request{1};
}
namespace roller::bench {
RollerCanTransport transport() noexcept {
    // Explicit bench build only: CANopen runs normally, its transport uses SWD
    // mailboxes instead of a physical CAN bus. No direct control-state writes.
    return {nullptr,
            [](void*) { return true; },
            [](void*) {},
            [](void*, const RollerCanFrame* frame) {
                if ((frame->ident & 0x780U) == 0x580U) {
                    auto& response = roller_bench_tx;
                    response.sequence = response.sequence + 1;
                    __DMB();
                    response.cob_id = frame->ident;
                    response.length = frame->size;
                    std::memcpy(&response.low, frame->data, 4);
                    std::memcpy(&response.high, frame->data + 4, 4);
                    __DMB();
                    response.sequence = response.sequence + 1;
                }
                return true;
            },
            [](void*) -> std::uint16_t { return 0; },
            [](void*) {}};
}
bool receive(RollerCanFrame& frame) noexcept {
    auto& input = roller_bench_rx;
    const auto seq = input.sequence;
    if (seq == input.accepted) {
        return false;
    }
    __DMB();
    const auto id = input.cob_id, length = input.length;
    const std::uint32_t words[]{input.low, input.high};
    std::memcpy(frame.data, words, sizeof(words));
    __DMB();
    if (seq != input.sequence) {
        return false;
    }
    input.accepted = seq;
    if (id > 0x7FF || length > 8) {
        return false;
    }
    frame.ident = static_cast<std::uint16_t>(id);
    frame.size = static_cast<std::uint8_t>(length);
    frame.synchronous = false;
    return true;
}
void publish(const drive::Telemetry& t, std::uint32_t isr_max, std::uint32_t epoch) noexcept {
    static std::uint32_t published{};
    const auto requested = roller_bench_request;
    if (requested == published) {
        return;
    }
    auto& s = roller_bench_state;
    s.sequence = s.sequence + 1;
    __DMB();
    s.samples = t.samples;
    s.status = t.statusword;
    s.fault = static_cast<std::uint32_t>(t.fault);
    s.calibration = static_cast<std::uint32_t>(t.calibration);
    s.pole_pairs = t.config.pole_pairs;
    s.direction = t.config.direction;
    s.bus = t.bus_voltage;
    s.temperature = t.mcu_temperature;
    s.position = t.position;
    s.velocity = t.velocity;
    s.id = t.id;
    s.iq = t.iq;
    s.ia = t.phase_current.a;
    s.ib = t.phase_current.b;
    s.ic = t.phase_current.c;
    s.current_limit = t.config.current_limit;
    s.kt = t.config.torque_constant;
    s.offset = t.config.electrical_offset;
    s.outputs = GPIOB->ODR;
    s.sample_max = profile.sample_max.load(std::memory_order_relaxed);
    s.control_max = profile.control_max.load(std::memory_order_relaxed);
    s.apply_max = profile.apply_max.load(std::memory_order_relaxed);
    s.total_max = profile.total_max.load(std::memory_order_relaxed);
    s.interval_min = profile.interval_min.load(std::memory_order_relaxed);
    s.interval_max = profile.interval_max.load(std::memory_order_relaxed);
    s.isr_max = isr_max;
    s.sampling_epoch = epoch;
    s.cordic_max_error = roller_cordic_max_error;
    __DMB();
    s.sequence = s.sequence + 1;
    published = requested;
}
void sampling_restart() noexcept {
    profile.last_start.store(0, std::memory_order_relaxed);
}
void record(std::uint32_t begin, std::uint32_t sampled, std::uint32_t controlled,
            std::uint32_t applied) noexcept {
    const auto maximum = [](auto& field, std::uint32_t value) {
        if (value > field.load(std::memory_order_relaxed)) {
            field.store(value, std::memory_order_relaxed);
        }
    };
    const auto previous = profile.last_start.load(std::memory_order_relaxed);
    profile.last_start.store(begin, std::memory_order_relaxed);
    // Exclude timer startup transients; all later intervals remain visible.
    const auto count = profile.samples.fetch_add(1, std::memory_order_relaxed);
    if (count < 100) {
        return;
    }
    maximum(profile.sample_max, sampled - begin);
    maximum(profile.control_max, controlled - sampled);
    maximum(profile.apply_max, applied - controlled);
    maximum(profile.total_max, applied - begin);
    if (!previous) {
        return; // ADC/PWM pause for parameter maintenance is deliberate.
    }
    const auto interval = begin - previous;
    if (interval < profile.interval_min.load(std::memory_order_relaxed)) {
        profile.interval_min.store(interval, std::memory_order_relaxed);
    }
    maximum(profile.interval_max, interval);
}
} // namespace roller::bench
