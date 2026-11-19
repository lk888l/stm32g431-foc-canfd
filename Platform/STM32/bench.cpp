#include "bench.hpp"
#include "board.hpp"
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
    const std::uint32_t magic = 0x524C4142, version = 5;
    volatile std::uint32_t sequence{};
    std::uint32_t samples{}, status{}, fault{}, calibration{}, pole_pairs{};
    std::int32_t direction{};
    float bus{}, temperature{}, position{}, velocity{}, id{}, iq{}, ia{}, ib{}, ic{}, current_limit{}, kt{},
        offset{};
    std::uint32_t outputs{}, sample_max{}, control_max{}, apply_max{}, total_max{}, interval_min{},
        interval_max{};
    std::uint32_t isr_max{}, sampling_epoch{};
    float cordic_max_error{};
    std::uint32_t warnings{}, encoder_missed{}, current_predicted{}, calibration_reason{},
        calibration_ticks{}, calibration_points{};
    float calibration_concentration{}, calibration_travel{}, iq_target{};
    std::uint32_t offsets_valid{}, zero_state{}, zero_reason{}, zero_span{}, driver_awake{}, pwm_channels{},
        no_motion{};
    float calibration_angle{};
    float voltage_a{}, voltage_b{}, voltage_c{};
};
static_assert(sizeof(Inbox) == 24 && sizeof(Response) == 20 && sizeof(Snapshot) == 204);
struct TraceSample {
    std::uint32_t samples;
    float position, velocity, iq, id, iq_target;
};
struct MotionSample {
    std::uint16_t angle_flags;
    std::int16_t velocity_512;
};
struct Peak {
    std::uint32_t frames, samples;
    float squared_sum, squared_peak, id, iq, iq_target, raw_a, raw_b, raw_c, electrical_angle, bus;
    std::uint16_t duty[3], reserved;
};
union TraceBuffer {
    TraceSample feedback[96];
    MotionSample motion[768];
    Peak peaks[8]; // Indexed by the ADC validity mask of the sampled PWM period.
};
static_assert(sizeof(TraceSample) == 24 && sizeof(MotionSample) == 4 && sizeof(Peak) == 56);
} // namespace roller::bench
extern "C" {
extern float roller_cordic_max_error;
roller::bench::Inbox roller_bench_rx;
roller::bench::Response roller_bench_tx;
roller::bench::Snapshot roller_bench_state;
volatile std::uint32_t roller_bench_request{1};
volatile std::uint32_t roller_bench_guard{};
volatile std::uint32_t roller_trace_request{}, roller_trace_ack{}, roller_trace_count{};
volatile std::uint32_t roller_trace_kind{}, roller_trace_first{}, roller_trace_last{};
std::int64_t roller_trace_origin{};
roller::bench::TraceBuffer roller_trace_data;
}
namespace roller::bench {
void trace(const drive::Controller& controller, const drive::Command& command, const drive::Sample& s,
           const std::array<std::uint16_t, 3>& duty) noexcept {
    static std::uint32_t generation{}, kind{};
    const auto requested = roller_trace_request;
    if (requested == roller_trace_ack) {
        return;
    }
    if (requested != generation) {
        generation = requested;
        kind = roller_trace_kind;
        roller_trace_count = 0;
        if (kind == 2) {
            // Initialize only counters here; the first frame fills each populated peak.
            for (unsigned i = 0; i < 8; ++i) {
                roller_trace_data.peaks[i].frames = 0;
                roller_trace_data.peaks[i].squared_sum = 0;
            }
        }
    }
    const auto& t = controller.telemetry();
    if (kind != 2 && t.samples % 20U != 0) {
        return;
    }
    const auto index = roller_trace_count;
    if (index == 0) {
        roller_trace_first = t.samples;
        roller_trace_origin = controller.position_counts();
    }
    std::uint32_t length{};
    if (kind == 1) {
        length = 768;
        const auto phase = static_cast<std::uint16_t>(controller.position_counts()) & 0x3FFFU;
        const bool marked = command.mode == drive::Mode::mit ? command.mit_torque != 0 : command.velocity > 0;
        const auto flags = phase | ((t.statusword & 0x1000U) << 2) | (marked ? 0x8000U : 0U);
        roller_trace_data.motion[index] = {
            static_cast<std::uint16_t>(flags),
            static_cast<std::int16_t>(std::clamp(t.velocity * 512.0F, -32767.0F, 32767.0F))};
    } else if (kind == 2) {
        length = 2000;
        auto& p = roller_trace_data.peaks[s.adc_ok ? s.current_mask & 7U : 0U];
        const float squared = t.id * t.id + t.iq * t.iq;
        p.squared_sum += squared;
        if (p.frames++ == 0 || squared > p.squared_peak) {
            p.samples = t.samples;
            p.squared_peak = squared;
            p.id = t.id;
            p.iq = t.iq;
            p.iq_target = t.iq_target;
            p.raw_a = s.current.a;
            p.raw_b = s.current.b;
            p.raw_c = s.current.c;
            p.electrical_angle = t.current_electrical_angle;
            p.bus = s.bus_voltage;
            std::copy(duty.begin(), duty.end(), p.duty);
        }
    } else {
        length = 96;
        roller_trace_data.feedback[index] = {t.samples, t.position, t.velocity, t.iq, t.id, t.iq_target};
    }
    roller_trace_count = index + 1;
    if (index + 1 == length) {
        roller_trace_last = t.samples;
        __DMB();
        roller_trace_ack = generation; // Frozen until the host requests another capture.
    }
}
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
    // An attended SWD client opts in before NMT Start. Stop also works when
    // the adapter is unplugged and the host cannot deliver its final frame.
    static bool guarded{};
    static std::uint32_t last_write_ms{};
    auto& input = roller_bench_rx;
    const auto seq = input.sequence;
    if (guarded && HAL_GetTick() - last_write_ms >= 600U) {
        guarded = false;
        input.accepted = seq;
        frame = {};
        frame.size = 2;
        frame.data[0] = 2; // Broadcast NMT Stop through the normal inhibit path.
        return true;
    }
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
    if (id == 0 && length == 2) {
        guarded = frame.data[0] == 1 && roller_bench_guard == 1;
        last_write_ms = HAL_GetTick();
    } else if (guarded && (id & 0x780U) == 0x600U && length == 8 && (frame.data[0] >> 5) == 1) {
        last_write_ms = HAL_GetTick(); // SDO downloads renew the attended session.
    }
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
    s.warnings = t.warnings;
    s.encoder_missed = t.encoder_missed;
    s.current_predicted = t.current_predicted;
    s.calibration_reason = static_cast<std::uint32_t>(t.calibration_reason);
    s.calibration_ticks = t.calibration_ticks;
    s.calibration_points = t.calibration_points;
    s.calibration_concentration = t.calibration_concentration;
    s.calibration_travel = t.calibration_travel;
    s.iq_target = t.iq_target;
    s.offsets_valid = roller_hardware.offsets_valid;
    s.zero_state = roller_hardware.zero_state;
    s.zero_reason = roller_hardware.zero_reason;
    s.zero_span = roller_hardware.zero_span;
    s.driver_awake = roller_hardware.driver_awake;
    s.pwm_channels = TIM1->CCER & 0x111U;
    s.no_motion = roller_hardware.no_motion;
    s.calibration_angle = t.calibration_angle;
    s.voltage_a = t.phase_voltage.a;
    s.voltage_b = t.phase_voltage.b;
    s.voltage_c = t.phase_voltage.c;
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
