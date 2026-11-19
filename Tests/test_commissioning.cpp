#include "roller/control/cogging.hpp"
#include "roller/drive/controller.hpp"
#include "roller/drivers/current_zero.hpp"
#include <cstdio>
#include <cstdlib>

#define CHECK(x)                                                                                             \
    do {                                                                                                     \
        if (!(x)) {                                                                                          \
            std::fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #x);                                        \
            std::exit(1);                                                                                    \
        }                                                                                                    \
    } while (false)
using namespace roller;

static void multi_turn_precision_tests() {
    using namespace control;
    const float lsb = 1.0F / encoder_cpr;
    AngleTracker angle_only;
    for (int code = 0; code < encoder_cpr; ++code) {
        CHECK(angle_only.update(static_cast<float>(code) * (tau / encoder_cpr), drive::Controller::dt));
        CHECK(angle_only.counts() == code);
    }
    for (int sign : {-1, 1}) {
        const auto origin = static_cast<std::int64_t>(sign) * 10000 * encoder_cpr;
        PositionProfile profile;
        profile.hold(origin);
        profile.set_target(origin, static_cast<float>(sign) * lsb, true, false);
        for (int i = 0; i < 8; ++i) {
            profile.advance(static_cast<float>(sign) * lsb / 8);
        }
        CHECK(profile.counts == origin + sign && profile.remaining() == 0);
        CHECK(profile.error(origin) == static_cast<float>(sign) * lsb);
        profile.set_target(origin + 1, 0.5F + lsb, false, true);
        CHECK(profile.target == origin + 1 - encoder_cpr / 2);

        // Exercise the normal controller entry point at a large origin. The
        // host supplies 64 actual encoder counts per tick, then stops one count
        // beyond +/-10000 turns. Its REAL32 telemetry rounds to the target.
        drive::Controller motor({0, 0.25F, 0.3F, 11, 1, 1});
        drive::Sample sample{{}, 19, 0, 25, true, true, true, true};
        (void)motor.step(sample);
        for (int i = 1; i <= 10000 * (encoder_cpr / 64); ++i) {
            const unsigned code = static_cast<unsigned>(i * sign * 64) & (encoder_cpr - 1);
            sample.angle = static_cast<float>(code) * (tau / encoder_cpr);
            sample.encoder_counts = static_cast<std::uint16_t>(code);
            (void)motor.step(sample);
        }
        sample.angle = tau / encoder_cpr;
        sample.encoder_counts = 1;
        drive::Command cmd;
        cmd.operational = true;
        cmd.cogging_enabled = false;
        cmd.mode = drive::Mode::mit;
        cmd.mit_position = static_cast<float>(sign * 10000);
        cmd.mit_kp = 3;
        for (auto cw : {6, 7, 15}) {
            cmd.controlword = static_cast<std::uint16_t>(cw);
            motor.command(cmd);
            for (int i = 0; i < 400; ++i) {
                (void)motor.step(sample);
            }
        }
        CHECK(motor.position_counts() == origin + 1);
        CHECK(motor.telemetry().iq_target == -3 * lsb / 0.25F);
        cmd.mode = drive::Mode::multi_turn_position;
        cmd.position = 17 * lsb;
        cmd.profile_velocity = 0.01F;
        for (auto cw : {15, 95}) {
            cmd.controlword = static_cast<std::uint16_t>(cw);
            motor.command(cmd);
            for (int i = 0; i < 4000; ++i) {
                (void)motor.step(sample);
            }
        }
        CHECK(motor.telemetry().statusword & 0x1000);
        CHECK(!(motor.telemetry().statusword & 0x0400));
        CHECK(motor.telemetry().iq_target > 0);
        sample.angle = 18 * tau / encoder_cpr;
        sample.encoder_counts = 18;
        for (int i = 0; i < 4000; ++i) {
            (void)motor.step(sample);
        }
        CHECK(motor.telemetry().statusword & 0x0400);
    }
}

static void velocity_estimator_tests() {
    using namespace control;
    Pi scheduled{0.03F, 3};
    (void)scheduled.step(1, 0.001F, 1);
    const float stored = scheduled.step(0, 0.001F, 1);
    scheduled.gains(0.06F, 3);
    CHECK(scheduled.step(0, 0.001F, 1) == stored);
    for (const float speed : {0.1F, -0.1F, 10.0F, -10.0F}) {
        AngleTracker tracker;
        double sum = 0, squared = 0;
        unsigned count = 0;
        double time = 0;
        float angle = 0;
        for (unsigned i = 0; i < 20000; ++i) {
            // Include missing frames and both wrap directions. The accepted
            // frame spans the complete elapsed interval after a dropout.
            const float dt = i % 101 == 0 ? 0.00015F : 0.00005F;
            time += dt;
            const double turns = 0.7 + speed * time;
            angle = static_cast<float>(std::floor((turns - std::floor(turns)) * 16384)) * tau / 16384;
            CHECK(tracker.update(angle, dt));
            if (i > 1000) {
                const double error = tracker.velocity() / tau - speed;
                sum += error;
                squared += error * error;
                ++count;
            }
        }
        CHECK(std::abs(sum / count) < 0.001);
        CHECK(std::sqrt(squared / count) < 0.025);
        for (unsigned n = 0; n < 200; ++n) {
            CHECK(tracker.update(angle, 0.00005F));
        }
        CHECK(std::abs(tracker.velocity()) < 1e-6F);
        tracker.resynchronize(1);
        CHECK(tracker.velocity() == 0);
        for (unsigned i = 0; i < 100; ++i) {
            CHECK(tracker.update(1, 0.00005F));
            CHECK(tracker.velocity() == 0);
        }
    }
}

static void zero_tests() {
    using drivers::CurrentZero;
    using drivers::ZeroReason;
    CurrentZero good;
    for (unsigned n = 0; n < CurrentZero::window; ++n) {
        good.update({static_cast<std::uint16_t>(2048 + n % 3), 2050, 2044}, true);
    }
    CHECK(good.valid());
    CHECK(good.span() == 2);
    CHECK(good.means()[0] > 2048 && good.means()[0] < 2050);
    // The actual board's observed stable high readings are not valid zeros.
    CurrentZero floating;
    for (unsigned n = 0; n < CurrentZero::window * 3; ++n) {
        floating.update({3420, 3550, 3632}, true);
    }
    CHECK(floating.failed() && floating.reason() == ZeroReason::range);
    CHECK(floating.means()[1] == 3550);
    floating.update({2048, 2048, 2048}, true);
    CHECK(floating.failed()); // A failed boot cannot silently re-arm.
    CurrentZero noisy, mismatch, missing;
    for (unsigned n = 0; n < CurrentZero::timeout; ++n) {
        noisy.update({static_cast<std::uint16_t>(2000 + n % 50), 2048, 2048}, true);
        mismatch.update({2048, 2200, 2048}, true);
        missing.update({2048, 2048, 2048}, false);
    }
    CHECK(noisy.failed() && noisy.reason() == ZeroReason::noise);
    CHECK(mismatch.failed() && mismatch.reason() == ZeroReason::mismatch);
    CHECK(missing.failed() && missing.reason() == ZeroReason::unavailable);
    CurrentZero interrupted;
    interrupted.fail(ZeroReason::cancelled);
    CHECK(interrupted.failed());
    CurrentZero transient;
    for (unsigned n = 0; n < CurrentZero::window; ++n) {
        transient.update({3500, 3500, 3500}, true);
    }
    for (unsigned n = 0; n < CurrentZero::window; ++n) {
        transient.update({2048, 2048, 2048}, true);
    }
    CHECK(transient.valid());
}

static void calibration_fault_tests() {
    using namespace drive;
    for (auto failure : {0, 1, 2, 3, 4, 5}) {
        Controller controller;
        Sample sample{{}, 19.35F, 1, 25, true, true, true, true};
        Command command;
        command.calibration_request = 1;
        if (failure == 0) {
            sample.current_calibrated = false;
        }
        if (failure == 1) {
            sample.current_zero_failed = true;
        }
        if (failure == 2) {
            sample.current_mask = 0;
        }
        if (failure == 3) {
            sample.current = {2, -1, -1};
        }
        if (failure == 4) {
            sample.bus_voltage = 40;
        }
        // failure 5: disconnected sense path reports zero regardless of voltage.
        bool ever_enabled = false;
        for (unsigned n = 0; n < 6000; ++n) {
            if (n % 20 == 0) {
                ++command.sequence;
                controller.command(command);
            }
            const auto output = controller.step(sample);
            ever_enabled |= output.enable;
            CHECK(!output.enable || (!output.stop && !sample.current_zero_failed));
        }
        const auto t = controller.telemetry();
        CHECK(t.calibration == Calibration::failed);
        CHECK(!t.config.calibrated);
        CHECK(t.fault != Fault::none);
        CHECK(!controller.step(sample).enable);
        if (failure < 2) {
            CHECK(!ever_enabled);
        }
        if (failure == 1) {
            CHECK(t.fault == Fault::current_sense);
        }
        if (failure == 2) {
            CHECK(t.calibration_reason == CalibrationReason::current_feedback);
        }
        if (failure == 3) {
            CHECK(t.calibration_reason == CalibrationReason::current_limit);
        }
        if (failure == 4) {
            CHECK(t.calibration_reason == CalibrationReason::bus_voltage);
        }
        if (failure == 5) {
            CHECK(t.calibration_reason == CalibrationReason::phase_response);
        }
    }
    // A small configured current limit also reduces the open-loop probe voltage.
    MotorConfig motor;
    motor.current_limit = 0.1F;
    Controller controller{motor};
    Sample sample{{}, 24, 1, 25, true, true, true, true};
    Command command;
    command.calibration_request = 1;
    for (int n = 0; n < 3000; ++n) {
        if (n % 20 == 0) {
            ++command.sequence;
            controller.command(command);
        }
        const auto output = controller.step(sample);
        const auto v = control::clarke({output.duty.a * 24, output.duty.b * 24, output.duty.c * 24});
        CHECK(std::hypot(v.alpha, v.beta) < 0.136F);
    }
    command.calibration_permitted = false;
    controller.command(command);
    const auto stopped = controller.step(sample);
    CHECK(!stopped.enable && stopped.stop);
}

static void wake_tests() {
    using namespace drive;
    MotorConfig motor;
    motor.calibrated = 1;
    Controller controller{motor};
    Sample sample{{}, 19, 0, 25, true, true, true, true};
    sample.power_ready = false;
    Command command;
    command.mode = Mode::torque;
    command.operational = true;
    command.torque = 200;
    for (auto cw : {6, 7, 15}) {
        command.controlword = static_cast<std::uint16_t>(cw);
        ++command.sequence;
        controller.command(command);
        for (int n = 0; n < 100; ++n) {
            const auto out = controller.step(sample);
            CHECK(out.duty.a == 0.5F && out.duty.b == 0.5F && out.duty.c == 0.5F);
        }
    }
    CHECK(controller.step(sample).enable);
    sample.power_ready = true;
    const auto out = controller.step(sample);
    const auto v = control::clarke({19 * out.duty.a, 19 * out.duty.b, 19 * out.duty.c});
    CHECK(v.beta > 0 && v.beta < 1); // No accumulated multi-volt wake-up kick.
}

static void phase_mapping_tests() {
    using namespace drive;
    for (int mapping = 0; mapping < 3; ++mapping) {
        Controller controller;
        Sample sample{{}, 19, 1, 25, true, true, true, true};
        Command command;
        command.calibration_request = 1;
        for (int n = 0; n < 12000; ++n) {
            if (n % 20 == 0) {
                ++command.sequence;
                controller.command(command);
            }
            const auto out = controller.step(sample);
            const auto common = (out.duty.a + out.duty.b + out.duty.c) / 3;
            const float scale = sample.bus_voltage / control::motor_resistance;
            sample.current = {(out.duty.a - common) * scale, (out.duty.b - common) * scale,
                              (out.duty.c - common) * scale};
            if (mapping == 0) { // Inverted current polarity.
                sample.current.a *= -1;
                sample.current.b *= -1;
                sample.current.c *= -1;
            } else if (mapping == 1) {
                std::swap(sample.current.b, sample.current.c);
            } else {
                sample.current.a = 0; // Stuck channel; Kirchhoff residual catches it.
            }
        }
        CHECK(controller.telemetry().calibration_reason == CalibrationReason::phase_response);
        CHECK(!controller.telemetry().config.calibrated);
        CHECK(!controller.step(sample).enable);
    }
}

static void cogging_tests() {
    using namespace control;
    const auto lookup = [](float angle) { return cogging_current(angle, cogging::reference_offset, 11, 1); };
    float sum = 0;
    for (unsigned i = 0; i < 1024; ++i) {
        const auto a = static_cast<float>(i) * tau / 1024;
        const auto value = lookup(a);
        CHECK(std::abs(value) < 0.101F);
        CHECK(std::abs(value - lookup(a + tau)) < 0.00002F);
        const auto midpoint = lookup(a + tau / 2048);
        CHECK(std::abs(midpoint - 0.5F * (value + lookup(a + tau / 1024))) < 0.00002F);
        sum += value;
    }
    CHECK(std::abs(sum / 1024) < 0.00002F);
    CHECK(cogging_current(0, cogging::reference_offset + 0.2F, 11, 1) == 0);
    CHECK(cogging_current(0, cogging::reference_offset, 10, 1) == 0);
    CHECK(cogging_current(0, cogging::reference_offset, 11, -1) == 0);
    using namespace drive;
    MotorConfig motor;
    motor.calibrated = 1;
    motor.electrical_offset = cogging::reference_offset;
    Controller controller{motor};
    Sample sample{{}, 19, 0.3F, 25, true, true, true, true};
    Command command;
    command.mode = Mode::torque;
    command.operational = true;
    command.max_torque = 20; // Total current, including feedforward, stays below 28 mA.
    command.torque = 1000;
    const auto update = [&](unsigned cw) {
        command.controlword = static_cast<std::uint16_t>(cw);
        ++command.sequence;
        controller.command(command);
        for (int n = 0; n < 40; ++n) {
            (void)controller.step(sample);
        }
    };
    for (auto cw : {6U, 7U, 15U}) {
        update(cw);
    }
    CHECK(std::abs(controller.telemetry().iq_target) <= 0.02801F);
    update(0x10FU);
    CHECK(controller.telemetry().iq_target == 0);
    command.torque = 0;
    command.cogging_enabled = false;
    update(15);
    CHECK(controller.telemetry().iq_target == 0);
    command.cogging_enabled = true;
    update(15);
    CHECK(std::abs(controller.telemetry().iq_target - std::clamp(lookup(sample.angle), -0.028F, 0.028F)) <
          0.00001F);
    command.max_torque = 1000;
    update(15);
    const float before = controller.telemetry().iq_target;
    sample.angle += 0.0005F;
    (void)controller.step(sample); // Between motion-loop ticks, feedforward still follows angle.
    CHECK(std::abs(controller.telemetry().iq_target - before) > 0.00001F);
    command.max_torque = 20;
    update(15);
    for (unsigned n = 0; n < 200; ++n) {
        sample.angle += 0.001F;
        (void)controller.step(sample);
        CHECK(std::abs(controller.telemetry().iq_target) <= 0.02801F);
    }
    for (unsigned n = 0; n < 200; ++n) {
        sample.angle += 0.003F;
        (void)controller.step(sample);
    }
    CHECK(controller.telemetry().velocity > 6);
    CHECK(controller.telemetry().iq_target == 0); // High-speed taper applies even in torque mode.
}

int main() {
    multi_turn_precision_tests();
    velocity_estimator_tests();
    zero_tests();
    calibration_fault_tests();
    wake_tests();
    phase_mapping_tests();
    cogging_tests();
    std::puts("PASS: current zero and commissioning fault injection");
}
