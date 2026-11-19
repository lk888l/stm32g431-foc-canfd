#include "roller/drive/controller.hpp"
#include "roller/control/cogging.hpp"
#include <algorithm>
#include <array>
#include <cmath>

namespace roller::drive {
using namespace control;
bool Controller::valid_config(const MotorConfig& c) noexcept {
    return std::isfinite(c.electrical_offset) && c.electrical_offset >= 0 && c.electrical_offset < tau &&
           std::isfinite(c.torque_constant) && c.torque_constant > 0.001F && c.torque_constant <= 1.0F &&
           std::isfinite(c.current_limit) && c.current_limit > 0 && c.current_limit <= 4.9F &&
           c.pole_pairs >= 1 && c.pole_pairs <= 32 && (c.direction == 1 || c.direction == -1) &&
           c.calibrated <= 1;
}
bool Controller::valid_command(const Command& c) noexcept {
    const std::array values{c.position,     c.velocity,     c.profile_velocity,
                            c.acceleration, c.deceleration, c.quick_deceleration,
                            c.mit_position, c.mit_velocity, c.mit_torque,
                            c.mit_kp,       c.mit_kd,       c.following_window};
    for (auto x : values) {
        if (!std::isfinite(x)) {
            return false;
        }
    }
    const bool mode = c.mode == Mode::none || is_position_mode(c.mode) || c.mode == Mode::velocity ||
                      c.mode == Mode::torque || c.mode == Mode::mit;
    return mode && std::abs(c.position) <= 100000 && std::abs(c.mit_position) <= 100000 &&
           std::abs(c.velocity) <= 40 && std::abs(c.mit_velocity) <= 40 && std::abs(c.mit_torque) <= 1 &&
           c.mit_kp >= 0 && c.mit_kp <= 100 && c.mit_kd >= 0 && c.mit_kd <= 20 && c.profile_velocity > 0 &&
           c.profile_velocity <= 40 && c.acceleration > 0 && c.acceleration <= 100 && c.deceleration > 0 &&
           c.deceleration <= 100 && c.quick_deceleration > 0 && c.quick_deceleration <= 100 &&
           c.max_torque <= 1000 && c.torque >= -1000 && c.torque <= 1000 && c.following_window > 0;
}
void Controller::command(const Command& value) noexcept {
    const auto validated = ValidatedCommand::make(value);
    if (!validated) {
        telemetry_.warnings |= math_warning;
        return;
    }
    command(*validated);
}
std::optional<ValidatedCommand> ValidatedCommand::make(const Command& value) noexcept {
    if (!Controller::valid_command(value)) {
        return std::nullopt;
    }
    return ValidatedCommand{value};
}
void Controller::command(const ValidatedCommand& validated) noexcept {
    const auto& value = validated.get();
    if (value.sequence != command_.sequence) {
        command_age_ = 0;
    }
    if (value.mode != command_.mode) {
        reset_loops();
    }
    command_ = value;
}
void Controller::reset_loops() noexcept {
    d_pi_.reset();
    q_pi_.reset();
    velocity_pi_.reset();
    iq_target_ = 0;
    iq_feedback_ = 0;
    velocity_setpoint_ = 0;
    profile_.hold(position_counts());
    setpoint_ack_ = false;
    following_ticks_ = 0;
}
void Controller::outer_step(const Sample& s) noexcept {
    const bool resetting = (command_.controlword & 0x80) && !(previous_cw_ & 0x80);
    const bool inputs_ok =
        s.encoder_ok && s.adc_ok && s.driver_ok && s.current_calibrated && !s.current_zero_failed &&
        s.bus_voltage >= 4.5F && s.bus_voltage <= 35.0F && s.mcu_temperature < 125.0F &&
        std::isfinite(s.current.a) && std::isfinite(s.current.b) && std::isfinite(s.current.c) &&
        std::max({std::abs(s.current.a), std::abs(s.current.b), std::abs(s.current.c)}) < 0.1F &&
        command_.network_ok && command_age_ < 2000 && valid_config(config_);
    if (resetting && inputs_ok) {
        fault_ = Fault::none;
        reset_loops();
    }
    const auto old_state = state_.state();
    const bool permit =
        command_.operational && command_.mode != Mode::none && config_.calibrated && s.current_calibrated;
    const auto next = state_.step(command_.controlword, fault_ == Fault::none, permit,
                                  std::abs(telemetry_.velocity) < 0.05F);
    if (old_state != next) {
        reset_loops();
        if (next == State::quick_stop) {
            velocity_setpoint_ = telemetry_.velocity;
        }
    }
    const auto max_i = config_.current_limit * static_cast<float>(command_.max_torque) / 1000.0F;
    const bool active = next == State::enabled || next == State::quick_stop;
    telemetry_.mode = active ? command_.mode : Mode::none;
    if (!active) {
        reset_loops();
        previous_cw_ = command_.controlword;
        return;
    }
    // Low-speed gain avoids the measured standstill limit cycle. Restore
    // damping progressively during motion without resetting the integrator.
    velocity_pi_.gains(0.03F + 0.03F * std::min(std::abs(velocity_setpoint_), 1.0F), 3.0F);
    if (next == State::quick_stop) {
        velocity_setpoint_ = slew(velocity_setpoint_, 0, command_.quick_deceleration, 0.001F);
        iq_target_ = velocity_pi_.step((velocity_setpoint_ - telemetry_.velocity) * tau, 0.001F, max_i);
    } else {
        switch (command_.mode) {
        case Mode::single_turn_position:
        case Mode::multi_turn_position: {
            const auto measured = position_counts();
            const bool trigger = (command_.controlword & 0x10) && !(previous_cw_ & 0x10);
            if (!(command_.controlword & 0x10)) {
                setpoint_ack_ = false;
            }
            if (trigger) {
                const bool relative = (command_.controlword & 0x40) != 0;
                profile_.set_target(measured, command_.position, relative,
                                    command_.mode == Mode::single_turn_position);
                setpoint_ack_ = true;
            }
            const auto error = profile_.remaining();
            const auto braking = std::sqrt(2.0F * command_.deceleration * std::abs(error));
            const auto desired = (command_.controlword & 0x100)
                                     ? 0.0F
                                     : std::copysign(std::min(command_.profile_velocity, braking), error);
            velocity_setpoint_ = velocity_ramp(velocity_setpoint_, desired, command_.acceleration,
                                               command_.deceleration, 0.001F);
            const auto increment = velocity_setpoint_ * 0.001F;
            if (increment * error >= 0 && std::abs(increment) >= std::abs(error)) {
                profile_.hold(profile_.target);
                velocity_setpoint_ = 0;
            } else {
                profile_.advance(increment);
            }
            const float lag =
                profile_.error(measured) - telemetry_.velocity * (dt * static_cast<float>(missed_angles_));
            const auto v = std::clamp(velocity_setpoint_ + 20.0F * lag, -40.0F, 40.0F);
            iq_target_ = velocity_pi_.step((v - telemetry_.velocity) * tau, 0.001F, max_i);
            if (std::abs(lag) > command_.following_window) {
                if (following_ticks_ < 65535) {
                    ++following_ticks_;
                }
                if (following_ticks_ >= command_.following_ms) {
                    telemetry_.warnings |= following_warning;
                }
            } else {
                following_ticks_ = 0;
            }
            break;
        }
        case Mode::velocity: {
            const auto desired = (command_.controlword & 0x100) ? 0.0F : command_.velocity;
            velocity_setpoint_ = velocity_ramp(velocity_setpoint_, desired, command_.acceleration,
                                               command_.deceleration, 0.001F);
            iq_target_ = velocity_pi_.step((velocity_setpoint_ - telemetry_.velocity) * tau, 0.001F, max_i);
            break;
        }
        case Mode::torque:
            iq_target_ =
                (command_.controlword & 0x100)
                    ? 0.0F
                    : std::clamp(config_.current_limit * static_cast<float>(command_.torque) / 1000.0F,
                                 -max_i, max_i);
            break;
        case Mode::mit: {
            const float error =
                count_difference_turns(command_counts(command_.mit_position) - position_counts()) -
                telemetry_.velocity * (dt * static_cast<float>(missed_angles_));
            const auto torque = command_.mit_torque + command_.mit_kp * error +
                                command_.mit_kd * (command_.mit_velocity - telemetry_.velocity);
            iq_target_ = (command_.controlword & 0x100)
                             ? 0.0F
                             : std::clamp(torque / config_.torque_constant, -max_i, max_i);
            break;
        }
        case Mode::none:
            break;
        }
    }
    iq_feedback_ = iq_target_;
    previous_cw_ = command_.controlword;
}
float Controller::cogging_feedforward(const Sample& s) const noexcept {
    if (!command_.cogging_enabled || (command_.controlword & 0x100)) {
        return 0;
    }
    // This slow-scan map also contains encoder/friction residuals. A/B tests
    // show useful compensation at <=1 rev/s but more ripple at >=6 rev/s.
    const float gain = std::clamp((6.0F - std::abs(telemetry_.velocity)) * 0.2F, 0.0F, 1.0F);
    if (gain == 0) {
        return 0;
    }
    // Compensate PWM transport and the nominal current-loop time constant.
    // The lookup runs at 20 kHz so high spatial harmonics do not alias at the
    // 1 kHz motion-loop rate. This does not increase the total current limit.
    const float lead =
        dt * static_cast<float>(missed_angles_) + s.voltage_angle_delay + 1.0F / (tau * current_bandwidth);
    return gain * cogging_current(last_angle_ + tracker_.velocity() * lead, config_.electrical_offset,
                                  config_.pole_pairs, config_.direction);
}
Output Controller::calibration_step(const Sample& s) noexcept {
    // Pole pairs are known from the motor specification. Never replace 11 with
    // an apparently plausible count computed from a slipping rotor's endpoints.
    ++calibration_tick_;
    telemetry_.calibration_ticks = calibration_tick_;
    const float current_limit = std::min(config_.current_limit, 0.8F);
    const float voltage_limit = std::min(1.2F, motor_resistance * current_limit * 0.75F);
    if (s.bus_voltage < 4.5F || s.bus_voltage > 35.0F) {
        return fail_calibration(CalibrationReason::bus_voltage);
    }
    calibration_missing_ = s.adc_ok && s.current_mask == 7 ? 0U : calibration_missing_ + 1U;
    if (calibration_missing_ > 200) {
        return fail_calibration(CalibrationReason::current_feedback);
    }
    const float peak = std::max({std::abs(s.current.a), std::abs(s.current.b), std::abs(s.current.c)});
    calibration_overcurrent_ =
        s.current_mask == 7 && peak > current_limit * 1.25F + 0.05F ? calibration_overcurrent_ + 1U : 0U;
    if (calibration_overcurrent_ >= 20) {
        return fail_calibration(CalibrationReason::current_limit);
    }
    float theta = 0;
    float amplitude = voltage_limit;
    const bool probing = calibration_ == Calibration::align && calibration_tick_ <= 12000;
    SinCos sc{};
    Dq field_current{};
    if (probing) {
        // Probe three independent stator directions before trusting current
        // polarity/phase order. This does not depend on the encoder offset.
        const auto local = (calibration_tick_ - 1U) % 4000U + 1U;
        theta = static_cast<float>((calibration_tick_ - 1U) / 4000U) * tau / 3.0F;
        sc = trig_(theta);
        field_current = park(clarke(s.current), sc.sine, sc.cosine);
        amplitude *= std::min(1.0F, static_cast<float>(local) / 2000.0F);
        if (local == 1) {
            calibration_d_sum_ = calibration_q_sum_ = calibration_phase_sum_ = 0;
            calibration_current_samples_ = 0;
        }
        if (local > 2000 && s.current_mask == 7) {
            calibration_d_sum_ += field_current.d;
            calibration_q_sum_ += field_current.q;
            calibration_phase_sum_ += s.current.a + s.current.b + s.current.c;
            ++calibration_current_samples_;
        }
        if (local == 4000) {
            const float n = static_cast<float>(std::max<std::uint32_t>(1U, calibration_current_samples_));
            const float d = calibration_d_sum_ / n, q = calibration_q_sum_ / n;
            if (calibration_current_samples_ < 1000 || d < std::min(0.03F, current_limit * 0.1F) ||
                std::abs(q) > 0.4F * d + 0.02F ||
                std::abs(calibration_phase_sum_ / n) > 0.1F + 0.2F * std::abs(d)) {
                return fail_calibration(CalibrationReason::phase_response);
            }
        }
    } else if (calibration_ == Calibration::align && calibration_tick_ < 22000) {
        amplitude *= std::min(1.0F, static_cast<float>(calibration_tick_ - 12000) / 2000.0F);
    } else if (calibration_ == Calibration::align && calibration_tick_ >= 22000) {
        calibration_start_ = tracker_.counts();
        calibration_segment_start_ = calibration_start_;
        calibration_tick_ = 0;
        calibration_segments_ = 0;
        calibration_direction_ = 0;
        offset_count_ = 0;
        offset_sine_ = 0;
        offset_cosine_ = 0;
        calibration_ = Calibration::sweep;
    } else if (calibration_ == Calibration::sweep) {
        // One complete mechanical revolution in each direction at 0.125 rev/s.
        // The old four electrical revolutions sampled only 4/11 of this motor.
        const std::uint32_t sweep_ticks = calibration_slow_ ? 2000000U : 160000U;
        const std::uint32_t segment_ticks = calibration_slow_ ? 100000U : 10000U;
        const bool reverse = calibration_tick_ > sweep_ticks;
        const float travel =
            static_cast<float>(config_.pole_pairs) * tau *
            static_cast<float>(reverse ? 2 * sweep_ticks - calibration_tick_ : calibration_tick_) /
            static_cast<float>(sweep_ticks);
        theta = wrap(travel);
        if (calibration_tick_ % segment_ticks == 0) {
            const float moved = count_difference_turns(tracker_.counts() - calibration_segment_start_) * tau;
            const float expected = tau * static_cast<float>(segment_ticks) / static_cast<float>(sweep_ticks);
            if (!calibration_direction_) {
                calibration_direction_ = moved >= 0 ? 1 : -1;
            }
            const float signed_motion =
                moved * static_cast<float>(calibration_direction_) * (reverse ? -1.0F : 1.0F);
            if (signed_motion < expected * 0.65F || signed_motion > expected * 1.35F) {
                return fail_calibration(CalibrationReason::travel);
            }
            calibration_segment_start_ = tracker_.counts();
            ++calibration_segments_;
            telemetry_.calibration_travel =
                count_difference_turns(tracker_.counts() - calibration_start_) * tau;
        }
        if (calibration_direction_ && calibration_tick_ % 20 == 0 && calibration_tick_ > segment_ticks &&
            (calibration_tick_ <= sweep_ticks || calibration_tick_ > sweep_ticks + segment_ticks)) {
            const auto offset = wrap(static_cast<float>(calibration_direction_) *
                                         static_cast<float>(config_.pole_pairs) * s.angle -
                                     theta);
            const auto offset_sc = trig_(offset);
            offset_sine_ += offset_sc.sine;
            offset_cosine_ += offset_sc.cosine;
            ++offset_count_;
        }
        if (calibration_tick_ >= 2 * sweep_ticks) {
            calibration_ = Calibration::settle;
            calibration_tick_ = 0;
            theta = 0;
        }
    } else if (calibration_ == Calibration::settle && calibration_tick_ >= 10000) {
        const float concentration = std::hypot(offset_sine_, offset_cosine_) /
                                    static_cast<float>(std::max<std::uint32_t>(1U, offset_count_));
        telemetry_.calibration_concentration = concentration;
        telemetry_.calibration_points = offset_count_;
        if (calibration_segments_ != (calibration_slow_ ? 40U : 32U)) {
            return fail_calibration(CalibrationReason::travel);
        }
        if (concentration < 0.95F) {
            return fail_calibration(CalibrationReason::concentration);
        }
        if (std::abs(count_difference_turns(tracker_.counts() - calibration_start_) * tau) > 0.15F ||
            std::abs(tracker_.velocity()) > 0.1F) {
            return fail_calibration(CalibrationReason::return_position);
        }
        config_.direction = calibration_direction_;
        config_.electrical_offset = wrap(std::atan2(offset_sine_, offset_cosine_));
        config_.calibrated = 1;
        calibration_ = Calibration::complete;
        reset_loops();
        return {};
    }
    if (!probing) {
        sc = trig_(theta);
        field_current = park(clarke(s.current), sc.sine, sc.cosine);
    }
    telemetry_.id = field_current.d;
    telemetry_.iq = field_current.q;
    telemetry_.calibration_angle = theta;
    return {svpwm({amplitude * sc.cosine, amplitude * sc.sine}, s.bus_voltage), true};
}
Output Controller::fail_calibration(CalibrationReason reason) noexcept {
    calibration_ = Calibration::failed;
    config_.calibrated = 0;
    telemetry_.calibration_reason = reason;
    trip(Fault::calibration);
    state_.trip();
    return {};
}
Output Controller::step(const Sample& s) noexcept {
    ++ticks_;
    if (command_age_ < 0xFFFFFFFFU) {
        ++command_age_;
    }
    telemetry_.warnings = 0;
    const bool finite = std::isfinite(s.bus_voltage) && std::isfinite(s.mcu_temperature) &&
                        std::isfinite(s.current.a) && std::isfinite(s.current.b) &&
                        std::isfinite(s.current.c);
    if (!s.driver_ok) {
        trip(Fault::driver);
    }
    if (s.current_zero_failed) {
        trip(Fault::current_sense);
    }
    const float elapsed_angle = dt * static_cast<float>(missed_angles_ + 1U);
    bool angle_ok = s.encoder_ok && tracker_.update(s.angle, elapsed_angle, s.encoder_counts);
    if (angle_ok) {
        last_angle_ = s.angle;
        missed_angles_ = 0;
        angle_initialized_ = true;
    } else {
        missed_angles_ = std::min<std::uint32_t>(missed_angles_ + 1U, 20000U);
        ++telemetry_.encoder_missed;
        telemetry_.warnings |= encoder_prediction;
    }
    if (!finite) {
        telemetry_.warnings |= math_warning;
    }
    if (s.bus_voltage < 4.5F || s.bus_voltage > 35.0F) {
        telemetry_.warnings |= voltage_warning;
    }
    if (s.mcu_temperature >= 100.0F) {
        telemetry_.warnings |= temperature_warning;
    }
    if (s.current_calibrated && s.current_mask == 7 &&
        std::max({std::abs(s.current.a), std::abs(s.current.b), std::abs(s.current.c)}) > 4.9F) {
        telemetry_.warnings |= current_warning;
    }
    if (!command_.network_ok || command_age_ > 2000) {
        telemetry_.warnings |= communication_warning;
    }
    telemetry_.samples = ticks_;
    telemetry_.command_sequence = command_.sequence;
    const float age = dt * static_cast<float>(missed_angles_);
    const float speed = tracker_.velocity();
    telemetry_.position = static_cast<float>(config_.direction) * tracker_.position_turns(speed * age);
    telemetry_.velocity = static_cast<float>(config_.direction) * speed / tau;
    if (std::isfinite(s.bus_voltage)) {
        telemetry_.bus_voltage = s.bus_voltage;
    }
    if (std::isfinite(s.mcu_temperature)) {
        telemetry_.mcu_temperature = s.mcu_temperature;
    }
    const auto mask = static_cast<std::uint8_t>(finite && s.adc_ok ? s.current_mask & 7U : 0U);
    if (mask != 7) {
        ++telemetry_.current_predicted;
        telemetry_.warnings |= current_prediction;
    }
    Dq currents{};
    SinCos voltage_sc{};
    bool trig_ok = true;
    if (calibration_ == Calibration::align || calibration_ == Calibration::sweep ||
        calibration_ == Calibration::settle) {
        // Calibration uses the commanded stator frame and requires measured
        // currents. Running a second, uncalibrated FOC/observer path here used
        // to consume the entire PWM commit window before producing any output.
        telemetry_.phase_current = finite ? s.current : Abc{};
    } else {
        const float electrical_speed =
            static_cast<float>(config_.direction) * static_cast<float>(config_.pole_pairs) * speed;
        const float base =
            static_cast<float>(config_.direction) * static_cast<float>(config_.pole_pairs) * last_angle_ -
            config_.electrical_offset;
        telemetry_.current_electrical_angle = wrap(base + electrical_speed * (age + s.current_angle_delay));
        const auto [sine, cosine] = trig_(telemetry_.current_electrical_angle);
        voltage_sc = trig_(wrap(base + electrical_speed * (age + s.voltage_angle_delay)));
        trig_ok = std::isfinite(sine) && std::isfinite(cosine) && std::isfinite(voltage_sc.sine) &&
                  std::isfinite(voltage_sc.cosine);
        // Use the electrically identified phase-peak coefficient. The public
        // torque constant has an unverified peak/RMS convention and is not an
        // adequate back-EMF model when only one phase can be sampled.
        const float emf_q = motor_back_emf * telemetry_.velocity * tau;
        const auto emf = trig_ok ? inverse_park({0, emf_q}, sine, cosine) : AlphaBeta{};
        const bool applied_finite =
            std::isfinite(s.applied_voltage.alpha) && std::isfinite(s.applied_voltage.beta);
        const auto voltage =
            s.applied_voltage_valid && applied_finite ? s.applied_voltage : clarke(telemetry_.phase_voltage);
        const auto estimated = observer_.update(finite ? s.current : Abc{}, mask, voltage, emf, dt);
        telemetry_.phase_current = estimated;
        currents = trig_ok ? park(clarke(estimated), sine, cosine) : Dq{};
        telemetry_.id = currents.d;
        telemetry_.iq = currents.q;
    }
    if (!initialized_) {
        reset_loops();
        initialized_ = true;
    }
    if (ticks_ % 20 == 0) {
        outer_step(s);
    }
    // A simultaneous controlword reset must not clear a persistent math fault.
    if (!trig_ok) {
        telemetry_.warnings |= math_warning;
        if (!previous_output_.enable) {
            trip(Fault::configuration);
        }
    }
    if (command_.calibration_request != calibration_seen_) {
        calibration_seen_ = command_.calibration_request;
        if (state_.state() == State::disabled && command_.calibration_permitted && !command_.operational &&
            fault_ == Fault::none && s.current_calibrated && angle_ok && finite) {
            calibration_ = Calibration::align;
            calibration_tick_ = 0;
            calibration_slow_ = command_.calibration_slow;
            config_.calibrated = 0;
            telemetry_.calibration_reason = CalibrationReason::none;
            telemetry_.calibration_ticks = telemetry_.calibration_points = 0;
            telemetry_.calibration_concentration = telemetry_.calibration_travel = 0;
            calibration_missing_ = calibration_overcurrent_ = 0;
            reset_loops();
        } else {
            (void)fail_calibration(CalibrationReason::not_ready);
        }
    }
    Output output{};
    if (fault_ != Fault::none) {
        // Fast faults update the state immediately, but only the 1 kHz state
        // machine may consume controlword edges (especially fault reset).
        if (state_.state() != State::fault) {
            reset_loops();
        }
        state_.trip();
        if (calibration_ == Calibration::align || calibration_ == Calibration::sweep ||
            calibration_ == Calibration::settle) {
            calibration_ = Calibration::failed;
        }
    } else if (calibration_ == Calibration::align || calibration_ == Calibration::sweep ||
               calibration_ == Calibration::settle) {
        if (!command_.calibration_permitted || command_.operational || command_age_ > 2000) {
            trip(Fault::communication);
            calibration_ = Calibration::failed;
            telemetry_.calibration_reason = CalibrationReason::interrupted;
        } else if (!angle_ok || !finite) {
            (void)fail_calibration(CalibrationReason::encoder);
        } else {
            output = calibration_step(s);
        }
    } else if (state_.state() == State::enabled || state_.state() == State::quick_stop) {
        const auto max_i = config_.current_limit * static_cast<float>(command_.max_torque) / 1000.0F;
        const float compensation = cogging_feedforward(s);
        iq_target_ = std::clamp(iq_feedback_ + compensation, -max_i, max_i);
        if (ticks_ % 20 == 0 && (is_position_mode(command_.mode) || command_.mode == Mode::velocity ||
                                 state_.state() == State::quick_stop)) {
            velocity_pi_.track(iq_target_ - compensation, iq_feedback_);
        }
        if (!s.power_ready) {
            // Request wake at neutral duty without winding up a current PI
            // against a sleeping power stage or invalid CSA output.
            d_pi_.reset();
            q_pi_.reset();
            output.enable = true;
        } else if (!trig_ok || !finite) {
            output = previous_output_; // Freeze PI state as well as the last applied duty.
        } else {
            const auto limit = std::max(0.0F, telemetry_.bus_voltage) / sqrt3;
            const Dq requested{d_pi_.step(-currents.d, dt, limit),
                               q_pi_.step(iq_target_ - currents.q, dt, limit)};
            const auto applied = limit_vector(requested, limit);
            d_pi_.track(applied.d, requested.d);
            q_pi_.track(applied.q, requested.q);
            output = {
                svpwm(inverse_park(applied, voltage_sc.sine, voltage_sc.cosine), telemetry_.bus_voltage),
                true};
        }
    }
    const auto common = (output.duty.a + output.duty.b + output.duty.c) / 3.0F;
    telemetry_.phase_voltage = output.enable ? Abc{(output.duty.a - common) * telemetry_.bus_voltage,
                                                   (output.duty.b - common) * telemetry_.bus_voltage,
                                                   (output.duty.c - common) * telemetry_.bus_voltage}
                                             : Abc{};
    telemetry_.statusword = state_.status() | (setpoint_ack_ ? 0x1000 : 0) | (telemetry_.warnings ? 0x80 : 0);
    if (state_.state() == State::enabled && is_position_mode(command_.mode) &&
        std::abs(telemetry_.velocity) < 0.05F) {
        const auto difference = profile_.target - position_counts();
        constexpr auto tolerance_counts = static_cast<int>(0.001F * encoder_cpr);
        const bool reached =
            missed_angles_ == 0
                ? difference >= -tolerance_counts && difference <= tolerance_counts
                : std::abs(count_difference_turns(difference) - telemetry_.velocity * age) < 0.001F;
        if (reached) {
            telemetry_.statusword |= 0x0400;
        }
    }
    telemetry_.fault = fault_;
    telemetry_.calibration = calibration_;
    telemetry_.config = config_;
    telemetry_.iq_target = iq_target_;
    if (fault_ != Fault::none || !angle_initialized_) {
        output.enable = false;
    }
    output.stop = fault_ != Fault::none || (!command_.calibration_permitted && !command_.operational);
    previous_output_ = output;
    return output;
}
} // namespace roller::drive
