#include "roller/drive/controller.hpp"
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
    velocity_setpoint_ = 0;
    position_setpoint_ = telemetry_.position;
    target_position_ = position_setpoint_;
    setpoint_ack_ = false;
    following_ticks_ = 0;
}
void Controller::outer_step(const Sample& s) noexcept {
    const bool resetting = (command_.controlword & 0x80) && !(previous_cw_ & 0x80);
    const bool inputs_ok =
        s.encoder_ok && s.adc_ok && s.driver_ok && s.current_calibrated && s.bus_voltage >= 4.5F &&
        s.bus_voltage <= 35.0F && s.mcu_temperature < 125.0F && std::isfinite(s.current.a) &&
        std::isfinite(s.current.b) && std::isfinite(s.current.c) &&
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
    if (next == State::quick_stop) {
        velocity_setpoint_ = slew(velocity_setpoint_, 0, command_.quick_deceleration, 0.001F);
        iq_target_ = velocity_pi_.step((velocity_setpoint_ - telemetry_.velocity) * tau, 0.001F, max_i);
    } else {
        switch (command_.mode) {
        case Mode::single_turn_position:
        case Mode::multi_turn_position: {
            const bool trigger = (command_.controlword & 0x10) && !(previous_cw_ & 0x10);
            if (!(command_.controlword & 0x10)) {
                setpoint_ack_ = false;
            }
            if (trigger) {
                const bool relative = (command_.controlword & 0x40) != 0;
                if (command_.mode == Mode::single_turn_position) {
                    // Resolve once at the set-point edge. Re-wrapping against
                    // the live position would let the target jump by a turn.
                    const auto delta = command_.position - (relative ? 0.0F : telemetry_.position);
                    // Exactly half a turn is consistently taken negatively.
                    target_position_ = telemetry_.position + delta - std::floor(delta + 0.5F);
                } else {
                    target_position_ = command_.position + (relative ? telemetry_.position : 0.0F);
                }
                setpoint_ack_ = true;
            }
            const auto error = target_position_ - position_setpoint_;
            const auto braking = std::sqrt(2.0F * command_.deceleration * std::abs(error));
            const auto desired = (command_.controlword & 0x100)
                                     ? 0.0F
                                     : std::copysign(std::min(command_.profile_velocity, braking), error);
            velocity_setpoint_ = velocity_ramp(velocity_setpoint_, desired, command_.acceleration,
                                               command_.deceleration, 0.001F);
            const auto increment = velocity_setpoint_ * 0.001F;
            if (increment * error >= 0 && std::abs(increment) >= std::abs(error)) {
                position_setpoint_ = target_position_;
                velocity_setpoint_ = 0;
            } else {
                position_setpoint_ += increment;
            }
            const auto v = std::clamp(velocity_setpoint_ + 10.0F * (position_setpoint_ - telemetry_.position),
                                      -40.0F, 40.0F);
            iq_target_ = velocity_pi_.step((v - telemetry_.velocity) * tau, 0.001F, max_i);
            if (std::abs(position_setpoint_ - telemetry_.position) > command_.following_window) {
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
            const auto torque = command_.mit_torque +
                                command_.mit_kp * (command_.mit_position - telemetry_.position) +
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
    previous_cw_ = command_.controlword;
}
Output Controller::calibration_step(const Sample& s) noexcept {
    // Pole pairs are known from the motor specification. Never replace 11 with
    // an apparently plausible count computed from a slipping rotor's endpoints.
    ++calibration_tick_;
    float theta = 0;
    if (calibration_ == Calibration::align && calibration_tick_ >= 10000) {
        calibration_start_ = tracker_.position();
        calibration_segment_start_ = calibration_start_;
        calibration_tick_ = 0;
        calibration_segments_ = 0;
        calibration_direction_ = 0;
        offset_count_ = 0;
        offset_sine_ = 0;
        offset_cosine_ = 0;
        calibration_ = Calibration::sweep;
    } else if (calibration_ == Calibration::sweep) {
        const bool reverse = calibration_tick_ > 60000;
        const float travel = 4.0F * tau *
                             static_cast<float>(reverse ? 120000 - calibration_tick_ : calibration_tick_) /
                             60000.0F;
        theta = wrap(travel);
        if (calibration_tick_ % 10000 == 0) {
            const float moved = tracker_.position() - calibration_segment_start_;
            const float expected = (4.0F * tau / 6.0F) / static_cast<float>(config_.pole_pairs);
            if (!calibration_direction_) {
                calibration_direction_ = moved >= 0 ? 1 : -1;
            }
            const float signed_motion =
                moved * static_cast<float>(calibration_direction_) * (reverse ? -1.0F : 1.0F);
            if (signed_motion < expected * 0.65F || signed_motion > expected * 1.35F) {
                calibration_ = Calibration::failed;
                trip(Fault::calibration);
                return {};
            }
            calibration_segment_start_ = tracker_.position();
            ++calibration_segments_;
        }
        if (calibration_direction_ && calibration_tick_ % 20 == 0 && calibration_tick_ > 10000 &&
            (calibration_tick_ < 60000 || calibration_tick_ > 70000)) {
            const auto offset = wrap(static_cast<float>(calibration_direction_) *
                                         static_cast<float>(config_.pole_pairs) * s.angle -
                                     theta);
            const auto sc = trig_(offset);
            offset_sine_ += sc.sine;
            offset_cosine_ += sc.cosine;
            ++offset_count_;
        }
        if (calibration_tick_ >= 120000) {
            calibration_ = Calibration::settle;
            calibration_tick_ = 0;
            theta = 0;
        }
    } else if (calibration_ == Calibration::settle && calibration_tick_ >= 10000) {
        const float concentration = std::hypot(offset_sine_, offset_cosine_) /
                                    static_cast<float>(std::max<std::uint32_t>(1U, offset_count_));
        if (calibration_segments_ != 12 || concentration < 0.95F ||
            std::abs(tracker_.position() - calibration_start_) > 0.15F ||
            std::abs(tracker_.velocity()) > 0.1F) {
            calibration_ = Calibration::failed;
            trip(Fault::calibration);
            return {};
        }
        config_.direction = calibration_direction_;
        config_.electrical_offset = wrap(std::atan2(offset_sine_, offset_cosine_));
        config_.calibrated = 1;
        calibration_ = Calibration::complete;
        reset_loops();
        return {};
    }
    const auto sc = trig_(theta);
    return {svpwm({1.2F * sc.cosine, 1.2F * sc.sine}, s.bus_voltage), true};
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
    const float elapsed_angle = dt * static_cast<float>(missed_angles_ + 1U);
    bool angle_ok = s.encoder_ok && tracker_.update(s.angle, elapsed_angle);
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
    telemetry_.position = static_cast<float>(config_.direction) * (tracker_.position() + speed * age) / tau;
    telemetry_.velocity = static_cast<float>(config_.direction) * speed / tau;
    if (std::isfinite(s.bus_voltage)) {
        telemetry_.bus_voltage = s.bus_voltage;
    }
    if (std::isfinite(s.mcu_temperature)) {
        telemetry_.mcu_temperature = s.mcu_temperature;
    }
    const float electrical_speed =
        static_cast<float>(config_.direction) * static_cast<float>(config_.pole_pairs) * speed;
    const float base =
        static_cast<float>(config_.direction) * static_cast<float>(config_.pole_pairs) * last_angle_ -
        config_.electrical_offset;
    const auto [sine, cosine] = trig_(wrap(base + electrical_speed * (age + s.current_angle_delay)));
    const auto voltage_sc = trig_(wrap(base + electrical_speed * (age + s.voltage_angle_delay)));
    const bool trig_ok = std::isfinite(sine) && std::isfinite(cosine) && std::isfinite(voltage_sc.sine) &&
                         std::isfinite(voltage_sc.cosine);
    const auto mask = static_cast<std::uint8_t>(finite && s.adc_ok ? s.current_mask & 7U : 0U);
    if (mask != 7) {
        ++telemetry_.current_predicted;
        telemetry_.warnings |= current_prediction;
    }
    // Kt=1.5*p*flux in the amplitude-invariant dq convention (an initial estimate).
    const float emf_q = config_.torque_constant * telemetry_.velocity * tau / 1.5F;
    const auto emf = trig_ok ? inverse_park({0, emf_q}, sine, cosine) : AlphaBeta{};
    const bool applied_finite =
        std::isfinite(s.applied_voltage.alpha) && std::isfinite(s.applied_voltage.beta);
    const auto voltage =
        s.applied_voltage_valid && applied_finite ? s.applied_voltage : clarke(telemetry_.phase_voltage);
    const auto estimated = observer_.update(finite ? s.current : Abc{}, mask, voltage, emf, dt);
    telemetry_.phase_current = estimated;
    const auto currents = trig_ok ? park(clarke(estimated), sine, cosine) : Dq{};
    telemetry_.id = currents.d;
    telemetry_.iq = currents.q;
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
            config_.calibrated = 0;
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
        } else if (!angle_ok || !finite) {
            calibration_ = Calibration::failed;
            trip(Fault::calibration);
        } else {
            output = calibration_step(s);
        }
    } else if (state_.state() == State::enabled || state_.state() == State::quick_stop) {
        if (!trig_ok || !finite) {
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
        std::abs(target_position_ - telemetry_.position) < 0.001F && std::abs(telemetry_.velocity) < 0.05F) {
        telemetry_.statusword |= 0x0400;
    }
    telemetry_.fault = fault_;
    telemetry_.calibration = calibration_;
    telemetry_.config = config_;
    if (fault_ != Fault::none || !angle_initialized_) {
        output.enable = false;
    }
    previous_output_ = output;
    return output;
}
} // namespace roller::drive
