#include "roller/drive/controller.hpp"
#include <algorithm>
#include <array>
#include <cmath>

namespace roller::drive {
using namespace control;
bool Controller::valid_config(const MotorConfig& c) noexcept {
    return std::isfinite(c.electrical_offset) && c.electrical_offset >= 0 && c.electrical_offset < tau &&
           std::isfinite(c.torque_constant) && c.torque_constant > 0.001F && c.torque_constant <= 1.0F &&
           std::isfinite(c.current_limit) && c.current_limit > 0 && c.current_limit <= 0.5F &&
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
    const bool mode = c.mode == Mode::none || c.mode == Mode::position || c.mode == Mode::velocity ||
                      c.mode == Mode::torque || c.mode == Mode::mit;
    return mode && std::abs(c.position) <= 100000 && std::abs(c.mit_position) <= 100000 &&
           std::abs(c.velocity) <= 40 && std::abs(c.mit_velocity) <= 40 && std::abs(c.mit_torque) <= 1 &&
           c.mit_kp >= 0 && c.mit_kp <= 100 && c.mit_kd >= 0 && c.mit_kd <= 20 && c.profile_velocity > 0 &&
           c.profile_velocity <= 40 && c.acceleration > 0 && c.acceleration <= 100 && c.deceleration > 0 &&
           c.deceleration <= 100 && c.quick_deceleration > 0 && c.quick_deceleration <= 100 &&
           c.max_torque <= 1000 && c.torque >= -1000 && c.torque <= 1000 && c.following_window > 0;
}
void Controller::command(const Command& value) noexcept {
    if (!valid_command(value)) {
        trip(Fault::configuration);
        return;
    }
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
        s.bus_voltage <= 16.0F && s.mcu_temperature < 85.0F &&
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
        case Mode::position: {
            const bool trigger = (command_.controlword & 0x10) && !(previous_cw_ & 0x10);
            if (!(command_.controlword & 0x10)) {
                setpoint_ack_ = false;
            }
            if (trigger) {
                target_position_ =
                    command_.position + ((command_.controlword & 0x40) ? telemetry_.position : 0.0F);
                setpoint_ack_ = true;
            }
            const auto error = target_position_ - position_setpoint_;
            const auto braking = std::sqrt(2.0F * command_.deceleration * std::abs(error));
            const auto desired = (command_.controlword & 0x100)
                                     ? 0.0F
                                     : std::copysign(std::min(command_.profile_velocity, braking), error);
            const auto rate = std::abs(desired) < std::abs(velocity_setpoint_) ? command_.deceleration
                                                                               : command_.acceleration;
            velocity_setpoint_ = slew(velocity_setpoint_, desired, rate, 0.001F);
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
                    trip(Fault::following);
                }
            } else {
                following_ticks_ = 0;
            }
            break;
        }
        case Mode::velocity: {
            const auto desired = (command_.controlword & 0x100) ? 0.0F : command_.velocity;
            const auto rate = std::abs(desired) < std::abs(velocity_setpoint_) ? command_.deceleration
                                                                               : command_.acceleration;
            velocity_setpoint_ = slew(velocity_setpoint_, desired, rate, 0.001F);
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
    // Explicit bench command only: 0.6 V, one electrical revolution, then align
    // at zero. Any safety trip aborts before another duty is applied.
    ++calibration_tick_;
    float theta = 0;
    if (calibration_ == Calibration::align && calibration_tick_ >= 10000) {
        calibration_start_ = tracker_.position();
        calibration_tick_ = 0;
        calibration_ = Calibration::sweep;
    } else if (calibration_ == Calibration::sweep) {
        theta = tau * static_cast<float>(calibration_tick_) / 40000.0F;
        if (calibration_tick_ >= 40000) {
            calibration_end_ = tracker_.position();
            const auto travel = calibration_end_ - calibration_start_;
            const auto estimate = tau / std::abs(travel);
            if (!std::isfinite(estimate) || estimate < 0.8F || estimate > 32.2F ||
                std::abs(estimate - std::round(estimate)) > 0.2F) {
                calibration_ = Calibration::failed;
                trip(Fault::calibration);
                return {};
            }
            config_.pole_pairs = static_cast<std::uint16_t>(std::round(estimate));
            config_.direction = travel > 0 ? 1 : -1;
            calibration_ = Calibration::settle;
            calibration_tick_ = 0;
        }
    } else if (calibration_ == Calibration::settle && calibration_tick_ >= 10000) {
        config_.electrical_offset =
            wrap(static_cast<float>(config_.direction) * static_cast<float>(config_.pole_pairs) * s.angle);
        config_.calibrated = 1;
        calibration_ = Calibration::complete;
        telemetry_.config = config_;
        reset_loops();
        return {};
    }
    return {svpwm({0.6F * std::cos(theta), 0.6F * std::sin(theta)}, s.bus_voltage), true};
}
Output Controller::step(const Sample& s) noexcept {
    ++ticks_;
    if (command_age_ < 0xFFFFFFFFU) {
        ++command_age_;
    }
    if (!valid_config(config_)) {
        trip(Fault::configuration);
    }
    const bool finite = std::isfinite(s.bus_voltage) && std::isfinite(s.mcu_temperature) &&
                        std::isfinite(s.current.a) && std::isfinite(s.current.b) &&
                        std::isfinite(s.current.c);
    if (!finite || !s.adc_ok) {
        trip(Fault::configuration);
    }
    if (!s.driver_ok) {
        trip(Fault::driver);
    }
    if (fault_ != Fault::none && (command_.controlword & 0x80) && !(previous_cw_ & 0x80) && s.encoder_ok) {
        tracker_.resynchronize(s.angle);
    }
    const bool angle_ok = s.encoder_ok && tracker_.update(s.angle, dt);
    if (!angle_ok) {
        trip(Fault::encoder);
    }
    if (s.bus_voltage > 16.0F) {
        trip(Fault::overvoltage);
    }
    if (s.bus_voltage < 4.5F) {
        trip(Fault::undervoltage);
    }
    if (s.mcu_temperature >= 85.0F) {
        trip(Fault::temperature);
    }
    if (s.current_calibrated &&
        std::max({std::abs(s.current.a), std::abs(s.current.b), std::abs(s.current.c)}) > 0.75F) {
        trip(Fault::overcurrent);
    }
    if (!command_.network_ok ||
        (command_age_ > 2000 && (state_.state() == State::enabled || state_.state() == State::quick_stop))) {
        trip(Fault::communication);
    }
    telemetry_.samples = ticks_;
    telemetry_.command_sequence = command_.sequence;
    telemetry_.position = static_cast<float>(config_.direction) * tracker_.position() / tau;
    telemetry_.velocity = static_cast<float>(config_.direction) * tracker_.velocity() / tau;
    telemetry_.bus_voltage = s.bus_voltage;
    telemetry_.mcu_temperature = s.mcu_temperature;
    telemetry_.phase_current = s.current;
    const auto angle =
        wrap(static_cast<float>(config_.direction) * static_cast<float>(config_.pole_pairs) * s.angle -
             config_.electrical_offset);
    const auto sine = std::sin(angle), cosine = std::cos(angle);
    const auto currents = (finite && angle_ok) ? park(clarke(s.current), sine, cosine) : Dq{};
    telemetry_.id = currents.d;
    telemetry_.iq = currents.q;
    if (!initialized_) {
        reset_loops();
        initialized_ = true;
    }
    if (ticks_ % 20 == 0) {
        outer_step(s);
    }
    if (command_.calibration_request != calibration_seen_) {
        calibration_seen_ = command_.calibration_request;
        if (state_.state() == State::disabled && !command_.operational && fault_ == Fault::none &&
            s.current_calibrated) {
            calibration_ = Calibration::align;
            calibration_tick_ = 0;
            config_.calibrated = 0;
        }
    }
    Output output{};
    if (fault_ != Fault::none) {
        // Fast faults update the state immediately, but only the 1 kHz state
        // machine may consume controlword edges (especially fault reset).
        state_.trip();
        if (calibration_ == Calibration::align || calibration_ == Calibration::sweep ||
            calibration_ == Calibration::settle) {
            calibration_ = Calibration::failed;
        }
        reset_loops();
    } else if (calibration_ == Calibration::align || calibration_ == Calibration::sweep ||
               calibration_ == Calibration::settle) {
        if (command_.operational || command_age_ > 2000) {
            trip(Fault::communication);
            calibration_ = Calibration::failed;
        } else {
            output = calibration_step(s);
        }
    } else if (state_.state() == State::enabled || state_.state() == State::quick_stop) {
        const auto limit = std::min(4.0F, s.bus_voltage * 0.40F);
        const Dq requested{d_pi_.step(-currents.d, dt, limit),
                           q_pi_.step(iq_target_ - currents.q, dt, limit)};
        const auto applied = limit_vector(requested, limit);
        d_pi_.track(applied.d, requested.d);
        q_pi_.track(applied.q, requested.q);
        output = {svpwm(inverse_park(applied, sine, cosine), s.bus_voltage), true};
    }
    const auto common = (output.duty.a + output.duty.b + output.duty.c) / 3.0F;
    telemetry_.phase_voltage = output.enable && finite ? Abc{(output.duty.a - common) * s.bus_voltage,
                                                             (output.duty.b - common) * s.bus_voltage,
                                                             (output.duty.c - common) * s.bus_voltage}
                                                       : Abc{};
    telemetry_.statusword = state_.status() | (setpoint_ack_ ? 0x1000 : 0);
    if (state_.state() == State::enabled && command_.mode == Mode::position &&
        std::abs(target_position_ - telemetry_.position) < 0.001F && std::abs(telemetry_.velocity) < 0.05F) {
        telemetry_.statusword |= 0x0400;
    }
    telemetry_.fault = fault_;
    telemetry_.calibration = calibration_;
    telemetry_.config = config_;
    if (fault_ != Fault::none) {
        output.enable = false;
    }
    return output;
}
} // namespace roller::drive
