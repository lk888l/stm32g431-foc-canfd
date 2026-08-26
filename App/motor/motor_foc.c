#include "motor_foc.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "foc_math.h"

static bool finite_positive(float value)
{
    return isfinite(value) && (value > 0.0f);
}

static bool valid_encoder_direction(float direction)
{
    return isfinite(direction) &&
           ((direction == 1.0f) || (direction == -1.0f));
}

static float absolute_max3(float a, float b, float c)
{
    const float ab = fmaxf(fabsf(a), fabsf(b));
    return fmaxf(ab, fabsf(c));
}

static void neutral_output(motor_foc_t *motor)
{
    motor->feedback.duty_a = 0.5f;
    motor->feedback.duty_b = 0.5f;
    motor->feedback.duty_c = 0.5f;
    motor->feedback.id_voltage_v = 0.0f;
    motor->feedback.iq_voltage_v = 0.0f;
    motor->feedback.alpha_voltage_v = 0.0f;
    motor->feedback.beta_voltage_v = 0.0f;
    motor->feedback.output_enabled = false;
}

bool motor_foc_init(motor_foc_t *motor, const motor_foc_config_t *config)
{
    if ((motor == NULL) || (config == NULL) ||
        !finite_positive(config->control_period_s) ||
        !finite_positive(config->pole_pairs) ||
        !valid_encoder_direction(config->encoder_direction) ||
        !finite_positive(config->current_limit_a) ||
        !finite_positive(config->bus_nominal_v) ||
        !finite_positive(config->voltage_utilization) ||
        (config->voltage_utilization > 1.0f) ||
        (config->offset_calibration_samples == 0U) ||
        (config->adc_min_valid_count >= config->adc_max_valid_count) ||
        (config->offset_min_valid_count >= config->offset_max_valid_count) ||
        (config->offset_min_valid_count < config->adc_min_valid_count) ||
        (config->offset_max_valid_count > config->adc_max_valid_count) ||
        (config->offset_max_span_count == 0U) ||
        (config->encoder_stale_timeout_us == 0U) ||
        !control_pi_config_is_valid(&config->d_current_pi) ||
        !control_pi_config_is_valid(&config->q_current_pi) ||
        !control_angle_pll_config_is_valid(&config->angle_pll)) {
        return false;
    }

    memset(motor, 0, sizeof(*motor));
    motor->config = *config;
    motor->phase_a_offset_min_count = UINT16_MAX;
    motor->phase_b_offset_min_count = UINT16_MAX;
    if (!control_pi_init(&motor->d_current_pi, &config->d_current_pi) ||
        !control_pi_init(&motor->q_current_pi, &config->q_current_pi)) {
        return false;
    }
    motor->feedback.bus_voltage_v = config->bus_nominal_v;
    motor->feedback.bus_voltage_valid = false;
    motor->feedback.electrical_calibration_valid =
        config->electrical_zero_valid;
    neutral_output(motor);
    motor->initialized = true;
    return true;
}

void motor_foc_set_command(motor_foc_t *motor,
                           const motor_foc_command_t *command)
{
    if ((motor == NULL) || (command == NULL) || !motor->initialized) {
        return;
    }
    motor->command = *command;
    if (!isfinite(motor->command.id_reference_a) ||
        !isfinite(motor->command.iq_reference_a) ||
        !isfinite(motor->command.lab_q_voltage_v) ||
        !isfinite(motor->command.lab_alpha_voltage_v) ||
        !isfinite(motor->command.lab_beta_voltage_v) ||
        (motor->command.lab_fixed_stator_voltage &&
         (motor->config.strategy != MOTOR_FOC_LAB_VOLTAGE_LOOP))) {
        memset(&motor->command, 0, sizeof(motor->command));
        motor->feedback.control_math_fault = true;
    }
    motor->command.id_reference_a =
        control_clampf(motor->command.id_reference_a,
                       -motor->config.current_limit_a,
                       motor->config.current_limit_a);
    motor->command.iq_reference_a =
        control_clampf(motor->command.iq_reference_a,
                       -motor->config.current_limit_a,
                       motor->config.current_limit_a);
}

bool motor_foc_set_electrical_calibration(motor_foc_t *motor,
                                          float encoder_direction,
                                          float electrical_zero_rad,
                                          bool valid)
{
    if ((motor == NULL) || !motor->initialized ||
        !valid_encoder_direction(encoder_direction) ||
        !isfinite(electrical_zero_rad)) {
        return false;
    }

    motor->config.encoder_direction = encoder_direction;
    motor->config.electrical_zero_rad =
        control_angle_wrap_pm_pi(electrical_zero_rad);
    motor->config.electrical_zero_valid = valid;
    motor->feedback.electrical_calibration_valid = valid;
    motor->feedback.encoder_valid = false;
    motor->feedback.mechanical_position_rad = 0.0f;
    motor->feedback.mechanical_velocity_rad_s = 0.0f;
    motor->feedback.electrical_angle_rad = 0.0f;
    motor->last_encoder_timestamp_us = 0U;
    motor->last_directed_encoder_angle_rad = 0.0f;
    motor->angle_pll = (control_angle_pll_t){0};
    (void)control_pi_reset(&motor->d_current_pi, 0.0f, 0.0f);
    (void)control_pi_reset(&motor->q_current_pi, 0.0f, 0.0f);
    neutral_output(motor);
    return true;
}

void motor_foc_set_bus_voltage(motor_foc_t *motor,
                               float bus_voltage_v,
                               bool valid)
{
    if ((motor == NULL) || !motor->initialized) {
        return;
    }
    motor->feedback.bus_voltage_valid =
        valid && finite_positive(bus_voltage_v);
    if (motor->feedback.bus_voltage_valid) {
        motor->feedback.bus_voltage_v = bus_voltage_v;
    }
}

bool motor_foc_update_encoder(motor_foc_t *motor,
                              float wrapped_mechanical_angle_rad,
                              uint32_t timestamp_us,
                              uint32_t health_flags,
                              bool valid)
{
    float directed_angle;

    if ((motor == NULL) || !motor->initialized) {
        return false;
    }
    motor->feedback.encoder_health = health_flags;
    if (!valid || !isfinite(wrapped_mechanical_angle_rad)) {
        motor->feedback.encoder_valid = false;
        return false;
    }
    directed_angle = control_angle_wrap_pm_pi(
        motor->config.encoder_direction * wrapped_mechanical_angle_rad);
    motor->last_directed_encoder_angle_rad = directed_angle;

    if (!motor->angle_pll.initialized) {
        if (!control_angle_pll_init(&motor->angle_pll,
                                    &motor->config.angle_pll,
                                    directed_angle)) {
            motor->feedback.encoder_valid = false;
            return false;
        }
    } else {
        const uint32_t elapsed_us = timestamp_us - motor->last_encoder_timestamp_us;
        const float timestep_s = (float)elapsed_us * 1.0e-6f;
        bool pll_ok;

        /* A sensor outage longer than the PLL's admissible step cannot be
         * integrated safely.  Re-acquire angle with zero estimated speed so
         * a latched feedback fault can be diagnosed and reset after the gate
         * has been removed; otherwise every later sample would be rejected
         * forever against the same stale timestamp. */
        if (timestep_s > motor->config.angle_pll.max_timestep_s) {
            pll_ok = control_angle_pll_init(&motor->angle_pll,
                                            &motor->config.angle_pll,
                                            directed_angle);
        } else {
            pll_ok = control_angle_pll_update(&motor->angle_pll,
                                              directed_angle,
                                              timestep_s);
        }
        if (!pll_ok) {
            motor->feedback.encoder_valid = false;
            return false;
        }
    }

    motor->last_encoder_timestamp_us = timestamp_us;
    motor->feedback.encoder_valid = true;
    motor->feedback.mechanical_position_rad =
        (float)motor->angle_pll.continuous_angle_precise_rad;
    motor->feedback.mechanical_velocity_rad_s = motor->angle_pll.speed_rad_s;
    return true;
}

static bool compute_currents(motor_foc_t *motor,
                             uint16_t phase_a_raw,
                             uint16_t phase_b_raw,
                             float electrical_angle_rad)
{
    control_abc_t phase;
    control_alphabeta_t stationary;
    control_dq_t rotating;

    if (motor->config.strategy != MOTOR_FOC_CURRENT_CLOSED_LOOP) {
        motor->feedback.calibration_complete = true;
        motor->feedback.current_feedback_valid = false;
        motor->feedback.phase_a_current_a = 0.0f;
        motor->feedback.phase_b_current_a = 0.0f;
        motor->feedback.phase_c_current_a = 0.0f;
        motor->feedback.id_current_a = 0.0f;
        motor->feedback.iq_current_a = 0.0f;
        return true;
    }

    if ((phase_a_raw < motor->config.adc_min_valid_count) ||
        (phase_a_raw > motor->config.adc_max_valid_count) ||
        (phase_b_raw < motor->config.adc_min_valid_count) ||
        (phase_b_raw > motor->config.adc_max_valid_count)) {
        motor->feedback.current_feedback_valid = false;
        motor->feedback.control_math_fault = true;
        return false;
    }

    if (!motor->feedback.calibration_complete) {
        motor->phase_a_offset_accumulator += phase_a_raw;
        motor->phase_b_offset_accumulator += phase_b_raw;
        if (phase_a_raw < motor->phase_a_offset_min_count) {
            motor->phase_a_offset_min_count = phase_a_raw;
        }
        if (phase_a_raw > motor->phase_a_offset_max_count) {
            motor->phase_a_offset_max_count = phase_a_raw;
        }
        if (phase_b_raw < motor->phase_b_offset_min_count) {
            motor->phase_b_offset_min_count = phase_b_raw;
        }
        if (phase_b_raw > motor->phase_b_offset_max_count) {
            motor->phase_b_offset_max_count = phase_b_raw;
        }
        ++motor->offset_samples_collected;
        if (motor->offset_samples_collected >=
            motor->config.offset_calibration_samples) {
            motor->phase_a_offset_count =
                (float)motor->phase_a_offset_accumulator /
                (float)motor->offset_samples_collected;
            motor->phase_b_offset_count =
                (float)motor->phase_b_offset_accumulator /
                (float)motor->offset_samples_collected;
            motor->feedback.calibration_complete = true;
            motor->feedback.current_feedback_valid =
                isfinite(motor->config.phase_a_current_a_per_count) &&
                isfinite(motor->config.phase_b_current_a_per_count) &&
                (fabsf(motor->config.phase_a_current_a_per_count) > 0.0f) &&
                (fabsf(motor->config.phase_b_current_a_per_count) > 0.0f) &&
                (motor->phase_a_offset_count >=
                 (float)motor->config.offset_min_valid_count) &&
                (motor->phase_a_offset_count <=
                 (float)motor->config.offset_max_valid_count) &&
                (motor->phase_b_offset_count >=
                 (float)motor->config.offset_min_valid_count) &&
                (motor->phase_b_offset_count <=
                 (float)motor->config.offset_max_valid_count) &&
                ((uint16_t)(motor->phase_a_offset_max_count -
                            motor->phase_a_offset_min_count) <=
                 motor->config.offset_max_span_count) &&
                ((uint16_t)(motor->phase_b_offset_max_count -
                            motor->phase_b_offset_min_count) <=
                 motor->config.offset_max_span_count);
            if (!motor->feedback.current_feedback_valid) {
                motor->feedback.control_math_fault = true;
            }
        }
        return false;
    }
    if (!motor->feedback.current_feedback_valid) {
        return false;
    }

    phase.a = ((float)phase_a_raw - motor->phase_a_offset_count) *
              motor->config.phase_a_current_a_per_count;
    phase.b = ((float)phase_b_raw - motor->phase_b_offset_count) *
              motor->config.phase_b_current_a_per_count;
    phase.c = -phase.a - phase.b;
    if (!control_clarke(&phase, &stationary) ||
        !control_park(&stationary, electrical_angle_rad, &rotating)) {
        return false;
    }

    motor->feedback.phase_a_current_a = phase.a;
    motor->feedback.phase_b_current_a = phase.b;
    motor->feedback.phase_c_current_a = phase.c;
    motor->feedback.id_current_a = rotating.d;
    motor->feedback.iq_current_a = rotating.q;
    motor->feedback.current_limit_exceeded =
        absolute_max3(phase.a, phase.b, phase.c) >
        (motor->config.current_limit_a * 1.10f);
    return !motor->feedback.current_limit_exceeded;
}

void motor_foc_fast_step(motor_foc_t *motor,
                         uint16_t phase_a_raw,
                         uint16_t phase_b_raw,
                         uint32_t now_us)
{
    control_dq_t voltage_dq = {0.0f, 0.0f};
    control_dq_t limited_dq;
    control_dq_t applied_dq;
    control_alphabeta_t voltage_ab;
    control_svpwm_output_t modulation;
    control_svpwm_config_t modulation_config;
    control_pi_t d_current_pi_next = {0};
    control_pi_t q_current_pi_next = {0};
    float predicted_mechanical_angle;
    float electrical_speed;
    float voltage_limit;
    bool current_data_ok;
    bool current_pi_update_pending = false;
    const bool fixed_stator_voltage =
        (motor != NULL) &&
        (motor->config.strategy == MOTOR_FOC_LAB_VOLTAGE_LOOP) &&
        motor->command.lab_fixed_stator_voltage;

    if ((motor == NULL) || !motor->initialized) {
        return;
    }
    motor->feedback.control_math_fault = false;
    motor->feedback.encoder_age_us = now_us - motor->last_encoder_timestamp_us;
    if (motor->feedback.encoder_age_us >
        motor->config.encoder_stale_timeout_us) {
        motor->feedback.encoder_valid = false;
    }

    if (motor->angle_pll.initialized) {
        if (motor->config.direct_encoder_angle_for_commutation) {
            /* AS5600 Lab operation is low-speed and its observed velocity can
             * contain a large transient when the rotor breaks static
             * friction. Do not turn that filtered-speed transient back into
             * a predictive electrical-angle error. At 1 kHz, the direct
             * sample delay is only 2 electrical degrees at 5 rad/s. */
            predicted_mechanical_angle =
                motor->last_directed_encoder_angle_rad;
        } else {
            predicted_mechanical_angle =
                motor->angle_pll.wrapped_angle_rad +
                motor->angle_pll.speed_rad_s *
                    ((float)motor->feedback.encoder_age_us * 1.0e-6f);
        }
    } else {
        predicted_mechanical_angle = 0.0f;
    }
    motor->feedback.electrical_angle_rad = control_angle_wrap_0_2pi(
        motor->config.pole_pairs * predicted_mechanical_angle +
        motor->config.electrical_zero_rad);
    electrical_speed =
        motor->config.pole_pairs * motor->angle_pll.speed_rad_s;

    current_data_ok = compute_currents(motor,
                                       phase_a_raw,
                                       phase_b_raw,
                                       motor->feedback.electrical_angle_rad);

    if (!motor->command.enable ||
        (!fixed_stator_voltage &&
         (!motor->feedback.encoder_valid ||
          !motor->feedback.electrical_calibration_valid)) ||
        !motor->feedback.bus_voltage_valid) {
        (void)control_pi_reset(&motor->d_current_pi, 0.0f, 0.0f);
        (void)control_pi_reset(&motor->q_current_pi, 0.0f, 0.0f);
        neutral_output(motor);
        return;
    }

    voltage_limit = motor->feedback.bus_voltage_v *
                    CONTROL_INV_SQRT3_F *
                    motor->config.voltage_utilization;
    if (motor->config.strategy == MOTOR_FOC_CURRENT_CLOSED_LOOP) {
        control_pi_result_t d_result;
        control_pi_result_t q_result;
        const float d_error =
            motor->command.id_reference_a - motor->feedback.id_current_a;
        const float q_error =
            motor->command.iq_reference_a - motor->feedback.iq_current_a;
        const float d_feedforward =
            motor->config.phase_resistance_ohm *
                motor->command.id_reference_a -
            electrical_speed * motor->config.q_inductance_h *
                motor->feedback.iq_current_a;
        const float q_feedforward =
            motor->config.phase_resistance_ohm *
                motor->command.iq_reference_a +
            electrical_speed *
                (motor->config.d_inductance_h * motor->feedback.id_current_a +
                 motor->config.flux_linkage_wb);

        /* Keep both current-loop states transactional.  A rejected q-axis
         * update, vector transform, or modulator calculation must not leave
         * only the d-axis integrator advanced. */
        d_current_pi_next = motor->d_current_pi;
        q_current_pi_next = motor->q_current_pi;
        if (!current_data_ok ||
            !control_pi_step(&d_current_pi_next,
                             d_error,
                             d_feedforward,
                             motor->config.control_period_s,
                             &d_result) ||
            !control_pi_step(&q_current_pi_next,
                             q_error,
                             q_feedforward,
                             motor->config.control_period_s,
                             &q_result)) {
            motor->feedback.control_math_fault = true;
            neutral_output(motor);
            return;
        }
        voltage_dq.d = d_result.output;
        voltage_dq.q = q_result.output;
        current_pi_update_pending = true;
    } else if (!fixed_stator_voltage) {
        voltage_dq.d = 0.0f;
        voltage_dq.q = control_clampf(motor->command.lab_q_voltage_v,
                                      -voltage_limit,
                                      voltage_limit);
    }

    if (fixed_stator_voltage) {
        control_dq_t requested_ab;

        requested_ab.d = motor->command.lab_alpha_voltage_v;
        requested_ab.q = motor->command.lab_beta_voltage_v;
        if (!control_limit_dq_voltage(&requested_ab,
                                      voltage_limit,
                                      &limited_dq,
                                      NULL)) {
            motor->feedback.control_math_fault = true;
            neutral_output(motor);
            return;
        }
        voltage_ab.alpha = limited_dq.d;
        voltage_ab.beta = limited_dq.q;
    } else if (!control_limit_dq_voltage(&voltage_dq,
                                         voltage_limit,
                                         &limited_dq,
                                         NULL) ||
               !control_inverse_park(&limited_dq,
                                     motor->feedback.electrical_angle_rad,
                                     &voltage_ab)) {
            motor->feedback.control_math_fault = true;
            neutral_output(motor);
            return;
    }

    modulation_config.dc_bus_voltage_v = motor->feedback.bus_voltage_v;
    modulation_config.pwm_period_s = motor->config.control_period_s;
    modulation_config.minimum_pulse_s = motor->config.minimum_pulse_s;
    if (!control_svpwm(&voltage_ab, &modulation_config, &modulation)) {
        motor->feedback.control_math_fault = true;
        neutral_output(motor);
        return;
    }

    /* SVPWM may impose a still tighter minimum-pulse limit.  Track the final
     * d/q voltage that reaches the actuator, not merely the independently
     * limited PI outputs. */
    if (fixed_stator_voltage) {
        applied_dq.d = 0.0f;
        applied_dq.q = 0.0f;
    } else {
        applied_dq.d = limited_dq.d * modulation.voltage_scale;
        applied_dq.q = limited_dq.q * modulation.voltage_scale;
    }
    if (!isfinite(applied_dq.d) || !isfinite(applied_dq.q) ||
        (current_pi_update_pending &&
         !control_pi_track_applied_pair(&d_current_pi_next,
                                        applied_dq.d,
                                        &q_current_pi_next,
                                        applied_dq.q,
                                        motor->config.control_period_s))) {
        motor->feedback.control_math_fault = true;
        neutral_output(motor);
        return;
    }
    if (current_pi_update_pending) {
        motor->d_current_pi = d_current_pi_next;
        motor->q_current_pi = q_current_pi_next;
    }

    motor->feedback.id_voltage_v = applied_dq.d;
    motor->feedback.iq_voltage_v = applied_dq.q;
    motor->feedback.alpha_voltage_v =
        modulation.applied_voltage_v.alpha;
    motor->feedback.beta_voltage_v =
        modulation.applied_voltage_v.beta;
    motor->feedback.duty_a = modulation.duty_a;
    motor->feedback.duty_b = modulation.duty_b;
    motor->feedback.duty_c = modulation.duty_c;
    motor->feedback.output_enabled = true;
}

const motor_foc_feedback_t *motor_foc_feedback(const motor_foc_t *motor)
{
    return (motor != NULL) ? &motor->feedback : NULL;
}
