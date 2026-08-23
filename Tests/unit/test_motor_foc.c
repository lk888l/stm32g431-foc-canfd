#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "encoder_tracker.h"
#include "foc_math.h"
#include "motor_foc.h"

static int almost_equal(float left, float right, float tolerance)
{
    return fabsf(left - right) <= tolerance;
}

static motor_foc_config_t make_config(motor_foc_strategy_t strategy)
{
    motor_foc_config_t config = {0};
    control_pi_config_t current_pi = {0};

    current_pi.proportional_gain = 0.5f;
    current_pi.integral_gain_per_s = 1000.0f;
    current_pi.anti_windup_gain_per_s = 1000.0f;
    current_pi.output_min = -6.0f;
    current_pi.output_max = 6.0f;
    current_pi.integrator_min = -6.0f;
    current_pi.integrator_max = 6.0f;
    current_pi.max_timestep_s = 0.0001f;

    config.strategy = strategy;
    config.control_period_s = 0.00005f;
    config.pole_pairs = 7.0f;
    config.encoder_direction = 1.0f;
    config.electrical_zero_rad = 0.0f;
    config.electrical_zero_valid = true;
    config.phase_a_current_a_per_count =
        strategy == MOTOR_FOC_CURRENT_CLOSED_LOOP ? 0.01f : 0.0f;
    config.phase_b_current_a_per_count =
        strategy == MOTOR_FOC_CURRENT_CLOSED_LOOP ? 0.01f : 0.0f;
    config.adc_min_valid_count = 8U;
    config.adc_max_valid_count = 4087U;
    config.offset_min_valid_count = 512U;
    config.offset_max_valid_count = 3584U;
    config.offset_max_span_count = 64U;
    config.offset_calibration_samples = 4U;
    config.phase_resistance_ohm = 2.55f;
    config.d_inductance_h = 0.00086f;
    config.q_inductance_h = 0.00086f;
    config.flux_linkage_wb = 0.0035f;
    config.current_limit_a = 1.0f;
    config.bus_nominal_v = 12.0f;
    config.voltage_utilization = 0.9f;
    config.minimum_pulse_s = 0.000001f;
    config.encoder_stale_timeout_us = 2000U;
    config.d_current_pi = current_pi;
    config.q_current_pi = current_pi;
    config.angle_pll.proportional_gain_per_s = 100.0f;
    config.angle_pll.integral_gain_per_s2 = 2500.0f;
    config.angle_pll.maximum_speed_rad_s = 300.0f;
    config.angle_pll.max_timestep_s = 0.01f;
    return config;
}

static void test_lab_voltage_path_and_stale_lockout(void)
{
    motor_foc_t motor;
    motor_foc_config_t config = make_config(MOTOR_FOC_LAB_VOLTAGE_LOOP);
    motor_foc_command_t command = {0};
    const motor_foc_feedback_t *feedback;

    assert(motor_foc_init(&motor, &config));
    motor_foc_set_bus_voltage(&motor, 12.0f, true);
    assert(motor_foc_update_encoder(&motor, 0.25f, 1000U, 0U, true));

    command.enable = true;
    command.lab_q_voltage_v = 1.0f;
    motor_foc_set_command(&motor, &command);
    motor_foc_fast_step(&motor, 2048U, 2048U, 1000U);
    feedback = motor_foc_feedback(&motor);
    assert(feedback != NULL);
    assert(feedback->output_enabled);
    assert(feedback->duty_a >= 0.0f && feedback->duty_a <= 1.0f);
    assert(feedback->duty_b >= 0.0f && feedback->duty_b <= 1.0f);
    assert(feedback->duty_c >= 0.0f && feedback->duty_c <= 1.0f);
    assert(fabsf(feedback->duty_a - 0.5f) > 1.0e-4f ||
           fabsf(feedback->duty_b - 0.5f) > 1.0e-4f ||
           fabsf(feedback->duty_c - 0.5f) > 1.0e-4f);

    motor_foc_fast_step(&motor, 2048U, 2048U, 4001U);
    feedback = motor_foc_feedback(&motor);
    assert(!feedback->encoder_valid);
    assert(!feedback->output_enabled);
    assert(feedback->duty_a == 0.5f);
    assert(feedback->duty_b == 0.5f);
    assert(feedback->duty_c == 0.5f);

    /* A long encoder outage must be recoverable after the safety layer has
     * removed gate drive; otherwise the old PLL timestamp rejects every
     * future sample and only a power cycle can clear the fault. */
    assert(motor_foc_update_encoder(&motor, 0.5f, 20000U, 0U, true));
    motor_foc_fast_step(&motor, 2048U, 2048U, 20000U);
    feedback = motor_foc_feedback(&motor);
    assert(feedback->encoder_valid);
    assert(feedback->output_enabled);
}

static void test_current_calibration_and_software_overcurrent(void)
{
    motor_foc_t motor;
    motor_foc_config_t config =
        make_config(MOTOR_FOC_CURRENT_CLOSED_LOOP);
    motor_foc_command_t command = {0};
    const motor_foc_feedback_t *feedback;
    unsigned int index;

    assert(motor_foc_init(&motor, &config));
    motor_foc_set_bus_voltage(&motor, 12.0f, true);
    assert(motor_foc_update_encoder(&motor, 0.0f, 1000U, 0U, true));
    for (index = 0U; index < 4U; ++index) {
        motor_foc_fast_step(&motor,
                            2000U,
                            2000U,
                            1000U + (uint32_t)index * 50U);
    }
    feedback = motor_foc_feedback(&motor);
    assert(feedback->calibration_complete);
    assert(feedback->current_feedback_valid);

    command.enable = true;
    command.iq_reference_a = 0.1f;
    motor_foc_set_command(&motor, &command);
    motor_foc_fast_step(&motor, 2000U, 2000U, 1200U);
    feedback = motor_foc_feedback(&motor);
    assert(feedback->output_enabled);
    assert(!feedback->current_limit_exceeded);

    motor_foc_fast_step(&motor, 2200U, 2000U, 1250U);
    feedback = motor_foc_feedback(&motor);
    assert(feedback->current_limit_exceeded);
    assert(!feedback->output_enabled);
}

static void test_coupled_dq_voltage_anti_windup(void)
{
    motor_foc_t motor;
    motor_foc_config_t config =
        make_config(MOTOR_FOC_CURRENT_CLOSED_LOOP);
    motor_foc_command_t command = {0};
    const motor_foc_feedback_t *feedback;
    const float voltage_limit = config.bus_nominal_v *
                                CONTROL_INV_SQRT3_F *
                                config.voltage_utilization;
    uint32_t now_us = 1200U;
    unsigned int index;

    /* Neither axis reaches its independent 6 V limit, but their combined
     * request exceeds the inverter's dq voltage circle. */
    config.d_current_pi.proportional_gain = 3.0f;
    config.q_current_pi.proportional_gain = 3.0f;
    assert(motor_foc_init(&motor, &config));
    motor_foc_set_bus_voltage(&motor, config.bus_nominal_v, true);
    assert(motor_foc_update_encoder(&motor, 0.0f, 1000U, 0U, true));
    for (index = 0U; index < config.offset_calibration_samples; ++index) {
        motor_foc_fast_step(&motor,
                            2000U,
                            2000U,
                            1000U + (uint32_t)index * 50U);
    }

    command.enable = true;
    command.id_reference_a = 1.0f;
    command.iq_reference_a = 1.0f;
    motor_foc_set_command(&motor, &command);
    motor_foc_fast_step(&motor, 2000U, 2000U, now_us);
    feedback = motor_foc_feedback(&motor);
    assert(feedback->output_enabled);
    assert(almost_equal(hypotf(feedback->id_voltage_v,
                               feedback->iq_voltage_v),
                        voltage_limit, 2.0e-5f));
    assert(feedback->id_voltage_v < 5.0f);
    assert(feedback->iq_voltage_v < 5.0f);
    assert(almost_equal(motor.d_current_pi.output,
                        feedback->id_voltage_v, 1.0e-6f));
    assert(almost_equal(motor.q_current_pi.output,
                        feedback->iq_voltage_v, 1.0e-6f));
    assert(motor.d_current_pi.integrator < 0.0f);
    assert(motor.q_current_pi.integrator < 0.0f);

    /* Under persistent simultaneous saturation, vector tracking converges to
     * a small finite bias instead of winding either integrator to +/-6 V. */
    for (index = 0U; index < 120U; ++index) {
        now_us += 50U;
        if ((index % 10U) == 0U) {
            assert(motor_foc_update_encoder(&motor,
                                            0.0f,
                                            now_us,
                                            0U,
                                            true));
        }
        motor_foc_fast_step(&motor, 2000U, 2000U, now_us);
        feedback = motor_foc_feedback(&motor);
        assert(feedback->output_enabled);
        assert(isfinite(motor.d_current_pi.integrator));
        assert(isfinite(motor.q_current_pi.integrator));
    }
    assert(fabsf(motor.d_current_pi.integrator) < 0.25f);
    assert(fabsf(motor.q_current_pi.integrator) < 0.25f);
    assert(almost_equal(motor.d_current_pi.output,
                        feedback->id_voltage_v, 1.0e-6f));
    assert(almost_equal(motor.q_current_pi.output,
                        feedback->iq_voltage_v, 1.0e-6f));

    /* Removing the demand must leave the voltage circle immediately; a
     * wound-up integrator would keep the output pinned for many cycles. */
    command.id_reference_a = 0.0f;
    command.iq_reference_a = 0.0f;
    motor_foc_set_command(&motor, &command);
    now_us += 50U;
    assert(motor_foc_update_encoder(&motor, 0.0f, now_us, 0U, true));
    motor_foc_fast_step(&motor, 2000U, 2000U, now_us);
    feedback = motor_foc_feedback(&motor);
    assert(feedback->output_enabled);
    assert(hypotf(feedback->id_voltage_v,
                  feedback->iq_voltage_v) < 0.35f);
    assert(almost_equal(motor.d_current_pi.output,
                        feedback->id_voltage_v, 1.0e-6f));
    assert(almost_equal(motor.q_current_pi.output,
                        feedback->iq_voltage_v, 1.0e-6f));
}

static void test_current_frontend_diagnostics(void)
{
    motor_foc_t motor;
    motor_foc_config_t config =
        make_config(MOTOR_FOC_CURRENT_CLOSED_LOOP);
    const motor_foc_feedback_t *feedback;

    assert(motor_foc_init(&motor, &config));
    motor_foc_fast_step(&motor, 0U, 2000U, 1000U);
    feedback = motor_foc_feedback(&motor);
    assert(!feedback->current_feedback_valid);
    assert(feedback->control_math_fault);

    assert(motor_foc_init(&motor, &config));
    motor_foc_fast_step(&motor, 1900U, 2000U, 1000U);
    motor_foc_fast_step(&motor, 2000U, 2000U, 1050U);
    motor_foc_fast_step(&motor, 1900U, 2000U, 1100U);
    motor_foc_fast_step(&motor, 2000U, 2000U, 1150U);
    feedback = motor_foc_feedback(&motor);
    assert(feedback->calibration_complete);
    assert(!feedback->current_feedback_valid);
    assert(feedback->control_math_fault);
}

static void test_multiturn_tracker_wrap_and_zero(void)
{
    encoder_tracker_t tracker = {0};
    bool saturated = true;
    int32_t q16;

    assert(encoder_tracker_update(&tracker, 4090U, 4096U));
    assert(encoder_tracker_update(&tracker, 2U, 4096U));
    assert(encoder_tracker_position_rad(&tracker) > 0.0f);
    q16 = encoder_tracker_position_q16_16(&tracker, &saturated);
    assert(q16 > 0);
    assert(!saturated);

    encoder_tracker_zero_at_current(&tracker);
    assert(fabsf(encoder_tracker_position_rad(&tracker)) < 1.0e-7f);
    assert(encoder_tracker_update(&tracker, 12U, 4096U));
    assert(encoder_tracker_position_rad(&tracker) > 0.0f);
}

int main(void)
{
    test_lab_voltage_path_and_stale_lockout();
    test_current_calibration_and_software_overcurrent();
    test_coupled_dq_voltage_anti_windup();
    test_current_frontend_diagnostics();
    test_multiturn_tracker_wrap_and_zero();
    puts("motor FOC unit tests passed");
    return 0;
}
