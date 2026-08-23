/*
 * Host-side tests for the hardware-independent control layer.
 *
 * Example build (the project build system may provide an equivalent target):
 *   cc -std=c11 -Wall -Wextra -Werror -IApp/control \
 *      Tests/unit/test_control.c App/control/foc_math.c ... -lm -o test_control
 */

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "../../App/control/angle_pll.h"
#include "../../App/control/control_pi.h"
#include "../../App/control/control_trajectory.h"
#include "../../App/control/foc_math.h"
#include "../../App/control/mit_controller.h"

static int almost_equal(float left, float right, float tolerance)
{
    return fabsf(left - right) <= tolerance;
}

static void test_angle_wrapping(void)
{
    assert(almost_equal(control_angle_wrap_pm_pi(0.0f), 0.0f, 1.0e-6f));
    assert(almost_equal(control_angle_wrap_pm_pi(CONTROL_PI_F),
                        -CONTROL_PI_F, 1.0e-5f));
    assert(almost_equal(control_angle_wrap_pm_pi(-CONTROL_PI_F - 0.2f),
                        CONTROL_PI_F - 0.2f, 1.0e-5f));
    assert(almost_equal(control_angle_wrap_0_2pi(-0.25f),
                        CONTROL_TWO_PI_F - 0.25f, 1.0e-5f));
    assert(control_angle_wrap_pm_pi(NAN) == 0.0f);
}

static void test_transforms(void)
{
    const control_abc_t abc = {1.0f, -0.5f, -0.5f};
    control_alphabeta_t alphabeta;
    control_alphabeta_t reconstructed_alphabeta;
    control_dq_t dq;
    control_abc_t reconstructed_abc;

    assert(control_clarke(&abc, &alphabeta));
    assert(almost_equal(alphabeta.alpha, 1.0f, 1.0e-6f));
    assert(almost_equal(alphabeta.beta, 0.0f, 1.0e-6f));
    assert(control_inverse_clarke(&alphabeta, &reconstructed_abc));
    assert(almost_equal(reconstructed_abc.a, abc.a, 1.0e-6f));
    assert(almost_equal(reconstructed_abc.b, abc.b, 1.0e-6f));
    assert(almost_equal(reconstructed_abc.c, abc.c, 1.0e-6f));

    assert(control_park(&alphabeta, 0.5f * CONTROL_PI_F, &dq));
    assert(almost_equal(dq.d, 0.0f, 1.0e-5f));
    assert(almost_equal(dq.q, -1.0f, 1.0e-5f));
    assert(control_inverse_park(&dq, 0.5f * CONTROL_PI_F,
                                &reconstructed_alphabeta));
    assert(almost_equal(reconstructed_alphabeta.alpha,
                        alphabeta.alpha, 1.0e-5f));
    assert(almost_equal(reconstructed_alphabeta.beta,
                        alphabeta.beta, 1.0e-5f));
}

static void test_voltage_limit_and_svpwm(void)
{
    const control_dq_t requested_dq = {3.0f, 4.0f};
    control_dq_t limited_dq;
    float scale = 0.0f;
    const control_svpwm_config_t pwm = {
        .dc_bus_voltage_v = 24.0f,
        .pwm_period_s = 50.0e-6f,
        .minimum_pulse_s = 2.0e-6f
    };
    control_alphabeta_t voltage = {0.0f, 0.0f};
    control_svpwm_output_t output;

    assert(control_limit_dq_voltage(&requested_dq, 2.5f,
                                    &limited_dq, &scale));
    assert(almost_equal(scale, 0.5f, 1.0e-6f));
    assert(almost_equal(limited_dq.d, 1.5f, 1.0e-6f));
    assert(almost_equal(limited_dq.q, 2.0f, 1.0e-6f));

    assert(control_svpwm(&voltage, &pwm, &output));
    assert(almost_equal(output.duty_a, 0.5f, 1.0e-6f));
    assert(almost_equal(output.duty_b, 0.5f, 1.0e-6f));
    assert(almost_equal(output.duty_c, 0.5f, 1.0e-6f));
    assert(!output.saturated);

    voltage.alpha = 100.0f;
    assert(control_svpwm(&voltage, &pwm, &output));
    assert(output.saturated);
    assert(output.voltage_scale < 1.0f);
    assert(output.duty_a >= 0.04f - 1.0e-6f);
    assert(output.duty_a <= 0.96f + 1.0e-6f);
    assert(output.duty_b >= 0.04f - 1.0e-6f);
    assert(output.duty_b <= 0.96f + 1.0e-6f);
    assert(output.duty_c >= 0.04f - 1.0e-6f);
    assert(output.duty_c <= 0.96f + 1.0e-6f);
}

static void test_extreme_finite_foc_inputs_fail_safe(void)
{
    const control_abc_t overflowing_abc = {FLT_MAX, -FLT_MAX, -FLT_MAX};
    const control_alphabeta_t overflowing_ab = {FLT_MAX, FLT_MAX};
    const control_dq_t extreme_dq = {FLT_MAX, FLT_MAX};
    const control_svpwm_config_t pwm = {
        .dc_bus_voltage_v = 24.0f,
        .pwm_period_s = 50.0e-6f,
        .minimum_pulse_s = 2.0e-6f
    };
    control_alphabeta_t ab;
    control_abc_t abc;
    control_dq_t dq;
    control_dq_t limited;
    control_svpwm_output_t modulation;
    float scale;

    assert(!control_clarke(&overflowing_abc, &ab));
    assert(ab.alpha == 0.0f && ab.beta == 0.0f);
    assert(!control_inverse_clarke(&overflowing_ab, &abc));
    assert(abc.a == 0.0f && abc.b == 0.0f && abc.c == 0.0f);
    assert(!control_park(&overflowing_ab, 0.25f * CONTROL_PI_F, &dq));
    assert(dq.d == 0.0f && dq.q == 0.0f);

    assert(control_limit_dq_voltage(&extreme_dq, 1.0f,
                                    &limited, &scale));
    assert(isfinite(limited.d) && isfinite(limited.q));
    assert(hypotf(limited.d, limited.q) <= 1.0001f);
    assert(scale >= 0.0f && scale < 1.0f);

    assert(control_svpwm(&overflowing_ab, &pwm, &modulation));
    assert(isfinite(modulation.duty_a));
    assert(isfinite(modulation.duty_b));
    assert(isfinite(modulation.duty_c));
    assert(modulation.duty_a >= 0.04f && modulation.duty_a <= 0.96f);
    assert(modulation.duty_b >= 0.04f && modulation.duty_b <= 0.96f);
    assert(modulation.duty_c >= 0.04f && modulation.duty_c <= 0.96f);
}

static void test_pi_anti_windup(void)
{
    const control_pi_config_t config = {
        .proportional_gain = 2.0f,
        .integral_gain_per_s = 5.0f,
        .anti_windup_gain_per_s = 20.0f,
        .output_min = -1.0f,
        .output_max = 1.0f,
        .integrator_min = -1.0f,
        .integrator_max = 1.0f,
        .max_timestep_s = 0.01f
    };
    control_pi_t pi;
    control_pi_result_t result;
    unsigned int iteration;

    assert(control_pi_init(&pi, &config));
    for (iteration = 0U; iteration < 200U; ++iteration) {
        assert(control_pi_step(&pi, 10.0f, 0.0f, 0.001f, &result));
        assert(result.output <= 1.0f);
        assert(result.integrator >= -1.0f);
        assert(result.integrator <= 1.0f);
    }
    assert(result.saturated);
    assert(almost_equal(result.output, 1.0f, 1.0e-6f));

    /* A scheduler stall is rejected and cannot mutate stored controller state. */
    {
        const float integrator_before = pi.integrator;
        assert(!control_pi_step(&pi, 0.0f, 0.0f, 0.1f, &result));
        assert(pi.integrator == integrator_before);
    }
}

static void test_pi_coupled_applied_output_tracking(void)
{
    const control_pi_config_t config = {
        .proportional_gain = 1.0f,
        .integral_gain_per_s = 2.0f,
        .anti_windup_gain_per_s = 10.0f,
        .output_min = -10.0f,
        .output_max = 10.0f,
        .integrator_min = -10.0f,
        .integrator_max = 10.0f,
        .max_timestep_s = 0.01f
    };
    control_pi_t first;
    control_pi_t second;
    control_pi_result_t result;
    float first_integrator_before;
    float first_output_before;
    float second_integrator_before;
    float second_output_before;

    assert(control_pi_init(&first, &config));
    assert(control_pi_init(&second, &config));
    assert(control_pi_step(&first, 2.0f, 0.0f, 0.01f, &result));
    assert(almost_equal(first.integrator, 0.04f, 1.0e-6f));
    assert(almost_equal(first.output, 2.04f, 1.0e-6f));
    assert(control_pi_step(&second, 4.0f, 0.0f, 0.01f, &result));
    assert(almost_equal(second.integrator, 0.08f, 1.0e-6f));
    assert(almost_equal(second.output, 4.08f, 1.0e-6f));

    assert(control_pi_track_applied_pair(&first, 1.0f,
                                         &second, -2.0f,
                                         0.01f));
    assert(almost_equal(first.integrator, -0.064f, 1.0e-6f));
    assert(almost_equal(second.integrator, -0.528f, 1.0e-6f));
    assert(first.output == 1.0f);
    assert(second.output == -2.0f);

    /* A bad second axis must not partially commit the valid first axis. */
    first_integrator_before = first.integrator;
    first_output_before = first.output;
    second_integrator_before = second.integrator;
    second_output_before = second.output;
    assert(!control_pi_track_applied_pair(&first, 0.0f,
                                          &second, NAN,
                                          0.01f));
    assert(first.integrator == first_integrator_before);
    assert(first.output == first_output_before);
    assert(second.integrator == second_integrator_before);
    assert(second.output == second_output_before);

    assert(!control_pi_track_applied_pair(&first, 0.0f,
                                          &second, 0.0f,
                                          0.0f));
    assert(!control_pi_track_applied_pair(&first, 0.0f,
                                          &second, 0.0f,
                                          0.01001f));
    assert(!control_pi_track_applied_pair(&first, 0.0f,
                                          &second,
                                          config.output_max + 0.001f,
                                          0.001f));
    assert(first.integrator == first_integrator_before);
    assert(first.output == first_output_before);
    assert(second.integrator == second_integrator_before);
    assert(second.output == second_output_before);

    /* Exact per-axis bounds are legal applied actuator values. */
    assert(control_pi_track_applied_pair(&first, config.output_max,
                                         &second, config.output_min,
                                         config.max_timestep_s));
    assert(first.output == config.output_max);
    assert(second.output == config.output_min);
    assert(isfinite(first.integrator));
    assert(isfinite(second.integrator));

    first_integrator_before = first.integrator;
    first_output_before = first.output;
    assert(!control_pi_track_applied_pair(&first, 0.0f,
                                          &first, 0.0f,
                                          0.001f));
    assert(first.integrator == first_integrator_before);
    assert(first.output == first_output_before);

    second.output = NAN;
    second_integrator_before = second.integrator;
    assert(!control_pi_track_applied_pair(&first, 0.0f,
                                          &second, 0.0f,
                                          0.001f));
    assert(first.integrator == first_integrator_before);
    assert(first.output == first_output_before);
    assert(second.integrator == second_integrator_before);
    assert(isnan(second.output));
}

static void test_angle_pll(void)
{
    const control_angle_pll_config_t config = {
        .proportional_gain_per_s = 80.0f,
        .integral_gain_per_s2 = 2000.0f,
        .maximum_speed_rad_s = 100.0f,
        .max_timestep_s = 0.002f
    };
    control_angle_pll_t pll;
    const float true_speed = 10.0f;
    const float timestep_s = 0.001f;
    float true_angle = 0.0f;
    unsigned int iteration;

    assert(control_angle_pll_init(&pll, &config, 0.0f));
    for (iteration = 0U; iteration < 3000U; ++iteration) {
        true_angle += true_speed * timestep_s;
        assert(control_angle_pll_update(
            &pll, control_angle_wrap_pm_pi(true_angle), timestep_s));
    }

    assert(almost_equal(pll.speed_rad_s, true_speed, 0.05f));
    assert(almost_equal(pll.continuous_angle_rad, true_angle, 0.02f));
    assert(fabsf(pll.phase_error_rad) < 0.01f);

    /* Phase estimation remains small-range even after the continuous position
     * has grown beyond the point where float can represent milliradians. */
    pll.continuous_angle_precise_rad = 100000000.0;
    pll.continuous_angle_rad = (float)pll.continuous_angle_precise_rad;
    pll.wrapped_angle_rad = 0.0f;
    pll.integral_speed_rad_s = true_speed;
    pll.speed_rad_s = true_speed;
    true_angle = 0.0f;
    for (iteration = 0U; iteration < 3000U; ++iteration) {
        true_angle += true_speed * timestep_s;
        assert(control_angle_pll_update(
            &pll, control_angle_wrap_pm_pi(true_angle), timestep_s));
    }
    assert(almost_equal(pll.speed_rad_s, true_speed, 0.05f));
    assert(fabs(pll.continuous_angle_precise_rad -
                (100000000.0 + (double)true_angle)) < 0.1);

    {
        control_angle_pll_t overflow_pll = pll;
        overflow_pll.config.integral_gain_per_s2 = FLT_MAX;
        overflow_pll.config.proportional_gain_per_s = FLT_MAX;
        assert(!control_angle_pll_update(&overflow_pll,
                                         CONTROL_PI_F * 0.5f,
                                         timestep_s));
        assert(overflow_pll.continuous_angle_precise_rad ==
               pll.continuous_angle_precise_rad);
    }
}

static void test_pv_and_pp_trajectories(void)
{
    const control_trajectory_limits_t limits = {
        .maximum_velocity_rad_s = 2.0f,
        .maximum_acceleration_rad_s2 = 10.0f,
        .maximum_deceleration_rad_s2 = 10.0f,
        .position_tolerance_rad = 1.0e-4f,
        .velocity_tolerance_rad_s = 1.0e-3f,
        .max_timestep_s = 0.01f
    };
    control_trajectory_state_t state;
    unsigned int iteration;

    assert(control_trajectory_init(&state, 0.0f, 0.0f));
    for (iteration = 0U; iteration < 10U; ++iteration) {
        assert(control_trajectory_step_pv(&state, &limits, 2.0f, 0.01f));
    }
    assert(almost_equal(state.velocity_rad_s, 1.0f, 1.0e-5f));
    assert(state.position_rad > 0.0f);

    assert(control_trajectory_init(&state, 0.0f, 0.0f));
    for (iteration = 0U;
         (iteration < 5000U) && !state.target_reached;
         ++iteration) {
        assert(control_trajectory_step_pp(&state, &limits, 1.0f, 0.001f));
    }
    assert(state.target_reached);
    assert(iteration < 5000U);
    assert(almost_equal(state.position_rad, 1.0f, 1.0e-6f));
    assert(state.velocity_rad_s == 0.0f);

    {
        const control_trajectory_limits_t reversal_limits = {
            .maximum_velocity_rad_s = 5.0f,
            .maximum_acceleration_rad_s2 = 2.0f,
            .maximum_deceleration_rad_s2 = 10.0f,
            .position_tolerance_rad = 1.0e-4f,
            .velocity_tolerance_rad_s = 1.0e-3f,
            .max_timestep_s = 0.2f
        };

        assert(control_trajectory_init(&state, 0.0f, 1.0f));
        assert(control_trajectory_step_pv(&state, &reversal_limits,
                                          -5.0f, 0.15f));
        /* 0.1 s deceleration to zero, then 0.05 s acceleration at 2 rad/s^2. */
        assert(almost_equal(state.velocity_rad_s, -0.1f, 1.0e-6f));
        assert(almost_equal(state.position_rad, 0.0475f, 1.0e-6f));

        assert(control_trajectory_init(&state, 0.0f, 1.0f));
        assert(control_trajectory_step_pv(&state, &reversal_limits,
                                          -0.01f, 0.15f));
        assert(almost_equal(state.velocity_rad_s, -0.01f, 1.0e-6f));
        assert(almost_equal(state.position_rad, 0.049525f, 1.0e-6f));
    }
}

static void test_mit_torque_control(void)
{
    const control_mit_limits_t limits = {
        .position_min_rad = -3.0f,
        .position_max_rad = 3.0f,
        .velocity_limit_rad_s = 20.0f,
        .kp_limit_nm_per_rad = 100.0f,
        .kd_limit_nm_s_per_rad = 5.0f,
        .feedforward_torque_limit_nm = 3.0f,
        .output_torque_limit_nm = 5.0f,
        .iq_limit_a = 8.0f,
        .torque_constant_nm_per_a = 0.5f,
        .command_timeout_us = 1000U
    };
    control_mit_command_t command = {
        .position_rad = 1.0f,
        .velocity_rad_s = 2.0f,
        .kp_nm_per_rad = 10.0f,
        .kd_nm_s_per_rad = 1.0f,
        .feedforward_torque_nm = 0.5f,
        .timestamp_us = 100U
    };
    const control_mit_feedback_t feedback = {
        .position_rad = 0.5f,
        .velocity_rad_s = 1.0f
    };
    control_mit_output_t output;

    assert(control_mit_compute(&limits, &command, &feedback, 200U, &output) ==
           CONTROL_MIT_STATUS_SATURATED);
    assert(almost_equal(output.requested_torque_nm, 6.5f, 1.0e-6f));
    assert(almost_equal(output.iq_command_a, 8.0f, 1.0e-6f));
    assert(almost_equal(output.applied_torque_nm, 4.0f, 1.0e-6f));
    assert((output.limit_flags & CONTROL_MIT_LIMIT_OUTPUT_TORQUE) != 0U);
    assert((output.limit_flags & CONTROL_MIT_LIMIT_IQ) != 0U);

    assert(control_mit_compute(&limits, &command, &feedback, 1200U, &output) ==
           CONTROL_MIT_STATUS_TIMEOUT);
    assert(output.iq_command_a == 0.0f);
    assert(output.applied_torque_nm == 0.0f);

    /* Unsigned subtraction correctly accepts a fresh command across wrap. */
    command.position_rad = feedback.position_rad;
    command.velocity_rad_s = feedback.velocity_rad_s;
    command.feedforward_torque_nm = 1.0f;
    command.timestamp_us = UINT32_MAX - 10U;
    assert(control_mit_compute(&limits, &command, &feedback, 10U, &output) ==
           CONTROL_MIT_STATUS_OK);
    assert(output.command_age_us == 21U);
    assert(almost_equal(output.iq_command_a, 2.0f, 1.0e-6f));

    {
        const control_mit_feedback_t extreme_feedback = {
            .position_rad = -FLT_MAX,
            .velocity_rad_s = 0.0f
        };
        command.position_rad = FLT_MAX;
        command.kp_nm_per_rad = FLT_MAX;
        assert(control_mit_compute(&limits, &command, &extreme_feedback,
                                   10U, &output) ==
               CONTROL_MIT_STATUS_INVALID_ARGUMENT);
        assert(output.requested_torque_nm == 0.0f);
        assert(output.iq_command_a == 0.0f);
    }
}

int main(void)
{
    test_angle_wrapping();
    test_transforms();
    test_voltage_limit_and_svpwm();
    test_extreme_finite_foc_inputs_fail_safe();
    test_pi_anti_windup();
    test_pi_coupled_applied_output_tracking();
    test_angle_pll();
    test_pv_and_pp_trajectories();
    test_mit_torque_control();

    (void)puts("control unit tests passed");
    return 0;
}
