#include "lab_commissioning.h"

#include <math.h>
#include <stddef.h>

#include "foc_math.h"

#define LAB_US_PER_MS UINT32_C(1000)
#define LAB_ALIGNMENT_MIN_MOTION_RATIO 0.55f
#define LAB_ALIGNMENT_MAX_MOTION_RATIO 1.30f

static bool finite_positive(float value)
{
    return isfinite(value) && (value > 0.0f);
}

static bool state_drives(lab_commissioning_state_id_t state)
{
    return (state == LAB_COMMISSIONING_STATE_ALIGN_ZERO) ||
           (state == LAB_COMMISSIONING_STATE_ALIGN_QUADRATURE) ||
           (state == LAB_COMMISSIONING_STATE_ALIGN_RETURN) ||
           (state == LAB_COMMISSIONING_STATE_RUN_VOLTAGE) ||
           (state == LAB_COMMISSIONING_STATE_RUN_VELOCITY) ||
           (state == LAB_COMMISSIONING_STATE_RUN_POSITION) ||
           (state == LAB_COMMISSIONING_STATE_RUN_OPEN_LOOP);
}

static uint32_t elapsed_us(uint32_t now_us, uint32_t started_us)
{
    return now_us - started_us;
}

static float alignment_ramp_fraction(uint32_t now_us,
                                     uint32_t started_us,
                                     uint32_t state_time_us)
{
    const float state_fraction =
        (float)elapsed_us(now_us, started_us) / (float)state_time_us;

    /* Use the first 60% for motion and the remaining 40% for settling. */
    return control_clampf(state_fraction / 0.60f, 0.0f, 1.0f);
}

static void enter_state(lab_commissioning_t *commissioning,
                        lab_commissioning_state_id_t state,
                        lab_commissioning_result_t result,
                        uint32_t now_us)
{
    commissioning->state = state;
    commissioning->result = result;
    commissioning->state_started_us = now_us;
}

static uint32_t requested_run_time_us(
    const lab_commissioning_t *commissioning,
    uint32_t duration_ms)
{
    uint64_t requested;

    if (duration_ms == 0U) {
        return commissioning->config.default_run_time_us;
    }
    requested = (uint64_t)duration_ms * LAB_US_PER_MS;
    if (requested > commissioning->config.maximum_run_time_us) {
        return commissioning->config.maximum_run_time_us;
    }
    return (uint32_t)requested;
}

bool lab_commissioning_init(lab_commissioning_t *commissioning,
                            const lab_commissioning_config_t *config)
{
    if ((commissioning == NULL) || (config == NULL) ||
        !finite_positive(config->pole_pairs) ||
        !finite_positive(config->minimum_alignment_voltage_v) ||
        !finite_positive(config->maximum_voltage_v) ||
        (config->minimum_alignment_voltage_v > config->maximum_voltage_v) ||
        !finite_positive(config->maximum_velocity_rad_s) ||
        !finite_positive(config->maximum_open_loop_voltage_v) ||
        (config->maximum_open_loop_voltage_v <
         config->minimum_alignment_voltage_v) ||
        (config->maximum_open_loop_voltage_v > config->maximum_voltage_v) ||
        !finite_positive(
            config->maximum_open_loop_electrical_velocity_rad_s) ||
        !finite_positive(config->maximum_abs_position_rad) ||
        (config->encoder_wait_timeout_us == 0U) ||
        (config->align_zero_time_us == 0U) ||
        (config->align_quadrature_time_us == 0U) ||
        (config->align_return_time_us == 0U) ||
        !finite_positive(config->maximum_alignment_repeat_error_rad) ||
        (config->maximum_alignment_repeat_error_rad > CONTROL_PI_F) ||
        (config->default_run_time_us == 0U) ||
        (config->maximum_run_time_us < config->default_run_time_us) ||
        (config->maximum_open_loop_run_time_us == 0U) ||
        (config->maximum_open_loop_run_time_us >
         config->maximum_run_time_us)) {
        return false;
    }

    *commissioning = (lab_commissioning_t){0};
    commissioning->config = *config;
    commissioning->state = LAB_COMMISSIONING_STATE_IDLE;
    commissioning->result = LAB_COMMISSIONING_RESULT_NONE;
    commissioning->encoder_direction = 1.0f;
    commissioning->alignment_direction = 1.0f;
    commissioning->initialized = true;
    return true;
}

bool lab_commissioning_submit(lab_commissioning_t *commissioning,
                              const lab_commissioning_request_t *request,
                              const lab_commissioning_input_t *input)
{
    float voltage_limit;

    if ((commissioning == NULL) || (request == NULL) || (input == NULL) ||
        !commissioning->initialized) {
        return false;
    }

    if (request->command == LAB_COMMISSIONING_COMMAND_STOP) {
        commissioning->request = *request;
        enter_state(commissioning,
                    LAB_COMMISSIONING_STATE_IDLE,
                    LAB_COMMISSIONING_RESULT_NONE,
                    input->now_us);
        return true;
    }
    if (request->command == LAB_COMMISSIONING_COMMAND_CLEAR_FAULT) {
        commissioning->request = *request;
        commissioning->fault_reset_pending = true;
        enter_state(commissioning,
                    LAB_COMMISSIONING_STATE_IDLE,
                    LAB_COMMISSIONING_RESULT_NONE,
                    input->now_us);
        return true;
    }
    if (!isfinite(request->value) || !isfinite(request->voltage_limit_v)) {
        enter_state(commissioning,
                    LAB_COMMISSIONING_STATE_FAULT,
                    LAB_COMMISSIONING_RESULT_INVALID_REQUEST,
                    input->now_us);
        return false;
    }

    voltage_limit = request->voltage_limit_v;
    if (voltage_limit <= 0.0f) {
        voltage_limit = commissioning->config.maximum_voltage_v;
    }
    voltage_limit = control_clampf(
        voltage_limit,
        commissioning->config.minimum_alignment_voltage_v,
        commissioning->config.maximum_voltage_v);

    commissioning->request = *request;
    commissioning->request.voltage_limit_v = voltage_limit;
    commissioning->run_duration_us = requested_run_time_us(
        commissioning, request->duration_ms);

    switch (request->command) {
    case LAB_COMMISSIONING_COMMAND_CALIBRATE:
        commissioning->alignment_direction =
            commissioning->encoder_direction;
        commissioning->alignment_pass = 0U;
        commissioning->first_alignment_zero_rad = 0.0f;
        commissioning->first_alignment_direction = 0.0f;
        enter_state(commissioning,
                    LAB_COMMISSIONING_STATE_WAIT_ENCODER,
                    LAB_COMMISSIONING_RESULT_RUNNING,
                    input->now_us);
        return true;

    case LAB_COMMISSIONING_COMMAND_VOLTAGE:
        if (!input->electrical_calibration_valid ||
            (fabsf(request->value) > voltage_limit)) {
            break;
        }
        enter_state(commissioning,
                    LAB_COMMISSIONING_STATE_RUN_VOLTAGE,
                    LAB_COMMISSIONING_RESULT_RUNNING,
                    input->now_us);
        return true;

    case LAB_COMMISSIONING_COMMAND_VELOCITY:
        if (!input->electrical_calibration_valid ||
            (fabsf(request->value) >
             commissioning->config.maximum_velocity_rad_s)) {
            break;
        }
        enter_state(commissioning,
                    LAB_COMMISSIONING_STATE_RUN_VELOCITY,
                    LAB_COMMISSIONING_RESULT_RUNNING,
                    input->now_us);
        return true;

    case LAB_COMMISSIONING_COMMAND_POSITION:
        if (!input->electrical_calibration_valid ||
            (fabsf(request->value) >
             commissioning->config.maximum_abs_position_rad)) {
            break;
        }
        enter_state(commissioning,
                    LAB_COMMISSIONING_STATE_RUN_POSITION,
                    LAB_COMMISSIONING_RESULT_RUNNING,
                    input->now_us);
        return true;

    case LAB_COMMISSIONING_COMMAND_OPEN_LOOP:
        if ((voltage_limit >
             commissioning->config.maximum_open_loop_voltage_v) ||
            (fabsf(request->value) >
            commissioning->config
                .maximum_open_loop_electrical_velocity_rad_s)) {
            break;
        }
        if (commissioning->run_duration_us >
            commissioning->config.maximum_open_loop_run_time_us) {
            commissioning->run_duration_us =
                commissioning->config.maximum_open_loop_run_time_us;
        }
        enter_state(commissioning,
                    LAB_COMMISSIONING_STATE_RUN_OPEN_LOOP,
                    LAB_COMMISSIONING_RESULT_RUNNING,
                    input->now_us);
        return true;

    case LAB_COMMISSIONING_COMMAND_STOP:
    case LAB_COMMISSIONING_COMMAND_CLEAR_FAULT:
    default:
        break;
    }

    enter_state(commissioning,
                LAB_COMMISSIONING_STATE_FAULT,
                LAB_COMMISSIONING_RESULT_INVALID_REQUEST,
                input->now_us);
    return false;
}

bool lab_commissioning_restore_calibration(
    lab_commissioning_t *commissioning,
    float encoder_direction,
    float electrical_zero_rad)
{
    if ((commissioning == NULL) || !commissioning->initialized ||
        ((encoder_direction != 1.0f) && (encoder_direction != -1.0f)) ||
        !isfinite(electrical_zero_rad) ||
        (fabsf(electrical_zero_rad) > CONTROL_PI_F)) {
        return false;
    }
    commissioning->encoder_direction = encoder_direction;
    commissioning->alignment_direction = encoder_direction;
    commissioning->electrical_zero_rad = electrical_zero_rad;
    return true;
}

static void step_alignment(lab_commissioning_t *commissioning,
                           const lab_commissioning_input_t *input,
                           lab_commissioning_output_t *output)
{
    const float expected_quadrature_motion =
        (0.5f * CONTROL_PI_F) / commissioning->config.pole_pairs;

    switch (commissioning->state) {
    case LAB_COMMISSIONING_STATE_WAIT_ENCODER:
        if (input->encoder_valid) {
            commissioning->zero_raw_angle_rad =
                input->raw_mechanical_angle_rad;
            enter_state(commissioning,
                        LAB_COMMISSIONING_STATE_ALIGN_ZERO,
                        LAB_COMMISSIONING_RESULT_RUNNING,
                        input->now_us);
        } else if (elapsed_us(input->now_us,
                              commissioning->state_started_us) >=
                   commissioning->config.encoder_wait_timeout_us) {
            enter_state(commissioning,
                        LAB_COMMISSIONING_STATE_FAULT,
                        LAB_COMMISSIONING_RESULT_ENCODER_INVALID,
                        input->now_us);
        }
        break;

    case LAB_COMMISSIONING_STATE_ALIGN_ZERO:
        output->drive_enable = true;
        output->fixed_stator_voltage = true;
        output->stator_voltage_v =
            commissioning->request.voltage_limit_v;
        output->stator_electrical_angle_rad = 0.0f;
        if (elapsed_us(input->now_us,
                       commissioning->state_started_us) >=
            commissioning->config.align_zero_time_us) {
            commissioning->zero_raw_angle_rad =
                input->raw_mechanical_angle_rad;
            enter_state(commissioning,
                        LAB_COMMISSIONING_STATE_ALIGN_QUADRATURE,
                        LAB_COMMISSIONING_RESULT_RUNNING,
                        input->now_us);
        }
        break;

    case LAB_COMMISSIONING_STATE_ALIGN_QUADRATURE:
        output->drive_enable = true;
        output->fixed_stator_voltage = true;
        output->stator_voltage_v =
            commissioning->request.voltage_limit_v;
        output->stator_electrical_angle_rad =
            0.5f * CONTROL_PI_F * alignment_ramp_fraction(
                input->now_us,
                commissioning->state_started_us,
                commissioning->config.align_quadrature_time_us);
        if (elapsed_us(input->now_us,
                       commissioning->state_started_us) >=
            commissioning->config.align_quadrature_time_us) {
            commissioning->alignment_delta_rad = control_angle_wrap_pm_pi(
                input->raw_mechanical_angle_rad -
                commissioning->zero_raw_angle_rad);
            /* Cogging can hold this small motor short of the commanded
             * quarter electrical turn. Direction is still unambiguous at
             * 55%, while the independent second pass protects the zero. */
            if ((fabsf(commissioning->alignment_delta_rad) <
                 (LAB_ALIGNMENT_MIN_MOTION_RATIO *
                  expected_quadrature_motion)) ||
                (fabsf(commissioning->alignment_delta_rad) >
                 (LAB_ALIGNMENT_MAX_MOTION_RATIO *
                  expected_quadrature_motion))) {
                enter_state(commissioning,
                            LAB_COMMISSIONING_STATE_FAULT,
                            LAB_COMMISSIONING_RESULT_ALIGNMENT_NO_MOTION,
                            input->now_us);
            } else {
                commissioning->alignment_direction =
                    (commissioning->alignment_delta_rad > 0.0f)
                        ? 1.0f
                        : -1.0f;
                enter_state(commissioning,
                            LAB_COMMISSIONING_STATE_ALIGN_RETURN,
                            LAB_COMMISSIONING_RESULT_RUNNING,
                            input->now_us);
            }
        }
        break;

    case LAB_COMMISSIONING_STATE_ALIGN_RETURN:
        output->drive_enable = true;
        output->fixed_stator_voltage = true;
        output->stator_voltage_v =
            commissioning->request.voltage_limit_v;
        output->stator_electrical_angle_rad =
            0.5f * CONTROL_PI_F *
            (1.0f - alignment_ramp_fraction(
                        input->now_us,
                        commissioning->state_started_us,
                        commissioning->config.align_return_time_us));
        if (elapsed_us(input->now_us,
                       commissioning->state_started_us) >=
            commissioning->config.align_return_time_us) {
            const float return_error = control_angle_wrap_pm_pi(
                input->raw_mechanical_angle_rad -
                commissioning->zero_raw_angle_rad);
            /* Average the aligned angle before and after the excursion. This
             * cancels first-order static-friction/cogging hysteresis instead
             * of making the electrical zero depend on one return endpoint. */
            const float aligned_raw_angle_rad = control_angle_wrap_pm_pi(
                commissioning->zero_raw_angle_rad +
                (0.5f * return_error));
            const float candidate_zero_rad = control_angle_wrap_pm_pi(
                -commissioning->config.pole_pairs *
                commissioning->alignment_direction *
                aligned_raw_angle_rad);

            if ((fabsf(return_error) >
                 (0.5f * expected_quadrature_motion)) ||
                (fabsf(input->mechanical_velocity_rad_s) > 0.5f)) {
                enter_state(
                    commissioning,
                    LAB_COMMISSIONING_STATE_FAULT,
                    LAB_COMMISSIONING_RESULT_ALIGNMENT_RETURN_ERROR,
                    input->now_us);
            } else if (commissioning->alignment_pass == 0U) {
                commissioning->first_alignment_zero_rad =
                    candidate_zero_rad;
                commissioning->first_alignment_direction =
                    commissioning->alignment_direction;
                commissioning->alignment_pass = 1U;
                enter_state(commissioning,
                            LAB_COMMISSIONING_STATE_ALIGN_ZERO,
                            LAB_COMMISSIONING_RESULT_RUNNING,
                            input->now_us);
            } else if ((commissioning->alignment_direction !=
                        commissioning->first_alignment_direction) ||
                       (fabsf(control_angle_wrap_pm_pi(
                            candidate_zero_rad -
                            commissioning->first_alignment_zero_rad)) >
                        commissioning->config
                            .maximum_alignment_repeat_error_rad)) {
                enter_state(
                    commissioning,
                    LAB_COMMISSIONING_STATE_FAULT,
                    LAB_COMMISSIONING_RESULT_ALIGNMENT_INCONSISTENT,
                    input->now_us);
            } else {
                commissioning->encoder_direction =
                    commissioning->alignment_direction;
                commissioning->electrical_zero_rad =
                    control_angle_wrap_pm_pi(
                        commissioning->first_alignment_zero_rad +
                        (0.5f * control_angle_wrap_pm_pi(
                            candidate_zero_rad -
                            commissioning->first_alignment_zero_rad)));
                output->calibration_update = true;
                output->encoder_direction =
                    commissioning->encoder_direction;
                output->electrical_zero_rad =
                    commissioning->electrical_zero_rad;
                enter_state(commissioning,
                            LAB_COMMISSIONING_STATE_COMPLETE,
                            LAB_COMMISSIONING_RESULT_OK,
                            input->now_us);
            }
            output->drive_enable = false;
            output->fixed_stator_voltage = false;
            output->stator_voltage_v = 0.0f;
        }
        break;

    default:
        break;
    }
}

lab_commissioning_output_t lab_commissioning_step(
    lab_commissioning_t *commissioning,
    const lab_commissioning_input_t *input)
{
    lab_commissioning_output_t output = {0};

    if ((commissioning == NULL) || (input == NULL) ||
        !commissioning->initialized) {
        output.state = LAB_COMMISSIONING_STATE_FAULT;
        output.result = LAB_COMMISSIONING_RESULT_INVALID_REQUEST;
        return output;
    }

    if (commissioning->fault_reset_pending) {
        output.fault_reset_request = true;
        commissioning->fault_reset_pending = false;
    }

    if (state_drives(commissioning->state) && input->safety_fault_active) {
        enter_state(commissioning,
                    LAB_COMMISSIONING_STATE_FAULT,
                    LAB_COMMISSIONING_RESULT_SAFETY_FAULT,
                    input->now_us);
    }
    if (state_drives(commissioning->state) &&
        (commissioning->state != LAB_COMMISSIONING_STATE_RUN_OPEN_LOOP) &&
        !input->encoder_valid) {
        enter_state(commissioning,
                    LAB_COMMISSIONING_STATE_FAULT,
                    LAB_COMMISSIONING_RESULT_ENCODER_INVALID,
                    input->now_us);
    }

    if ((commissioning->state >= LAB_COMMISSIONING_STATE_WAIT_ENCODER) &&
        (commissioning->state <= LAB_COMMISSIONING_STATE_ALIGN_RETURN)) {
        step_alignment(commissioning, input, &output);
    } else if ((commissioning->state ==
                LAB_COMMISSIONING_STATE_RUN_VOLTAGE) ||
               (commissioning->state ==
               LAB_COMMISSIONING_STATE_RUN_VELOCITY) ||
               (commissioning->state ==
                LAB_COMMISSIONING_STATE_RUN_POSITION) ||
               (commissioning->state ==
                LAB_COMMISSIONING_STATE_RUN_OPEN_LOOP)) {
        if (elapsed_us(input->now_us, commissioning->state_started_us) >=
            commissioning->run_duration_us) {
            enter_state(commissioning,
                        LAB_COMMISSIONING_STATE_COMPLETE,
                        LAB_COMMISSIONING_RESULT_OK,
                        input->now_us);
        } else {
            output.drive_enable = true;
            output.voltage_limit_v =
                commissioning->request.voltage_limit_v;
            if (commissioning->state ==
                LAB_COMMISSIONING_STATE_RUN_VOLTAGE) {
                output.q_voltage_v = commissioning->request.value;
            } else if (commissioning->state ==
                       LAB_COMMISSIONING_STATE_RUN_VELOCITY) {
                output.velocity_target_rad_s =
                    commissioning->request.value;
            } else if (commissioning->state ==
                       LAB_COMMISSIONING_STATE_RUN_POSITION) {
                output.position_target_rad =
                    commissioning->request.value;
            } else {
                const float elapsed_s =
                    (float)elapsed_us(input->now_us,
                                      commissioning->state_started_us) *
                    1.0e-6f;

                output.fixed_stator_voltage = true;
                output.stator_voltage_v =
                    commissioning->request.voltage_limit_v;
                output.stator_electrical_angle_rad =
                    control_angle_wrap_pm_pi(
                        commissioning->request.value * elapsed_s);
            }
        }
    }

    output.state = commissioning->state;
    output.result = commissioning->result;
    output.encoder_direction = commissioning->encoder_direction;
    output.electrical_zero_rad = commissioning->electrical_zero_rad;
    output.alignment_delta_rad = commissioning->alignment_delta_rad;
    return output;
}
