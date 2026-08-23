#include "mit_controller.h"

#include "foc_math.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>

static bool control_mit_float_is_finite(float value)
{
    return isfinite(value) != 0;
}

static void control_mit_clear_output(control_mit_output_t *output,
                                     control_mit_status_t status)
{
    if (output != NULL) {
        output->status = status;
        output->limit_flags = CONTROL_MIT_LIMIT_NONE;
        output->command_age_us = 0U;
        output->requested_torque_nm = 0.0f;
        output->applied_torque_nm = 0.0f;
        output->iq_command_a = 0.0f;
    }
}

static float control_mit_clamp_with_flag(float value,
                                         float lower,
                                         float upper,
                                         uint32_t flag,
                                         uint32_t *flags)
{
    if (value < lower) {
        *flags |= flag;
        return lower;
    }
    if (value > upper) {
        *flags |= flag;
        return upper;
    }
    return value;
}

bool control_mit_limits_are_valid(const control_mit_limits_t *limits)
{
    if (limits == NULL) {
        return false;
    }

    return control_mit_float_is_finite(limits->position_min_rad) &&
           control_mit_float_is_finite(limits->position_max_rad) &&
           control_mit_float_is_finite(limits->velocity_limit_rad_s) &&
           control_mit_float_is_finite(limits->kp_limit_nm_per_rad) &&
           control_mit_float_is_finite(limits->kd_limit_nm_s_per_rad) &&
           control_mit_float_is_finite(
               limits->feedforward_torque_limit_nm) &&
           control_mit_float_is_finite(limits->output_torque_limit_nm) &&
           control_mit_float_is_finite(limits->iq_limit_a) &&
           control_mit_float_is_finite(limits->torque_constant_nm_per_a) &&
           (limits->position_min_rad < limits->position_max_rad) &&
           (limits->velocity_limit_rad_s > 0.0f) &&
           (limits->kp_limit_nm_per_rad >= 0.0f) &&
           (limits->kd_limit_nm_s_per_rad >= 0.0f) &&
           (limits->feedforward_torque_limit_nm >= 0.0f) &&
           (limits->output_torque_limit_nm > 0.0f) &&
           (limits->iq_limit_a > 0.0f) &&
           (limits->torque_constant_nm_per_a > 0.0f) &&
           (limits->command_timeout_us > 0U) &&
           (limits->command_timeout_us <= (uint32_t)INT32_MAX);
}

control_mit_status_t control_mit_compute(
    const control_mit_limits_t *limits,
    const control_mit_command_t *command,
    const control_mit_feedback_t *feedback,
    uint32_t now_us,
    control_mit_output_t *output)
{
    uint32_t flags = CONTROL_MIT_LIMIT_NONE;
    uint32_t command_age;
    float desired_position;
    float desired_velocity;
    float kp;
    float kd;
    float feedforward_torque;
    float requested_torque;
    float torque_limited;
    float iq_command;

    if (output == NULL) {
        return CONTROL_MIT_STATUS_INVALID_ARGUMENT;
    }
    control_mit_clear_output(output, CONTROL_MIT_STATUS_INVALID_ARGUMENT);

    if (!control_mit_limits_are_valid(limits) || (command == NULL) ||
        (feedback == NULL)) {
        return output->status;
    }

    if (!control_mit_float_is_finite(command->position_rad) ||
        !control_mit_float_is_finite(command->velocity_rad_s) ||
        !control_mit_float_is_finite(command->kp_nm_per_rad) ||
        !control_mit_float_is_finite(command->kd_nm_s_per_rad) ||
        !control_mit_float_is_finite(command->feedforward_torque_nm) ||
        !control_mit_float_is_finite(feedback->position_rad) ||
        !control_mit_float_is_finite(feedback->velocity_rad_s)) {
        return output->status;
    }

    command_age = now_us - command->timestamp_us;
    output->command_age_us = command_age;
    if (command_age > limits->command_timeout_us) {
        output->status = CONTROL_MIT_STATUS_TIMEOUT;
        return output->status;
    }

    desired_position = control_mit_clamp_with_flag(
        command->position_rad, limits->position_min_rad,
        limits->position_max_rad, CONTROL_MIT_LIMIT_POSITION, &flags);
    desired_velocity = control_mit_clamp_with_flag(
        command->velocity_rad_s, -limits->velocity_limit_rad_s,
        limits->velocity_limit_rad_s, CONTROL_MIT_LIMIT_VELOCITY, &flags);
    kp = control_mit_clamp_with_flag(
        command->kp_nm_per_rad, 0.0f, limits->kp_limit_nm_per_rad,
        CONTROL_MIT_LIMIT_KP, &flags);
    kd = control_mit_clamp_with_flag(
        command->kd_nm_s_per_rad, 0.0f, limits->kd_limit_nm_s_per_rad,
        CONTROL_MIT_LIMIT_KD, &flags);
    feedforward_torque = control_mit_clamp_with_flag(
        command->feedforward_torque_nm,
        -limits->feedforward_torque_limit_nm,
        limits->feedforward_torque_limit_nm,
        CONTROL_MIT_LIMIT_FEEDFORWARD_TORQUE, &flags);

    requested_torque = (kp * (desired_position - feedback->position_rad)) +
                       (kd * (desired_velocity - feedback->velocity_rad_s)) +
                       feedforward_torque;
    if (!control_mit_float_is_finite(requested_torque)) {
        return output->status;
    }

    torque_limited = control_mit_clamp_with_flag(
        requested_torque, -limits->output_torque_limit_nm,
        limits->output_torque_limit_nm, CONTROL_MIT_LIMIT_OUTPUT_TORQUE,
        &flags);
    iq_command = torque_limited / limits->torque_constant_nm_per_a;
    iq_command = control_mit_clamp_with_flag(
        iq_command, -limits->iq_limit_a, limits->iq_limit_a,
        CONTROL_MIT_LIMIT_IQ, &flags);

    output->requested_torque_nm = requested_torque;
    output->iq_command_a = iq_command;
    output->applied_torque_nm = iq_command * limits->torque_constant_nm_per_a;
    output->limit_flags = flags;
    output->status = (flags == CONTROL_MIT_LIMIT_NONE) ?
                     CONTROL_MIT_STATUS_OK : CONTROL_MIT_STATUS_SATURATED;
    return output->status;
}
