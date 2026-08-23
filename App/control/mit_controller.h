#ifndef APP_CONTROL_MIT_CONTROLLER_H
#define APP_CONTROL_MIT_CONTROLLER_H

/**
 * @file mit_controller.h
 * @brief MIT-style impedance command converted to a real torque/current demand.
 *
 * This implementation does not command voltage. It calculates
 *   torque = kp*(q_des-q) + kd*(v_des-v) + torque_feed_forward
 * and then maps the strictly limited torque to q-axis current through the
 * configured effective torque constant. A calibrated current controller must
 * consume iq_command_a for this to be torque control in the physical sense.
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float position_min_rad;
    float position_max_rad;
    float velocity_limit_rad_s;
    float kp_limit_nm_per_rad;
    float kd_limit_nm_s_per_rad;
    float feedforward_torque_limit_nm;
    float output_torque_limit_nm;
    float iq_limit_a;
    /** Effective torque constant at the controlled shaft, N*m/A. */
    float torque_constant_nm_per_a;
    /** 32-bit monotonic timestamp timeout; must be below 2^31 us. */
    uint32_t command_timeout_us;
} control_mit_limits_t;

typedef struct {
    float position_rad;
    float velocity_rad_s;
    float kp_nm_per_rad;
    float kd_nm_s_per_rad;
    float feedforward_torque_nm;
    uint32_t timestamp_us;
} control_mit_command_t;

typedef struct {
    float position_rad;
    float velocity_rad_s;
} control_mit_feedback_t;

typedef enum {
    CONTROL_MIT_STATUS_OK = 0,
    CONTROL_MIT_STATUS_SATURATED,
    CONTROL_MIT_STATUS_TIMEOUT,
    CONTROL_MIT_STATUS_INVALID_ARGUMENT
} control_mit_status_t;

enum {
    CONTROL_MIT_LIMIT_NONE = 0U,
    CONTROL_MIT_LIMIT_POSITION = (1U << 0),
    CONTROL_MIT_LIMIT_VELOCITY = (1U << 1),
    CONTROL_MIT_LIMIT_KP = (1U << 2),
    CONTROL_MIT_LIMIT_KD = (1U << 3),
    CONTROL_MIT_LIMIT_FEEDFORWARD_TORQUE = (1U << 4),
    CONTROL_MIT_LIMIT_OUTPUT_TORQUE = (1U << 5),
    CONTROL_MIT_LIMIT_IQ = (1U << 6)
};

typedef struct {
    control_mit_status_t status;
    uint32_t limit_flags;
    uint32_t command_age_us;
    /** Torque before the final output-torque and iq limits. */
    float requested_torque_nm;
    /** Torque represented by the final q-axis current request. */
    float applied_torque_nm;
    float iq_command_a;
} control_mit_output_t;

bool control_mit_limits_are_valid(const control_mit_limits_t *limits);

/**
 * Evaluate an MIT command at monotonic time now_us.
 *
 * Timestamp subtraction intentionally uses unsigned modulo arithmetic, so a
 * single 32-bit timer wrap is handled correctly. On timeout or invalid input,
 * iq_command_a and both torque fields are exactly zero.
 */
control_mit_status_t control_mit_compute(
    const control_mit_limits_t *limits,
    const control_mit_command_t *command,
    const control_mit_feedback_t *feedback,
    uint32_t now_us,
    control_mit_output_t *output);

#ifdef __cplusplus
}
#endif

#endif /* APP_CONTROL_MIT_CONTROLLER_H */
