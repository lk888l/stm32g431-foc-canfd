#ifndef APP_MOTOR_LAB_COMMISSIONING_H
#define APP_MOTOR_LAB_COMMISSIONING_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    LAB_COMMISSIONING_COMMAND_STOP = 0,
    LAB_COMMISSIONING_COMMAND_CLEAR_FAULT = 1,
    LAB_COMMISSIONING_COMMAND_CALIBRATE = 2,
    LAB_COMMISSIONING_COMMAND_VOLTAGE = 3,
    LAB_COMMISSIONING_COMMAND_VELOCITY = 4,
    LAB_COMMISSIONING_COMMAND_POSITION = 5,
    LAB_COMMISSIONING_COMMAND_OPEN_LOOP = 6
} lab_commissioning_command_t;

typedef enum {
    LAB_COMMISSIONING_STATE_IDLE = 0,
    LAB_COMMISSIONING_STATE_WAIT_ENCODER,
    LAB_COMMISSIONING_STATE_ALIGN_ZERO,
    LAB_COMMISSIONING_STATE_ALIGN_QUADRATURE,
    LAB_COMMISSIONING_STATE_ALIGN_RETURN,
    LAB_COMMISSIONING_STATE_RUN_VOLTAGE,
    LAB_COMMISSIONING_STATE_RUN_VELOCITY,
    LAB_COMMISSIONING_STATE_RUN_POSITION,
    LAB_COMMISSIONING_STATE_COMPLETE,
    LAB_COMMISSIONING_STATE_FAULT,
    LAB_COMMISSIONING_STATE_RUN_OPEN_LOOP
} lab_commissioning_state_id_t;

typedef enum {
    LAB_COMMISSIONING_RESULT_NONE = 0,
    LAB_COMMISSIONING_RESULT_RUNNING,
    LAB_COMMISSIONING_RESULT_OK,
    LAB_COMMISSIONING_RESULT_INVALID_REQUEST,
    LAB_COMMISSIONING_RESULT_ENCODER_INVALID,
    LAB_COMMISSIONING_RESULT_ALIGNMENT_NO_MOTION,
    LAB_COMMISSIONING_RESULT_SAFETY_FAULT,
    LAB_COMMISSIONING_RESULT_TIMEOUT,
    LAB_COMMISSIONING_RESULT_ALIGNMENT_RETURN_ERROR,
    LAB_COMMISSIONING_RESULT_ALIGNMENT_INCONSISTENT
} lab_commissioning_result_t;

typedef struct {
    float pole_pairs;
    float minimum_alignment_voltage_v;
    float maximum_voltage_v;
    float maximum_velocity_rad_s;
    float maximum_open_loop_voltage_v;
    float maximum_open_loop_electrical_velocity_rad_s;
    float maximum_abs_position_rad;
    uint32_t encoder_wait_timeout_us;
    uint32_t align_zero_time_us;
    uint32_t align_quadrature_time_us;
    uint32_t align_return_time_us;
    float maximum_alignment_repeat_error_rad;
    uint32_t default_run_time_us;
    uint32_t maximum_run_time_us;
    uint32_t maximum_open_loop_run_time_us;
} lab_commissioning_config_t;

typedef struct {
    lab_commissioning_command_t command;
    float value;
    float voltage_limit_v;
    uint32_t duration_ms;
} lab_commissioning_request_t;

typedef struct {
    uint32_t now_us;
    float raw_mechanical_angle_rad;
    float directed_position_rad;
    float mechanical_velocity_rad_s;
    bool encoder_valid;
    bool electrical_calibration_valid;
    bool safety_fault_active;
} lab_commissioning_input_t;

typedef struct {
    lab_commissioning_state_id_t state;
    lab_commissioning_result_t result;
    bool drive_enable;
    bool fixed_stator_voltage;
    bool fault_reset_request;
    bool calibration_update;
    float stator_electrical_angle_rad;
    float stator_voltage_v;
    float q_voltage_v;
    float velocity_target_rad_s;
    float position_target_rad;
    float voltage_limit_v;
    float encoder_direction;
    float electrical_zero_rad;
    float alignment_delta_rad;
} lab_commissioning_output_t;

typedef struct {
    lab_commissioning_config_t config;
    lab_commissioning_request_t request;
    lab_commissioning_state_id_t state;
    lab_commissioning_result_t result;
    uint32_t state_started_us;
    uint32_t run_duration_us;
    float zero_raw_angle_rad;
    float encoder_direction;
    float alignment_direction;
    float electrical_zero_rad;
    float alignment_delta_rad;
    float first_alignment_zero_rad;
    float first_alignment_direction;
    uint8_t alignment_pass;
    bool fault_reset_pending;
    bool initialized;
} lab_commissioning_t;

bool lab_commissioning_init(lab_commissioning_t *commissioning,
                            const lab_commissioning_config_t *config);

bool lab_commissioning_submit(lab_commissioning_t *commissioning,
                              const lab_commissioning_request_t *request,
                              const lab_commissioning_input_t *input);

bool lab_commissioning_restore_calibration(
    lab_commissioning_t *commissioning,
    float encoder_direction,
    float electrical_zero_rad);

lab_commissioning_output_t lab_commissioning_step(
    lab_commissioning_t *commissioning,
    const lab_commissioning_input_t *input);

#ifdef __cplusplus
}
#endif

#endif /* APP_MOTOR_LAB_COMMISSIONING_H */
