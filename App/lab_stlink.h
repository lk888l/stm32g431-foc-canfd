#ifndef APP_LAB_STLINK_H
#define APP_LAB_STLINK_H

#include <stdbool.h>
#include <stdint.h>

#include "lab_commissioning.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LAB_STLINK_COMMAND_MAGIC UINT32_C(0x4C414243) /* LABC */
#define LAB_STLINK_STATUS_MAGIC  UINT32_C(0x4C414253) /* LABS */
#define LAB_STLINK_GUARD_SALT    UINT32_C(0xA5C35A3C)
#define LAB_STLINK_CALIBRATION_MAGIC UINT32_C(0x4C43414C) /* LCAL */
#define LAB_STLINK_CALIBRATION_SALT  UINT32_C(0x6D13B7E9)
#define LAB_STLINK_TRACE_MAGIC   UINT32_C(0x4C545243) /* LTRC */
#define LAB_STLINK_ABI_VERSION   UINT32_C(4)
#define LAB_STLINK_TRACE_ABI_VERSION UINT32_C(3)
#define LAB_STLINK_TRACE_CAPACITY UINT32_C(384)
#define LAB_STLINK_TRACE_PERIOD_US UINT32_C(3000)

enum {
    LAB_STLINK_FLAG_ENCODER_VALID = UINT32_C(1) << 0,
    LAB_STLINK_FLAG_ELECTRICAL_CALIBRATED = UINT32_C(1) << 1,
    LAB_STLINK_FLAG_GATE_ARMED = UINT32_C(1) << 2,
    LAB_STLINK_FLAG_OUTPUT_ENABLED = UINT32_C(1) << 3,
    LAB_STLINK_FLAG_GATE_FAULT_ACTIVE = UINT32_C(1) << 4,
    LAB_STLINK_FLAG_CURRENT_FEEDBACK_VALID = UINT32_C(1) << 5,
    LAB_STLINK_FLAG_THREE_PWM_PROFILE = UINT32_C(1) << 6,
    LAB_STLINK_FLAG_CURRENT_LOOP_INTERFACE_AVAILABLE = UINT32_C(1) << 7,
    LAB_STLINK_FLAG_CAN_ENABLED = UINT32_C(1) << 8,
    LAB_STLINK_FLAG_WEAK_MAGNET_ACCEPTED = UINT32_C(1) << 9,
    LAB_STLINK_FLAG_ENCODER_BUS_QUIESCED = UINT32_C(1) << 10,
    LAB_STLINK_FLAG_OUTPUT_WAS_ENABLED = UINT32_C(1) << 11,
    LAB_STLINK_FLAG_ENCODER_SCL_HIGH = UINT32_C(1) << 12,
    LAB_STLINK_FLAG_ENCODER_SDA_HIGH = UINT32_C(1) << 13,
    LAB_STLINK_FLAG_ENCODER_DRIVER_INITIALIZED = UINT32_C(1) << 14
};

/* The sequence word is the commit marker and must be written last. The guard
 * covers every payload word, including sequence, so a debugger interruption
 * cannot accidentally start the motor with a torn command. */
typedef struct {
    volatile uint32_t magic;
    volatile uint32_t command;
    volatile uint32_t duration_ms;
    volatile uint32_t value_bits;
    volatile uint32_t voltage_limit_bits;
    volatile uint32_t guard;
    volatile uint32_t sequence;
} lab_stlink_command_mailbox_t;

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t publish_sequence;
    uint32_t applied_command_sequence;
    uint32_t state;
    uint32_t result;
    uint32_t flags;
    uint32_t timestamp_us;
    uint32_t encoder_raw_count;
    uint32_t encoder_health;
    uint32_t active_faults;
    uint32_t latched_faults;
    uint32_t current_loop_count;
    uint32_t supervisor_loop_count;
    float raw_mechanical_angle_rad;
    float directed_position_rad;
    float mechanical_velocity_rad_s;
    float encoder_direction;
    float electrical_zero_rad;
    float alignment_delta_rad;
    float alpha_voltage_v;
    float beta_voltage_v;
    float q_voltage_v;
    float duty_a;
    float duty_b;
    float duty_c;
    float bus_voltage_v;
    uint32_t encoder_status_raw;
    uint32_t encoder_agc;
    uint32_t encoder_magnitude;
    uint32_t encoder_minimum_agc;
    uint32_t encoder_maximum_agc;
    uint32_t encoder_minimum_magnitude;
    uint32_t encoder_maximum_magnitude;
    uint32_t encoder_angle_read_count;
    uint32_t encoder_diagnostic_read_count;
    uint32_t encoder_bus_error_count;
    uint32_t encoder_bus_recovery_count;
    uint32_t encoder_bus_failure_count;
    uint32_t encoder_diagnostic_failure_count;
    uint32_t encoder_age_us;
    uint32_t encoder_maximum_age_us;
    uint32_t encoder_angle_glitch_count;
    uint32_t encoder_angle_glitch_recovery_count;
} lab_stlink_status_t;

/* Compact nominal 333 Hz diagnostic trace. The 384-sample ring retains the
 * final roughly 1.5 s of a long tuning command while keeping enough
 * resolution for the low-bandwidth speed loop. The buffer is dumped only
 * after the bridge has been disabled.
 *
 * state_result: state[15:0], result[31:16]
 * encoder_sample: raw[15:0], health[23:16], valid[31]
 * encoder_diagnostics: STATUS[7:0], AGC[15:8], MAGNITUDE[27:16], valid[31]
 */
typedef struct {
    uint32_t timestamp_us;
    uint32_t state_result;
    uint32_t encoder_sample;
    uint32_t encoder_diagnostics;
    float position_rad;
    float velocity_rad_s;
    float reference;
    float voltage_v;
    uint32_t encoder_age_us;
    uint32_t encoder_angle_read_count;
    uint32_t encoder_bus_error_count;
    uint32_t encoder_bus_failure_count;
} lab_stlink_trace_sample_t;

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t publish_sequence;
    uint32_t command_sequence;
    uint32_t sample_count;
    uint32_t capacity;
    uint32_t sample_period_us;
    uint32_t final_state;
    uint32_t final_result;
    uint32_t oldest_sample_index;
    lab_stlink_trace_sample_t samples[LAB_STLINK_TRACE_CAPACITY];
} lab_stlink_trace_t;

extern lab_stlink_command_mailbox_t g_lab_stlink_command;
extern volatile lab_stlink_status_t g_lab_stlink_status;
extern volatile lab_stlink_trace_t g_lab_stlink_trace;

uint32_t lab_stlink_command_guard(uint32_t command,
                                  uint32_t duration_ms,
                                  uint32_t value_bits,
                                  uint32_t voltage_limit_bits,
                                  uint32_t sequence);

bool lab_stlink_take_request(uint32_t *last_sequence,
                             lab_commissioning_request_t *request,
                             uint32_t *accepted_sequence);

bool lab_stlink_consume_request(uint32_t sequence);

void lab_stlink_initialize_retention(bool allow_software_reset_retention);
void lab_stlink_clear_calibration(void);
bool lab_stlink_store_calibration(float encoder_direction,
                                  float electrical_zero_rad);
bool lab_stlink_load_calibration(float *encoder_direction,
                                 float *electrical_zero_rad);

void lab_stlink_publish_status(const lab_stlink_status_t *status);
void lab_stlink_trace_reset(uint32_t command_sequence);
void lab_stlink_trace_append(const lab_stlink_trace_sample_t *sample);
void lab_stlink_trace_finish(uint32_t state, uint32_t result);

#ifdef __cplusplus
}
#endif

#endif /* APP_LAB_STLINK_H */
