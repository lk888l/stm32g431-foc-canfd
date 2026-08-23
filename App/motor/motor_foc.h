#ifndef BRUSHLESS_MOTOR_FOC_H
#define BRUSHLESS_MOTOR_FOC_H

#include <stdbool.h>
#include <stdint.h>

#include "angle_pll.h"
#include "control_pi.h"

typedef enum {
    MOTOR_FOC_CURRENT_CLOSED_LOOP = 0,
    MOTOR_FOC_LAB_VOLTAGE_LOOP
} motor_foc_strategy_t;

typedef struct {
    motor_foc_strategy_t strategy;
    float control_period_s;
    float pole_pairs;
    float encoder_direction;
    float electrical_zero_rad;
    bool electrical_zero_valid;
    float phase_a_current_a_per_count;
    float phase_b_current_a_per_count;
    uint16_t adc_min_valid_count;
    uint16_t adc_max_valid_count;
    uint16_t offset_min_valid_count;
    uint16_t offset_max_valid_count;
    uint16_t offset_max_span_count;
    uint16_t offset_calibration_samples;
    float phase_resistance_ohm;
    float d_inductance_h;
    float q_inductance_h;
    float flux_linkage_wb;
    float current_limit_a;
    float bus_nominal_v;
    float voltage_utilization;
    float minimum_pulse_s;
    uint32_t encoder_stale_timeout_us;
    control_pi_config_t d_current_pi;
    control_pi_config_t q_current_pi;
    control_angle_pll_config_t angle_pll;
} motor_foc_config_t;

typedef struct {
    bool enable;
    float id_reference_a;
    float iq_reference_a;
    float lab_q_voltage_v;
} motor_foc_command_t;

typedef struct {
    float mechanical_position_rad;
    float mechanical_velocity_rad_s;
    float electrical_angle_rad;
    float phase_a_current_a;
    float phase_b_current_a;
    float phase_c_current_a;
    float id_current_a;
    float iq_current_a;
    float id_voltage_v;
    float iq_voltage_v;
    float bus_voltage_v;
    float duty_a;
    float duty_b;
    float duty_c;
    uint32_t encoder_health;
    uint32_t encoder_age_us;
    bool encoder_valid;
    bool current_feedback_valid;
    bool bus_voltage_valid;
    bool calibration_complete;
    bool current_limit_exceeded;
    bool control_math_fault;
    bool output_enabled;
} motor_foc_feedback_t;

typedef struct {
    motor_foc_config_t config;
    control_pi_t d_current_pi;
    control_pi_t q_current_pi;
    control_angle_pll_t angle_pll;
    motor_foc_command_t command;
    motor_foc_feedback_t feedback;
    uint64_t phase_a_offset_accumulator;
    uint64_t phase_b_offset_accumulator;
    float phase_a_offset_count;
    float phase_b_offset_count;
    uint32_t offset_samples_collected;
    uint16_t phase_a_offset_min_count;
    uint16_t phase_a_offset_max_count;
    uint16_t phase_b_offset_min_count;
    uint16_t phase_b_offset_max_count;
    uint32_t last_encoder_timestamp_us;
    bool initialized;
} motor_foc_t;

bool motor_foc_init(motor_foc_t *motor, const motor_foc_config_t *config);
void motor_foc_set_command(motor_foc_t *motor,
                           const motor_foc_command_t *command);
void motor_foc_set_bus_voltage(motor_foc_t *motor,
                               float bus_voltage_v,
                               bool valid);
bool motor_foc_update_encoder(motor_foc_t *motor,
                              float wrapped_mechanical_angle_rad,
                              uint32_t timestamp_us,
                              uint32_t health_flags,
                              bool valid);
void motor_foc_fast_step(motor_foc_t *motor,
                         uint16_t phase_a_raw,
                         uint16_t phase_b_raw,
                         uint32_t now_us);
const motor_foc_feedback_t *motor_foc_feedback(const motor_foc_t *motor);

#endif
