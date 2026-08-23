#ifndef APP_SAFETY_DRIVE_SAFETY_H
#define APP_SAFETY_DRIVE_SAFETY_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef uint32_t drive_fault_bits_t;

enum {
    DRIVE_FAULT_PHASE_OVERCURRENT = UINT32_C(1) << 0,
    DRIVE_FAULT_DC_OVERVOLTAGE = UINT32_C(1) << 1,
    DRIVE_FAULT_DC_UNDERVOLTAGE = UINT32_C(1) << 2,
    DRIVE_FAULT_POWER_STAGE_OVERTEMP = UINT32_C(1) << 3,
    DRIVE_FAULT_MOTOR_OVERTEMP = UINT32_C(1) << 4,
    DRIVE_FAULT_GATE_DRIVER = UINT32_C(1) << 5,
    DRIVE_FAULT_CURRENT_FEEDBACK = UINT32_C(1) << 6,
    DRIVE_FAULT_POSITION_FEEDBACK = UINT32_C(1) << 7,
    DRIVE_FAULT_OVERSPEED = UINT32_C(1) << 8,
    DRIVE_FAULT_CONTROL_DEADLINE = UINT32_C(1) << 9,
    DRIVE_FAULT_GATE_INTERLOCK = UINT32_C(1) << 10,
    DRIVE_FAULT_CAN_BUS_OFF = UINT32_C(1) << 11,
    DRIVE_FAULT_COMMAND_SOFT_TIMEOUT = UINT32_C(1) << 12,
    DRIVE_FAULT_COMMAND_HARD_TIMEOUT = UINT32_C(1) << 13,
    DRIVE_FAULT_PROTOCOL = UINT32_C(1) << 14,
    DRIVE_FAULT_STO_ACTIVE = UINT32_C(1) << 15
};

#define DRIVE_FAULT_ALL_DEFINED_MASK UINT32_C(0x0000FFFF)
#define DRIVE_SAFETY_FULL_TORQUE_Q15 UINT16_C(32767)

typedef struct {
    drive_fault_bits_t latching_mask;
    drive_fault_bits_t gate_inhibit_mask;
    drive_fault_bits_t immediate_gate_off_mask;
    drive_fault_bits_t quick_stop_mask;
    drive_fault_bits_t warning_mask;
    uint32_t command_soft_timeout_us;
    uint32_t command_hard_timeout_us;
    uint16_t soft_timeout_torque_limit_q15;
    uint16_t fault_derate_torque_limit_q15;
} drive_safety_config_t;

typedef struct {
    uint32_t now_us;
    drive_fault_bits_t sensed_faults;
    uint16_t thermal_torque_limit_q15;
    uint16_t supply_torque_limit_q15;
    bool command_watchdog_required;
    bool reset_rising_edge;
    bool safe_to_reset;
} drive_safety_input_t;

typedef struct {
    drive_fault_bits_t active_faults;
    drive_fault_bits_t latched_faults;
    drive_fault_bits_t effective_faults;
    uint32_t command_age_us;
    uint16_t torque_limit_q15;
    bool warning;
    bool command_soft_timeout;
    bool command_hard_timeout;
    bool controlled_stop_request;
    bool quick_stop_request;
    bool immediate_gate_off;
    bool gate_enable_allowed;
} drive_safety_output_t;

typedef struct {
    drive_safety_config_t config;
    drive_fault_bits_t latched_faults;
    uint32_t last_valid_command_us;
    bool command_watchdog_armed;
} drive_safety_t;

bool drive_safety_init(drive_safety_t *safety,
                       const drive_safety_config_t *config);

/* Arm/reload the watchdog only after all protocol and range checks pass. */
void drive_safety_note_valid_command(drive_safety_t *safety, uint32_t now_us);
void drive_safety_disarm_command_watchdog(drive_safety_t *safety);

/* Unsigned subtraction makes time comparisons safe across a 32-bit wrap. */
drive_safety_output_t drive_safety_step(drive_safety_t *safety,
                                        const drive_safety_input_t *input);

drive_safety_config_t drive_safety_default_config(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_SAFETY_DRIVE_SAFETY_H */
