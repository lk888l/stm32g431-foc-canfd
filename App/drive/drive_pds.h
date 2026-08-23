#ifndef APP_DRIVE_DRIVE_PDS_H
#define APP_DRIVE_DRIVE_PDS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Controlword bits used by the portable PDS state machine. */
enum {
    DRIVE_PDS_CW_SWITCH_ON = UINT16_C(1) << 0,
    DRIVE_PDS_CW_ENABLE_VOLTAGE = UINT16_C(1) << 1,
    DRIVE_PDS_CW_QUICK_STOP = UINT16_C(1) << 2,
    DRIVE_PDS_CW_ENABLE_OPERATION = UINT16_C(1) << 3,
    DRIVE_PDS_CW_NEW_SET_POINT = UINT16_C(1) << 4,
    DRIVE_PDS_CW_CHANGE_IMMEDIATELY = UINT16_C(1) << 5,
    DRIVE_PDS_CW_RELATIVE = UINT16_C(1) << 6,
    DRIVE_PDS_CW_FAULT_RESET = UINT16_C(1) << 7,
    DRIVE_PDS_CW_HALT = UINT16_C(1) << 8
};

/* Statusword bits. Bits 10, 12 and 13 are supplied by the mode manager. */
enum {
    DRIVE_PDS_SW_READY_TO_SWITCH_ON = UINT16_C(1) << 0,
    DRIVE_PDS_SW_SWITCHED_ON = UINT16_C(1) << 1,
    DRIVE_PDS_SW_OPERATION_ENABLED = UINT16_C(1) << 2,
    DRIVE_PDS_SW_FAULT = UINT16_C(1) << 3,
    DRIVE_PDS_SW_VOLTAGE_ENABLED = UINT16_C(1) << 4,
    DRIVE_PDS_SW_QUICK_STOP = UINT16_C(1) << 5,
    DRIVE_PDS_SW_SWITCH_ON_DISABLED = UINT16_C(1) << 6,
    DRIVE_PDS_SW_WARNING = UINT16_C(1) << 7,
    DRIVE_PDS_SW_REMOTE = UINT16_C(1) << 9,
    DRIVE_PDS_SW_TARGET_REACHED = UINT16_C(1) << 10,
    DRIVE_PDS_SW_INTERNAL_LIMIT = UINT16_C(1) << 11,
    DRIVE_PDS_SW_MODE_SPECIFIC_1 = UINT16_C(1) << 12,
    DRIVE_PDS_SW_MODE_SPECIFIC_2 = UINT16_C(1) << 13
};

#define DRIVE_PDS_MODE_STATUS_MASK                                                \
    ((uint16_t)(DRIVE_PDS_SW_TARGET_REACHED | DRIVE_PDS_SW_INTERNAL_LIMIT |       \
                DRIVE_PDS_SW_MODE_SPECIFIC_1 | DRIVE_PDS_SW_MODE_SPECIFIC_2))

typedef enum {
    DRIVE_PDS_NOT_READY_TO_SWITCH_ON = 0,
    DRIVE_PDS_SWITCH_ON_DISABLED,
    DRIVE_PDS_READY_TO_SWITCH_ON,
    DRIVE_PDS_SWITCHED_ON,
    DRIVE_PDS_OPERATION_ENABLED,
    DRIVE_PDS_QUICK_STOP_ACTIVE,
    DRIVE_PDS_FAULT_REACTION_ACTIVE,
    DRIVE_PDS_FAULT
} drive_pds_state_t;

typedef enum {
    /* Finish the controlled stop, then remove gate drive and enter disabled. */
    DRIVE_PDS_QS_DISABLE_VOLTAGE = 0,
    /* Keep the power stage energized after the stop; a shutdown command exits. */
    DRIVE_PDS_QS_HOLD_POWER,
    /* Permit an explicit enable-operation command to resume after stop complete. */
    DRIVE_PDS_QS_ALLOW_REENABLE
} drive_pds_quick_stop_policy_t;

typedef struct {
    drive_pds_quick_stop_policy_t quick_stop_policy;
} drive_pds_config_t;

typedef struct {
    uint16_t controlword;
    uint16_t mode_status_bits;
    bool hardware_ready;
    bool external_fault_active;
    bool fault_reaction_complete;
    bool fault_reaction_requires_immediate_gate_off;
    bool quick_stop_complete;
    bool gate_enable_allowed;
    bool gate_feedback_enabled;
    bool operation_enable_allowed;
    bool warning;
    bool remote;
} drive_pds_input_t;

typedef struct {
    drive_pds_state_t state;
    uint16_t statusword;
    bool gate_enable_request;
    bool torque_enable;
    bool quick_stop_request;
    bool fault_reset_rising_edge;
    bool gate_interlock_fault;
} drive_pds_output_t;

typedef struct {
    drive_pds_config_t config;
    drive_pds_state_t state;
    drive_pds_state_t power_down_target;
    uint16_t previous_controlword;
    bool power_down_pending;
    bool gate_enable_request;
    bool torque_enable;
    bool quick_stop_request;
    /** True only when quick stop was entered from operation-enabled torque. */
    bool quick_stop_torque_allowed;
    bool gate_was_confirmed;
    bool gate_interlock_latched;
    bool fault_reaction_was_powered;
    bool fault_reaction_torque_allowed;
} drive_pds_t;

void drive_pds_init(drive_pds_t *pds, const drive_pds_config_t *config);

/*
 * Execute one supervisory tick. The gate feedback input must represent the
 * physical gate-driver enable state, not the last software command.
 */
drive_pds_output_t drive_pds_step(drive_pds_t *pds,
                                  const drive_pds_input_t *input);

uint16_t drive_pds_statusword_for_state(drive_pds_state_t state);
bool drive_pds_controlword_is_shutdown(uint16_t controlword);
bool drive_pds_controlword_is_switch_on(uint16_t controlword);
bool drive_pds_controlword_is_enable_operation(uint16_t controlword);

#ifdef __cplusplus
}
#endif

#endif /* APP_DRIVE_DRIVE_PDS_H */
