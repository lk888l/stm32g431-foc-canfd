#ifndef APP_DRIVE_DRIVE_MODE_H
#define APP_DRIVE_DRIVE_MODE_H

#include <stdbool.h>
#include <stdint.h>

#include "drive_pds.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Values 1 and 3 are profile modes. 0x80 is deliberately manufacturer-specific. */
typedef enum {
    DRIVE_MODE_NONE = 0,
    DRIVE_MODE_PROFILE_POSITION = 1,
    DRIVE_MODE_PROFILE_VELOCITY = 3,
    DRIVE_MODE_MIT = -128
} drive_mode_of_operation_t;

typedef struct {
    int32_t minimum_position_q16_16;
    int32_t maximum_position_q16_16;
    int32_t maximum_abs_velocity_q16_16;
} drive_mode_config_t;

typedef struct {
    int8_t requested_mode;
    uint32_t mode_instance_epoch;
    uint16_t controlword;
    /* PP fields below are the snapshot captured at the 0x6040 bit-4 rising
     * write, not a live view of 0x607A. */
    uint16_t pp_setpoint_controlword;
    int8_t pp_setpoint_mode;
    uint32_t pp_setpoint_mode_instance_epoch;
    int32_t actual_position_q16_16;
    int32_t target_position_q16_16;
    uint32_t target_position_epoch;
    uint32_t pp_request_epoch;
    int32_t target_velocity_q16_16;
    uint32_t target_velocity_epoch;
    int8_t target_velocity_mode;
    uint32_t target_velocity_mode_instance_epoch;
    /* Mode writes/transitions are blocked for the full Operation Enabled
     * state and whenever another state is still producing controlled torque. */
    bool mode_change_inhibited;
    /* Actual published PDS torque permission, used by MIT execution. */
    bool torque_active;
    bool current_feedback_valid;
    bool trajectory_target_reached;
    bool trajectory_internal_limit;
} drive_mode_input_t;

typedef struct {
    int8_t mode_display;
    uint16_t statusword_bits;
    int32_t position_reference_q16_16;
    int32_t velocity_reference_q16_16;
    bool operation_enable_allowed;
    bool halt_active;
    bool pp_setpoint_acknowledge;
    bool pp_new_target;
    bool pp_setpoint_rejected;
    bool pv_target_updated;
    bool mode_change_rejected;
    bool mit_control_enabled;
} drive_mode_output_t;

typedef struct {
    drive_mode_config_t config;
    int8_t active_mode;
    uint32_t active_mode_instance_epoch;
    int32_t pp_active_target_q16_16;
    int32_t pp_pending_target_q16_16;
    int32_t pp_last_command_target_q16_16;
    bool pp_has_active_target;
    bool pp_has_pending_target;
    bool pp_has_last_command_target;
    bool pp_setpoint_acknowledge;
    /** Completion is accepted only after low has been observed for this target. */
    bool pp_target_reached_low_seen;
    bool pp_target_completed;
    uint32_t pp_consumed_request_epoch;
    uint32_t pp_consumed_position_epoch;
    int32_t pv_active_target_q16_16;
    uint32_t pv_consumed_velocity_epoch;
} drive_mode_manager_t;

bool drive_mode_init(drive_mode_manager_t *manager,
                     const drive_mode_config_t *config);

/*
 * PP accepts a set-point request only when its captured target-position epoch
 * has not already been consumed. Relative commands are based on the last
 * accepted command (or actual position before the first command). One pending
 * point is retained without dynamic memory. PV latches each new velocity
 * epoch once; re-entering PV requires a post-entry refresh.
 */
drive_mode_output_t drive_mode_step(drive_mode_manager_t *manager,
                                    const drive_mode_input_t *input);

bool drive_mode_is_supported(int8_t mode);

#ifdef __cplusplus
}
#endif

#endif /* APP_DRIVE_DRIVE_MODE_H */
