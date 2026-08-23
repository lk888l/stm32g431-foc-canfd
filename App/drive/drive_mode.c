#include "drive_mode.h"

#include <limits.h>
#include <stddef.h>

bool drive_mode_is_supported(int8_t mode)
{
    return (mode == (int8_t)DRIVE_MODE_PROFILE_POSITION) ||
           (mode == (int8_t)DRIVE_MODE_PROFILE_VELOCITY) ||
           (mode == (int8_t)DRIVE_MODE_MIT);
}

bool drive_mode_init(drive_mode_manager_t *manager,
                     const drive_mode_config_t *config)
{
    if ((manager == NULL) || (config == NULL) ||
        (config->minimum_position_q16_16 >
         config->maximum_position_q16_16) ||
        (config->maximum_abs_velocity_q16_16 < 0)) {
        return false;
    }

    manager->config = *config;
    manager->active_mode = (int8_t)DRIVE_MODE_NONE;
    manager->active_mode_instance_epoch = 0U;
    manager->pp_active_target_q16_16 = 0;
    manager->pp_pending_target_q16_16 = 0;
    manager->pp_last_command_target_q16_16 = 0;
    manager->pp_has_active_target = false;
    manager->pp_has_pending_target = false;
    manager->pp_has_last_command_target = false;
    manager->pp_setpoint_acknowledge = false;
    manager->pp_target_reached_low_seen = false;
    manager->pp_target_completed = false;
    manager->pp_consumed_request_epoch = 0U;
    manager->pp_consumed_position_epoch = 0U;
    manager->pv_active_target_q16_16 = 0;
    manager->pv_consumed_velocity_epoch = 0U;
    return true;
}

static void reset_command_handshakes(drive_mode_manager_t *manager,
                                     const drive_mode_input_t *input)
{
    (void)input;
    manager->pp_has_active_target = false;
    manager->pp_has_pending_target = false;
    manager->pp_has_last_command_target = false;
    manager->pp_setpoint_acknowledge = false;
    manager->pp_target_reached_low_seen = false;
    manager->pp_target_completed = false;
    /* Clear active motion, but retain consumed generations.  A command
     * explicitly tagged with the newly requested mode may have been written
     * by SDO immediately before this supervisor step and remains legitimate. */
    manager->pv_active_target_q16_16 = 0;
}

static bool compute_pp_target(const drive_mode_manager_t *manager,
                              const drive_mode_input_t *input,
                              int32_t *target)
{
    int64_t candidate = input->target_position_q16_16;

    if ((input->pp_setpoint_controlword & DRIVE_PDS_CW_RELATIVE) != 0U) {
        const int32_t base = manager->pp_has_last_command_target
                                 ? manager->pp_last_command_target_q16_16
                                 : input->actual_position_q16_16;
        candidate += (int64_t)base;
    }
    if ((candidate < (int64_t)manager->config.minimum_position_q16_16) ||
        (candidate > (int64_t)manager->config.maximum_position_q16_16) ||
        (candidate < INT32_MIN) || (candidate > INT32_MAX)) {
        return false;
    }
    *target = (int32_t)candidate;
    return true;
}

static void promote_pending_if_ready(drive_mode_manager_t *manager,
                                     const drive_mode_input_t *input,
                                     drive_mode_output_t *output)
{
    if (!manager->pp_has_active_target) {
        return;
    }
    if (!input->trajectory_target_reached) {
        manager->pp_target_reached_low_seen = true;
        return;
    }
    if (!manager->pp_target_reached_low_seen) {
        return;
    }

    manager->pp_target_reached_low_seen = false;

    if (manager->pp_has_pending_target) {
        manager->pp_active_target_q16_16 =
            manager->pp_pending_target_q16_16;
        manager->pp_has_pending_target = false;
        manager->pp_target_completed = false;
        output->pp_new_target = true;
    } else {
        manager->pp_has_active_target = false;
        manager->pp_target_completed = true;
    }
}

static void handle_pp_setpoint(drive_mode_manager_t *manager,
                               const drive_mode_input_t *input,
                               drive_mode_output_t *output)
{
    const bool new_setpoint =
        (input->controlword & DRIVE_PDS_CW_NEW_SET_POINT) != 0U;
    const bool request_pending =
        input->pp_request_epoch != manager->pp_consumed_request_epoch;

    promote_pending_if_ready(manager, input, output);

    if (!new_setpoint) {
        manager->pp_setpoint_acknowledge = false;
    }
    if (request_pending) {
        int32_t candidate;
        bool accepted;
        const bool change_immediately =
            (input->pp_setpoint_controlword &
             DRIVE_PDS_CW_CHANGE_IMMEDIATELY) != 0U;

        /* Consume both the edge and, if present, its captured target before
         * any queue/range decision.  A rejected relative command can never be
         * replayed later against a different base position. */
        manager->pp_consumed_request_epoch = input->pp_request_epoch;
        accepted = input->target_position_epoch !=
                   manager->pp_consumed_position_epoch;
        if (accepted) {
            manager->pp_consumed_position_epoch =
                input->target_position_epoch;
            accepted = input->pp_setpoint_mode ==
                       (int8_t)DRIVE_MODE_PROFILE_POSITION;
            accepted = accepted &&
                       (input->pp_setpoint_mode_instance_epoch ==
                        manager->active_mode_instance_epoch);
        }
        if (accepted) {
            accepted = compute_pp_target(manager, input, &candidate);
        }

        if (accepted) {
            if (change_immediately || !manager->pp_has_active_target) {
                manager->pp_active_target_q16_16 = candidate;
                manager->pp_has_active_target = true;
                manager->pp_has_pending_target = false;
                manager->pp_target_reached_low_seen =
                    !input->trajectory_target_reached;
                manager->pp_target_completed = false;
                output->pp_new_target = true;
            } else if (!manager->pp_has_pending_target) {
                manager->pp_pending_target_q16_16 = candidate;
                manager->pp_has_pending_target = true;
            } else {
                accepted = false;
            }
        }

        if (accepted) {
            manager->pp_last_command_target_q16_16 = candidate;
            manager->pp_has_last_command_target = true;
            manager->pp_setpoint_acknowledge = new_setpoint;
        } else {
            manager->pp_setpoint_acknowledge = false;
            output->pp_setpoint_rejected = true;
        }
    }

    output->pp_setpoint_acknowledge =
        manager->pp_setpoint_acknowledge;
    output->position_reference_q16_16 =
        manager->pp_has_active_target
            ? manager->pp_active_target_q16_16
            : input->actual_position_q16_16;
    if (manager->pp_setpoint_acknowledge) {
        output->statusword_bits |= DRIVE_PDS_SW_MODE_SPECIFIC_1;
    }
    if (manager->pp_target_completed &&
        !manager->pp_has_active_target &&
        !manager->pp_has_pending_target &&
        input->trajectory_target_reached) {
        output->statusword_bits |= DRIVE_PDS_SW_TARGET_REACHED;
    }
}

drive_mode_output_t drive_mode_step(drive_mode_manager_t *manager,
                                    const drive_mode_input_t *input)
{
    drive_mode_output_t output = {0};

    if ((manager == NULL) || (input == NULL)) {
        return output;
    }

    if ((input->requested_mode != manager->active_mode) ||
        (input->mode_instance_epoch !=
         manager->active_mode_instance_epoch)) {
        if (!drive_mode_is_supported(input->requested_mode)) {
            output.mode_change_rejected = true;
        } else if (input->mode_change_inhibited) {
            /* Mode-instance changes are forbidden while any torque-producing
             * state is active, including a controlled Quick Stop. */
            output.mode_change_rejected = true;
        } else {
            manager->active_mode = input->requested_mode;
            manager->active_mode_instance_epoch =
                input->mode_instance_epoch;
            reset_command_handshakes(manager, input);
        }
    }

    output.mode_display = manager->active_mode;
    output.halt_active =
        (input->controlword & DRIVE_PDS_CW_HALT) != 0U;
    output.operation_enable_allowed =
        drive_mode_is_supported(manager->active_mode);

    switch (manager->active_mode) {
    case DRIVE_MODE_PROFILE_POSITION:
        handle_pp_setpoint(manager, input, &output);
        break;

    case DRIVE_MODE_PROFILE_VELOCITY:
        if (input->target_velocity_epoch !=
            manager->pv_consumed_velocity_epoch) {
            manager->pv_consumed_velocity_epoch =
                input->target_velocity_epoch;
            if (input->target_velocity_mode ==
                    (int8_t)DRIVE_MODE_PROFILE_VELOCITY &&
                (input->target_velocity_mode_instance_epoch ==
                 manager->active_mode_instance_epoch)) {
                manager->pv_active_target_q16_16 =
                    input->target_velocity_q16_16;
                output.pv_target_updated = true;
            }
        }
        if ((manager->pv_active_target_q16_16 >
             manager->config.maximum_abs_velocity_q16_16) ||
            (manager->pv_active_target_q16_16 <
             -manager->config.maximum_abs_velocity_q16_16)) {
            output.statusword_bits |= DRIVE_PDS_SW_INTERNAL_LIMIT;
            output.velocity_reference_q16_16 =
                (manager->pv_active_target_q16_16 < 0)
                    ? -manager->config.maximum_abs_velocity_q16_16
                    : manager->config.maximum_abs_velocity_q16_16;
        } else {
            output.velocity_reference_q16_16 =
                manager->pv_active_target_q16_16;
        }
        break;

    case DRIVE_MODE_MIT:
        /* MIT torque output is impossible until phase-current feedback is valid. */
        output.operation_enable_allowed = input->current_feedback_valid;
        output.mit_control_enabled = input->torque_active &&
                                     input->current_feedback_valid;
        break;

    case DRIVE_MODE_NONE:
    default:
        output.operation_enable_allowed = false;
        break;
    }

    if (input->trajectory_internal_limit) {
        output.statusword_bits |= DRIVE_PDS_SW_INTERNAL_LIMIT;
    }
    return output;
}
