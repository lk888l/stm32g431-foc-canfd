#include "drive_safety.h"

#include <limits.h>
#include <stddef.h>

static uint16_t minimum_u16(uint16_t a, uint16_t b)
{
    return (a < b) ? a : b;
}

drive_safety_config_t drive_safety_default_config(void)
{
    drive_safety_config_t config;

    config.latching_mask =
        DRIVE_FAULT_PHASE_OVERCURRENT | DRIVE_FAULT_DC_OVERVOLTAGE |
        DRIVE_FAULT_POWER_STAGE_OVERTEMP | DRIVE_FAULT_MOTOR_OVERTEMP |
        DRIVE_FAULT_GATE_DRIVER | DRIVE_FAULT_CURRENT_FEEDBACK |
        DRIVE_FAULT_POSITION_FEEDBACK | DRIVE_FAULT_OVERSPEED |
        DRIVE_FAULT_CONTROL_DEADLINE | DRIVE_FAULT_GATE_INTERLOCK |
        DRIVE_FAULT_CAN_BUS_OFF | DRIVE_FAULT_COMMAND_HARD_TIMEOUT |
        DRIVE_FAULT_PROTOCOL | DRIVE_FAULT_STO_ACTIVE;
    config.immediate_gate_off_mask =
        DRIVE_FAULT_PHASE_OVERCURRENT | DRIVE_FAULT_DC_OVERVOLTAGE |
        DRIVE_FAULT_GATE_DRIVER | DRIVE_FAULT_CURRENT_FEEDBACK |
        DRIVE_FAULT_POSITION_FEEDBACK | DRIVE_FAULT_CONTROL_DEADLINE |
        DRIVE_FAULT_GATE_INTERLOCK |
        DRIVE_FAULT_COMMAND_HARD_TIMEOUT | DRIVE_FAULT_STO_ACTIVE;
    /* Controlled-stop faults must retain gate drive long enough to brake. */
    config.gate_inhibit_mask = config.immediate_gate_off_mask;
    config.quick_stop_mask =
        DRIVE_FAULT_DC_UNDERVOLTAGE | DRIVE_FAULT_POWER_STAGE_OVERTEMP |
        DRIVE_FAULT_MOTOR_OVERTEMP | DRIVE_FAULT_POSITION_FEEDBACK |
        DRIVE_FAULT_OVERSPEED | DRIVE_FAULT_CAN_BUS_OFF |
        DRIVE_FAULT_COMMAND_SOFT_TIMEOUT | DRIVE_FAULT_PROTOCOL;
    config.warning_mask = DRIVE_FAULT_DC_UNDERVOLTAGE |
                          DRIVE_FAULT_POWER_STAGE_OVERTEMP |
                          DRIVE_FAULT_MOTOR_OVERTEMP |
                          DRIVE_FAULT_COMMAND_SOFT_TIMEOUT;
    config.command_soft_timeout_us = UINT32_C(100000);
    config.command_hard_timeout_us = UINT32_C(500000);
    config.soft_timeout_torque_limit_q15 = UINT16_C(8192);
    config.fault_derate_torque_limit_q15 = UINT16_C(16384);
    return config;
}

bool drive_safety_init(drive_safety_t *safety,
                       const drive_safety_config_t *config)
{
    if ((safety == NULL) || (config == NULL) ||
        (config->command_soft_timeout_us == 0U) ||
        (config->command_hard_timeout_us <=
         config->command_soft_timeout_us) ||
        (config->command_hard_timeout_us > (uint32_t)INT32_MAX) ||
        (config->soft_timeout_torque_limit_q15 >
         DRIVE_SAFETY_FULL_TORQUE_Q15) ||
        (config->fault_derate_torque_limit_q15 >
         DRIVE_SAFETY_FULL_TORQUE_Q15) ||
        (((config->latching_mask | config->gate_inhibit_mask |
           config->immediate_gate_off_mask | config->quick_stop_mask |
           config->warning_mask) &
          ~DRIVE_FAULT_ALL_DEFINED_MASK) != 0U)) {
        return false;
    }

    safety->config = *config;
    safety->latched_faults = 0U;
    safety->last_valid_command_us = 0U;
    safety->command_watchdog_armed = false;
    return true;
}

void drive_safety_note_valid_command(drive_safety_t *safety, uint32_t now_us)
{
    if (safety == NULL) {
        return;
    }
    safety->last_valid_command_us = now_us;
    safety->command_watchdog_armed = true;
}

void drive_safety_disarm_command_watchdog(drive_safety_t *safety)
{
    if (safety != NULL) {
        safety->command_watchdog_armed = false;
    }
}

drive_safety_output_t drive_safety_step(drive_safety_t *safety,
                                        const drive_safety_input_t *input)
{
    drive_safety_output_t output = {0};
    drive_fault_bits_t active;
    uint16_t torque_limit;

    if ((safety == NULL) || (input == NULL)) {
        output.immediate_gate_off = true;
        output.gate_enable_allowed = false;
        return output;
    }

    active = input->sensed_faults & DRIVE_FAULT_ALL_DEFINED_MASK;
    if (input->command_watchdog_required) {
        /* Requiring the watchdog is itself an arming event. This prevents an
         * integration error from leaving monitoring disabled forever before
         * the first valid network command arrives. */
        if (!safety->command_watchdog_armed) {
            safety->last_valid_command_us = input->now_us;
            safety->command_watchdog_armed = true;
        }
        output.command_age_us =
            (uint32_t)(input->now_us - safety->last_valid_command_us);
        if (output.command_age_us >=
            safety->config.command_soft_timeout_us) {
            active |= DRIVE_FAULT_COMMAND_SOFT_TIMEOUT;
            output.command_soft_timeout = true;
        }
        if (output.command_age_us >=
            safety->config.command_hard_timeout_us) {
            active |= DRIVE_FAULT_COMMAND_HARD_TIMEOUT;
            output.command_hard_timeout = true;
        }
    } else {
        safety->command_watchdog_armed = false;
    }

    /* A reset clears only inactive causes and only after hardware says it is safe. */
    if (input->reset_rising_edge && input->safe_to_reset) {
        safety->latched_faults &= active;
    }
    safety->latched_faults |= active & safety->config.latching_mask;

    output.active_faults = active;
    output.latched_faults = safety->latched_faults;
    output.effective_faults = active | safety->latched_faults;
    output.warning =
        (output.effective_faults & safety->config.warning_mask) != 0U;
    output.immediate_gate_off =
        (output.effective_faults &
         safety->config.immediate_gate_off_mask) != 0U;
    output.quick_stop_request =
        (output.effective_faults & safety->config.quick_stop_mask) != 0U;
    output.controlled_stop_request =
        output.quick_stop_request && !output.immediate_gate_off;
    output.gate_enable_allowed =
        !output.immediate_gate_off &&
        ((output.effective_faults & safety->config.gate_inhibit_mask) == 0U);

    torque_limit = minimum_u16(input->thermal_torque_limit_q15,
                               input->supply_torque_limit_q15);
    torque_limit = minimum_u16(torque_limit,
                               DRIVE_SAFETY_FULL_TORQUE_Q15);
    if (output.command_soft_timeout) {
        torque_limit = minimum_u16(
            torque_limit, safety->config.soft_timeout_torque_limit_q15);
    }
    if ((output.effective_faults & safety->config.quick_stop_mask) != 0U) {
        torque_limit = minimum_u16(
            torque_limit, safety->config.fault_derate_torque_limit_q15);
    }
    if (output.immediate_gate_off) {
        torque_limit = 0U;
    }
    output.torque_limit_q15 = torque_limit;
    return output;
}
