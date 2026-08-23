#include "drive_pds.h"

#include <stddef.h>

#define DRIVE_PDS_NO_POWER_DOWN_TARGET DRIVE_PDS_NOT_READY_TO_SWITCH_ON

static bool controlword_disable_voltage(uint16_t controlword)
{
    return (controlword & DRIVE_PDS_CW_ENABLE_VOLTAGE) == 0U;
}

static bool controlword_quick_stop(uint16_t controlword)
{
    return ((controlword & DRIVE_PDS_CW_ENABLE_VOLTAGE) != 0U) &&
           ((controlword & DRIVE_PDS_CW_QUICK_STOP) == 0U);
}

static bool controlword_disable_operation(uint16_t controlword)
{
    const uint16_t mask = (uint16_t)(DRIVE_PDS_CW_SWITCH_ON |
                                     DRIVE_PDS_CW_ENABLE_VOLTAGE |
                                     DRIVE_PDS_CW_QUICK_STOP |
                                     DRIVE_PDS_CW_ENABLE_OPERATION);
    const uint16_t value = (uint16_t)(DRIVE_PDS_CW_SWITCH_ON |
                                      DRIVE_PDS_CW_ENABLE_VOLTAGE |
                                      DRIVE_PDS_CW_QUICK_STOP);
    return (controlword & mask) == value;
}

bool drive_pds_controlword_is_shutdown(uint16_t controlword)
{
    const uint16_t mask = (uint16_t)(DRIVE_PDS_CW_SWITCH_ON |
                                     DRIVE_PDS_CW_ENABLE_VOLTAGE |
                                     DRIVE_PDS_CW_QUICK_STOP);
    const uint16_t value =
        (uint16_t)(DRIVE_PDS_CW_ENABLE_VOLTAGE | DRIVE_PDS_CW_QUICK_STOP);
    return (controlword & mask) == value;
}

bool drive_pds_controlword_is_switch_on(uint16_t controlword)
{
    const uint16_t mask = (uint16_t)(DRIVE_PDS_CW_SWITCH_ON |
                                     DRIVE_PDS_CW_ENABLE_VOLTAGE |
                                     DRIVE_PDS_CW_QUICK_STOP |
                                     DRIVE_PDS_CW_ENABLE_OPERATION);
    const uint16_t value = (uint16_t)(DRIVE_PDS_CW_SWITCH_ON |
                                      DRIVE_PDS_CW_ENABLE_VOLTAGE |
                                      DRIVE_PDS_CW_QUICK_STOP);
    return (controlword & mask) == value;
}

bool drive_pds_controlword_is_enable_operation(uint16_t controlword)
{
    const uint16_t mask = (uint16_t)(DRIVE_PDS_CW_SWITCH_ON |
                                     DRIVE_PDS_CW_ENABLE_VOLTAGE |
                                     DRIVE_PDS_CW_QUICK_STOP |
                                     DRIVE_PDS_CW_ENABLE_OPERATION);
    return (controlword & mask) == mask;
}

uint16_t drive_pds_statusword_for_state(drive_pds_state_t state)
{
    switch (state) {
    case DRIVE_PDS_NOT_READY_TO_SWITCH_ON:
        return UINT16_C(0x0000);
    case DRIVE_PDS_SWITCH_ON_DISABLED:
        return UINT16_C(0x0040);
    case DRIVE_PDS_READY_TO_SWITCH_ON:
        return UINT16_C(0x0021);
    case DRIVE_PDS_SWITCHED_ON:
        return UINT16_C(0x0033);
    case DRIVE_PDS_OPERATION_ENABLED:
        return UINT16_C(0x0037);
    case DRIVE_PDS_QUICK_STOP_ACTIVE:
        return UINT16_C(0x0017);
    case DRIVE_PDS_FAULT_REACTION_ACTIVE:
        return UINT16_C(0x000F);
    case DRIVE_PDS_FAULT:
        return UINT16_C(0x0008);
    default:
        return UINT16_C(0x0000);
    }
}

static void begin_power_down(drive_pds_t *pds, drive_pds_state_t target)
{
    pds->power_down_pending = true;
    pds->power_down_target = target;
    pds->gate_enable_request = false;
    pds->torque_enable = false;
    pds->quick_stop_request = false;
    pds->quick_stop_torque_allowed = false;
    pds->fault_reaction_torque_allowed = false;
}

static void enter_fault_reaction(drive_pds_t *pds, bool immediate_gate_off)
{
    pds->fault_reaction_torque_allowed = pds->torque_enable;
    pds->fault_reaction_was_powered =
        pds->gate_enable_request || pds->gate_was_confirmed;
    pds->state = DRIVE_PDS_FAULT_REACTION_ACTIVE;
    pds->power_down_pending = false;
    pds->torque_enable = false;
    pds->quick_stop_torque_allowed = false;
    pds->quick_stop_request = !immediate_gate_off &&
                              pds->fault_reaction_was_powered;
    if (immediate_gate_off || !pds->fault_reaction_was_powered) {
        pds->gate_enable_request = false;
    }
}

static bool state_expects_confirmed_gate(const drive_pds_t *pds)
{
    return !pds->power_down_pending && pds->gate_was_confirmed &&
           ((pds->state == DRIVE_PDS_SWITCHED_ON) ||
            (pds->state == DRIVE_PDS_OPERATION_ENABLED) ||
            (pds->state == DRIVE_PDS_QUICK_STOP_ACTIVE));
}

static bool state_is_stably_off(const drive_pds_t *pds)
{
    return !pds->power_down_pending && !pds->gate_enable_request &&
           ((pds->state == DRIVE_PDS_NOT_READY_TO_SWITCH_ON) ||
            (pds->state == DRIVE_PDS_SWITCH_ON_DISABLED) ||
            (pds->state == DRIVE_PDS_READY_TO_SWITCH_ON) ||
            (pds->state == DRIVE_PDS_FAULT));
}

void drive_pds_init(drive_pds_t *pds, const drive_pds_config_t *config)
{
    if (pds == NULL) {
        return;
    }

    pds->config.quick_stop_policy = DRIVE_PDS_QS_DISABLE_VOLTAGE;
    if (config != NULL) {
        pds->config = *config;
        if (pds->config.quick_stop_policy > DRIVE_PDS_QS_ALLOW_REENABLE) {
            pds->config.quick_stop_policy = DRIVE_PDS_QS_DISABLE_VOLTAGE;
        }
    }
    pds->state = DRIVE_PDS_NOT_READY_TO_SWITCH_ON;
    pds->power_down_target = DRIVE_PDS_NO_POWER_DOWN_TARGET;
    pds->previous_controlword = 0U;
    pds->power_down_pending = false;
    pds->gate_enable_request = false;
    pds->torque_enable = false;
    pds->quick_stop_request = false;
    pds->quick_stop_torque_allowed = false;
    pds->gate_was_confirmed = false;
    pds->gate_interlock_latched = false;
    pds->fault_reaction_was_powered = false;
    pds->fault_reaction_torque_allowed = false;
}

static drive_pds_output_t make_output(const drive_pds_t *pds,
                                      const drive_pds_input_t *input,
                                      bool reset_edge)
{
    drive_pds_output_t output;
    uint16_t statusword = drive_pds_statusword_for_state(pds->state);

    /* Statusword bit 4 reports physical voltage-stage feedback rather than a
     * state-machine intention. This also exposes a welded/stuck enable path. */
    statusword &= (uint16_t)~DRIVE_PDS_SW_VOLTAGE_ENABLED;
    if (input->gate_feedback_enabled) {
        statusword |= DRIVE_PDS_SW_VOLTAGE_ENABLED;
    }

    statusword |= (uint16_t)(input->mode_status_bits &
                             DRIVE_PDS_MODE_STATUS_MASK);
    if (input->warning) {
        statusword |= DRIVE_PDS_SW_WARNING;
    }
    if (input->remote) {
        statusword |= DRIVE_PDS_SW_REMOTE;
    }

    output.state = pds->state;
    output.statusword = statusword;
    output.gate_enable_request =
        pds->gate_enable_request && input->gate_enable_allowed;
    output.torque_enable = pds->torque_enable &&
                           output.gate_enable_request &&
                           input->gate_feedback_enabled &&
                           input->operation_enable_allowed;
    output.quick_stop_request = pds->quick_stop_request;
    output.fault_reset_rising_edge = reset_edge;
    output.gate_interlock_fault = pds->gate_interlock_latched;
    return output;
}

drive_pds_output_t drive_pds_step(drive_pds_t *pds,
                                  const drive_pds_input_t *input)
{
    bool reset_edge;

    if ((pds == NULL) || (input == NULL)) {
        drive_pds_output_t invalid = {0};
        invalid.state = DRIVE_PDS_NOT_READY_TO_SWITCH_ON;
        return invalid;
    }

    reset_edge = ((input->controlword & DRIVE_PDS_CW_FAULT_RESET) != 0U) &&
                 ((pds->previous_controlword & DRIVE_PDS_CW_FAULT_RESET) == 0U);
    pds->previous_controlword = input->controlword;

    /* A confirmed power stage may never silently disappear while energized. */
    if (state_expects_confirmed_gate(pds) &&
        !input->gate_feedback_enabled) {
        pds->gate_interlock_latched = true;
    }
    /* Conversely, an energized stage in a stable off state is also unsafe. */
    if (state_is_stably_off(pds) && input->gate_feedback_enabled) {
        pds->gate_interlock_latched = true;
    }
    if (pds->gate_enable_request && !input->gate_enable_allowed &&
        (pds->state != DRIVE_PDS_FAULT_REACTION_ACTIVE)) {
        pds->gate_interlock_latched = true;
    }

    if (pds->gate_interlock_latched &&
        (pds->state != DRIVE_PDS_FAULT_REACTION_ACTIVE) &&
        (pds->state != DRIVE_PDS_FAULT)) {
        enter_fault_reaction(pds, true);
    } else if (input->external_fault_active &&
               (pds->state != DRIVE_PDS_FAULT_REACTION_ACTIVE) &&
               (pds->state != DRIVE_PDS_FAULT)) {
        enter_fault_reaction(
            pds, input->fault_reaction_requires_immediate_gate_off);
    }

    if (pds->state == DRIVE_PDS_FAULT_REACTION_ACTIVE) {
        if (input->fault_reaction_requires_immediate_gate_off ||
            input->fault_reaction_complete ||
            pds->gate_interlock_latched) {
            pds->gate_enable_request = false;
            pds->torque_enable = false;
            pds->quick_stop_request = false;
        } else if (pds->fault_reaction_was_powered &&
                   input->gate_enable_allowed) {
            pds->gate_enable_request = true;
            pds->torque_enable = pds->fault_reaction_torque_allowed &&
                                 input->gate_feedback_enabled;
            pds->quick_stop_request = true;
        }
        if (input->fault_reaction_complete &&
            !input->gate_feedback_enabled) {
            pds->state = DRIVE_PDS_FAULT;
            pds->gate_was_confirmed = false;
            pds->fault_reaction_torque_allowed = false;
        }
        return make_output(pds, input, reset_edge);
    }

    if (pds->state == DRIVE_PDS_FAULT) {
        pds->gate_enable_request = false;
        pds->torque_enable = false;
        pds->quick_stop_request = false;
        if (reset_edge && !input->external_fault_active &&
            !input->gate_feedback_enabled) {
            pds->gate_interlock_latched = false;
            pds->state = input->hardware_ready
                             ? DRIVE_PDS_SWITCH_ON_DISABLED
                             : DRIVE_PDS_NOT_READY_TO_SWITCH_ON;
        }
        return make_output(pds, input, reset_edge);
    }

    if (pds->power_down_pending) {
        if (controlword_disable_voltage(input->controlword)) {
            pds->power_down_target = DRIVE_PDS_SWITCH_ON_DISABLED;
        }
        if (!input->gate_feedback_enabled) {
            pds->state = pds->power_down_target;
            pds->power_down_pending = false;
            pds->power_down_target = DRIVE_PDS_NO_POWER_DOWN_TARGET;
            pds->gate_was_confirmed = false;
        }
        return make_output(pds, input, reset_edge);
    }

    switch (pds->state) {
    case DRIVE_PDS_NOT_READY_TO_SWITCH_ON:
        pds->gate_enable_request = false;
        pds->torque_enable = false;
        if (input->hardware_ready && !input->gate_feedback_enabled) {
            pds->state = DRIVE_PDS_SWITCH_ON_DISABLED;
        }
        break;

    case DRIVE_PDS_SWITCH_ON_DISABLED:
        pds->gate_enable_request = false;
        pds->torque_enable = false;
        if (!input->hardware_ready) {
            pds->state = DRIVE_PDS_NOT_READY_TO_SWITCH_ON;
        } else if (drive_pds_controlword_is_shutdown(input->controlword)) {
            pds->state = DRIVE_PDS_READY_TO_SWITCH_ON;
        }
        break;

    case DRIVE_PDS_READY_TO_SWITCH_ON:
        pds->torque_enable = false;
        if (controlword_disable_voltage(input->controlword)) {
            pds->state = DRIVE_PDS_SWITCH_ON_DISABLED;
            pds->gate_enable_request = false;
        } else if (drive_pds_controlword_is_switch_on(input->controlword) ||
                   drive_pds_controlword_is_enable_operation(
                       input->controlword)) {
            if (input->gate_enable_allowed) {
                pds->gate_enable_request = true;
                if (input->gate_feedback_enabled) {
                    pds->gate_was_confirmed = true;
                    pds->state = DRIVE_PDS_SWITCHED_ON;
                }
            }
        } else {
            pds->gate_enable_request = false;
        }
        break;

    case DRIVE_PDS_SWITCHED_ON:
        pds->gate_enable_request = true;
        pds->torque_enable = false;
        if (controlword_disable_voltage(input->controlword)) {
            begin_power_down(pds, DRIVE_PDS_SWITCH_ON_DISABLED);
        } else if (drive_pds_controlword_is_shutdown(input->controlword)) {
            begin_power_down(pds, DRIVE_PDS_READY_TO_SWITCH_ON);
        } else if (controlword_quick_stop(input->controlword)) {
            pds->state = DRIVE_PDS_QUICK_STOP_ACTIVE;
            pds->quick_stop_request = true;
            pds->quick_stop_torque_allowed = false;
            pds->torque_enable = false;
        } else if (drive_pds_controlword_is_enable_operation(
                       input->controlword) &&
                   input->operation_enable_allowed) {
            pds->state = DRIVE_PDS_OPERATION_ENABLED;
            pds->torque_enable = true;
        }
        break;

    case DRIVE_PDS_OPERATION_ENABLED:
        pds->gate_enable_request = true;
        pds->torque_enable = input->operation_enable_allowed;
        if (controlword_disable_voltage(input->controlword)) {
            begin_power_down(pds, DRIVE_PDS_SWITCH_ON_DISABLED);
        } else if (drive_pds_controlword_is_shutdown(input->controlword)) {
            begin_power_down(pds, DRIVE_PDS_READY_TO_SWITCH_ON);
        } else if (controlword_quick_stop(input->controlword)) {
            pds->state = DRIVE_PDS_QUICK_STOP_ACTIVE;
            pds->quick_stop_request = true;
            pds->quick_stop_torque_allowed = pds->torque_enable;
            pds->torque_enable = pds->quick_stop_torque_allowed;
        } else if (controlword_disable_operation(input->controlword) ||
                   !input->operation_enable_allowed) {
            pds->state = DRIVE_PDS_SWITCHED_ON;
            pds->torque_enable = false;
        }
        break;

    case DRIVE_PDS_QUICK_STOP_ACTIVE:
        pds->gate_enable_request = true;
        pds->quick_stop_request = !input->quick_stop_complete;
        pds->torque_enable = pds->quick_stop_torque_allowed &&
                             !input->quick_stop_complete;
        if (controlword_disable_voltage(input->controlword) ||
            drive_pds_controlword_is_shutdown(input->controlword)) {
            begin_power_down(
                pds, controlword_disable_voltage(input->controlword)
                         ? DRIVE_PDS_SWITCH_ON_DISABLED
                         : DRIVE_PDS_READY_TO_SWITCH_ON);
        } else if (input->quick_stop_complete) {
            if (pds->config.quick_stop_policy ==
                DRIVE_PDS_QS_DISABLE_VOLTAGE) {
                begin_power_down(pds, DRIVE_PDS_SWITCH_ON_DISABLED);
            } else if ((pds->config.quick_stop_policy ==
                        DRIVE_PDS_QS_ALLOW_REENABLE) &&
                       drive_pds_controlword_is_enable_operation(
                           input->controlword) &&
                       input->operation_enable_allowed) {
                pds->state = DRIVE_PDS_OPERATION_ENABLED;
                pds->torque_enable = true;
                pds->quick_stop_torque_allowed = false;
            }
        }
        break;

    case DRIVE_PDS_FAULT_REACTION_ACTIVE:
    case DRIVE_PDS_FAULT:
    default:
        break;
    }

    return make_output(pds, input, reset_edge);
}
