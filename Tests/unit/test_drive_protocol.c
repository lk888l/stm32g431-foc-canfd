#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "cia402_od.h"
#include "drive_mode.h"
#include "drive_pds.h"
#include "drive_safety.h"
#include "motor_vendor_canfd.h"

#define Q16(value) ((int32_t)((value) * 65536L))
#define CW_SHUTDOWN                                                           \
    ((uint16_t)(DRIVE_PDS_CW_ENABLE_VOLTAGE | DRIVE_PDS_CW_QUICK_STOP))
#define CW_SWITCH_ON                                                          \
    ((uint16_t)(CW_SHUTDOWN | DRIVE_PDS_CW_SWITCH_ON))
#define CW_ENABLE_OPERATION                                                   \
    ((uint16_t)(CW_SWITCH_ON | DRIVE_PDS_CW_ENABLE_OPERATION))

static drive_pds_output_t pds_tick(drive_pds_t *pds,
                                   drive_pds_input_t *input,
                                   uint16_t controlword,
                                   bool gate_feedback)
{
    input->controlword = controlword;
    input->gate_feedback_enabled = gate_feedback;
    return drive_pds_step(pds, input);
}

static void test_pds_gate_interlock_quick_stop_and_fault_reset(void)
{
    drive_pds_t pds;
    drive_pds_config_t config = {DRIVE_PDS_QS_DISABLE_VOLTAGE};
    drive_pds_input_t input = {0};
    drive_pds_output_t output;

    drive_pds_init(&pds, &config);
    input.hardware_ready = true;
    input.gate_enable_allowed = true;
    input.operation_enable_allowed = true;
    input.remote = true;

    output = pds_tick(&pds, &input, 0U, false);
    assert(output.state == DRIVE_PDS_SWITCH_ON_DISABLED);
    assert((output.statusword & UINT16_C(0x006F)) == UINT16_C(0x0040));

    output = pds_tick(&pds, &input, CW_SHUTDOWN, false);
    assert(output.state == DRIVE_PDS_READY_TO_SWITCH_ON);

    /* Software requests the stage but does not claim switched-on yet. */
    output = pds_tick(&pds, &input, CW_ENABLE_OPERATION, false);
    assert(output.state == DRIVE_PDS_READY_TO_SWITCH_ON);
    assert(output.gate_enable_request);
    assert(!output.torque_enable);

    output = pds_tick(&pds, &input, CW_ENABLE_OPERATION, true);
    assert(output.state == DRIVE_PDS_SWITCHED_ON);
    assert((output.statusword & DRIVE_PDS_SW_VOLTAGE_ENABLED) != 0U);
    output = pds_tick(&pds, &input, CW_ENABLE_OPERATION, true);
    assert(output.state == DRIVE_PDS_OPERATION_ENABLED);
    assert(output.torque_enable);

    output = pds_tick(
        &pds, &input,
        (uint16_t)(CW_ENABLE_OPERATION & (uint16_t)~DRIVE_PDS_CW_QUICK_STOP),
        true);
    assert(output.state == DRIVE_PDS_QUICK_STOP_ACTIVE);
    assert(output.quick_stop_request);
    input.quick_stop_complete = true;
    output = pds_tick(&pds, &input, CW_ENABLE_OPERATION, true);
    assert(output.state == DRIVE_PDS_QUICK_STOP_ACTIVE);
    assert(!output.gate_enable_request);
    output = pds_tick(&pds, &input, CW_ENABLE_OPERATION, false);
    assert(output.state == DRIVE_PDS_SWITCH_ON_DISABLED);

    /* An energized stage in a stable off state is an interlock fault. */
    input.quick_stop_complete = false;
    input.fault_reaction_complete = false;
    output = pds_tick(&pds, &input, 0U, true);
    assert(output.state == DRIVE_PDS_FAULT_REACTION_ACTIVE);
    assert(output.gate_interlock_fault);
    assert(!output.gate_enable_request);
    input.fault_reaction_complete = true;
    output = pds_tick(&pds, &input, 0U, false);
    assert(output.state == DRIVE_PDS_FAULT);

    /* Reset is edge-triggered and only accepted while the physical stage is off. */
    output = pds_tick(&pds, &input, DRIVE_PDS_CW_FAULT_RESET, false);
    assert(output.fault_reset_rising_edge);
    assert(output.state == DRIVE_PDS_SWITCH_ON_DISABLED);
    assert(!output.gate_interlock_fault);
}

static void test_fault_reset_must_be_a_new_edge(void)
{
    drive_pds_t pds;
    drive_pds_input_t input = {0};
    drive_pds_output_t output;

    drive_pds_init(&pds, NULL);
    input.hardware_ready = true;
    input.gate_enable_allowed = true;
    input.operation_enable_allowed = true;
    (void)pds_tick(&pds, &input, DRIVE_PDS_CW_FAULT_RESET, false);
    input.external_fault_active = true;
    input.fault_reaction_requires_immediate_gate_off = true;
    input.fault_reaction_complete = true;
    output = pds_tick(&pds, &input, DRIVE_PDS_CW_FAULT_RESET, false);
    assert(output.state == DRIVE_PDS_FAULT);
    input.external_fault_active = false;
    output = pds_tick(&pds, &input, DRIVE_PDS_CW_FAULT_RESET, false);
    assert(output.state == DRIVE_PDS_FAULT);
    (void)pds_tick(&pds, &input, 0U, false);
    output = pds_tick(&pds, &input, DRIVE_PDS_CW_FAULT_RESET, false);
    assert(output.state == DRIVE_PDS_SWITCH_ON_DISABLED);
}

static void test_quick_stop_never_creates_torque_from_switched_on(void)
{
    drive_pds_t pds;
    drive_pds_input_t input = {0};
    drive_pds_output_t output;

    drive_pds_init(&pds, NULL);
    input.hardware_ready = true;
    input.gate_enable_allowed = true;
    input.operation_enable_allowed = true;
    (void)pds_tick(&pds, &input, 0U, false);
    (void)pds_tick(&pds, &input, CW_SHUTDOWN, false);
    output = pds_tick(&pds, &input, CW_SWITCH_ON, false);
    assert(output.state == DRIVE_PDS_READY_TO_SWITCH_ON);
    assert((output.statusword & DRIVE_PDS_SW_VOLTAGE_ENABLED) == 0U);
    output = pds_tick(&pds, &input, CW_SWITCH_ON, true);
    assert(output.state == DRIVE_PDS_SWITCHED_ON);
    assert((output.statusword & DRIVE_PDS_SW_VOLTAGE_ENABLED) != 0U);
    assert(!output.torque_enable);

    output = pds_tick(&pds, &input, DRIVE_PDS_CW_ENABLE_VOLTAGE, true);
    assert(output.state == DRIVE_PDS_QUICK_STOP_ACTIVE);
    assert(output.quick_stop_request);
    assert(!output.torque_enable);
    output = pds_tick(&pds, &input, DRIVE_PDS_CW_ENABLE_VOLTAGE, true);
    assert(!output.torque_enable);

    /* A controlled fault reaction from switched-on obeys the same rule. */
    drive_pds_init(&pds, NULL);
    input = (drive_pds_input_t){0};
    input.hardware_ready = true;
    input.gate_enable_allowed = true;
    input.operation_enable_allowed = true;
    (void)pds_tick(&pds, &input, 0U, false);
    (void)pds_tick(&pds, &input, CW_SHUTDOWN, false);
    (void)pds_tick(&pds, &input, CW_SWITCH_ON, false);
    output = pds_tick(&pds, &input, CW_SWITCH_ON, true);
    assert(output.state == DRIVE_PDS_SWITCHED_ON);
    input.external_fault_active = true;
    input.fault_reaction_requires_immediate_gate_off = false;
    output = pds_tick(&pds, &input, CW_SWITCH_ON, true);
    assert(output.state == DRIVE_PDS_FAULT_REACTION_ACTIVE);
    assert(output.quick_stop_request);
    assert(!output.torque_enable);
}

static void test_pp_pv_and_mit_modes(void)
{
    drive_mode_manager_t manager;
    drive_mode_config_t config = {-Q16(100), Q16(100), Q16(50)};
    drive_mode_input_t input = {0};
    drive_mode_output_t output;

    /* A legal SDO sequence (mode -> target -> bit-4 edge) may complete before
     * one supervisor tick; the captured mode/version makes it atomic. */
    assert(drive_mode_init(&manager, &config));
    input.requested_mode = DRIVE_MODE_PROFILE_POSITION;
    input.controlword = DRIVE_PDS_CW_NEW_SET_POINT;
    input.pp_setpoint_controlword = DRIVE_PDS_CW_NEW_SET_POINT;
    input.pp_setpoint_mode = DRIVE_MODE_PROFILE_POSITION;
    input.target_position_q16_16 = Q16(8);
    input.target_position_epoch = 1U;
    input.pp_request_epoch = 1U;
    output = drive_mode_step(&manager, &input);
    assert(output.pp_new_target);
    assert(output.position_reference_q16_16 == Q16(8));

    assert(drive_mode_init(&manager, &config));
    input = (drive_mode_input_t){0};
    input.requested_mode = DRIVE_MODE_PROFILE_POSITION;
    input.actual_position_q16_16 = Q16(10);
    output = drive_mode_step(&manager, &input);
    assert(output.mode_display == DRIVE_MODE_PROFILE_POSITION);
    assert(!output.pp_new_target);
    input.pp_setpoint_mode = DRIVE_MODE_PROFILE_POSITION;

    input.target_position_q16_16 = Q16(20);
    input.target_position_epoch = 1U;
    input.pp_request_epoch = 1U;
    input.pp_setpoint_controlword = DRIVE_PDS_CW_NEW_SET_POINT;
    input.controlword = DRIVE_PDS_CW_NEW_SET_POINT;
    output = drive_mode_step(&manager, &input);
    assert(output.pp_new_target);
    assert(output.pp_setpoint_acknowledge);
    assert(output.position_reference_q16_16 == Q16(20));

    /* Holding bit 4 high cannot retrigger a command. */
    output = drive_mode_step(&manager, &input);
    assert(!output.pp_new_target);
    assert(output.position_reference_q16_16 == Q16(20));

    input.controlword = 0U;
    output = drive_mode_step(&manager, &input);
    assert(!output.pp_setpoint_acknowledge);

    /* Relative command is based on the last accepted target: 20 + 5 = 25. */
    input.target_position_q16_16 = Q16(5);
    input.target_position_epoch = 2U;
    input.pp_request_epoch = 2U;
    input.pp_setpoint_controlword =
        DRIVE_PDS_CW_NEW_SET_POINT | DRIVE_PDS_CW_RELATIVE;
    input.controlword =
        DRIVE_PDS_CW_NEW_SET_POINT | DRIVE_PDS_CW_RELATIVE;
    output = drive_mode_step(&manager, &input);
    assert(output.pp_setpoint_acknowledge);
    assert(!output.pp_new_target); /* queued behind the active target */
    input.controlword = 0U;
    input.trajectory_target_reached = true;
    output = drive_mode_step(&manager, &input);
    assert(output.pp_new_target);
    assert(output.position_reference_q16_16 == Q16(25));

    /* Change-immediately replaces both the active and queued targets. */
    input.trajectory_target_reached = false;
    input.target_position_q16_16 = -Q16(7);
    input.target_position_epoch = 3U;
    input.pp_request_epoch = 3U;
    input.pp_setpoint_controlword =
        DRIVE_PDS_CW_NEW_SET_POINT |
        DRIVE_PDS_CW_CHANGE_IMMEDIATELY;
    input.controlword = DRIVE_PDS_CW_NEW_SET_POINT |
                        DRIVE_PDS_CW_CHANGE_IMMEDIATELY;
    output = drive_mode_step(&manager, &input);
    assert(output.pp_new_target);
    assert(output.position_reference_q16_16 == -Q16(7));

    input.controlword = 0U;
    input.mode_change_inhibited = true;
    input.requested_mode = DRIVE_MODE_PROFILE_VELOCITY;
    output = drive_mode_step(&manager, &input);
    assert(output.mode_change_rejected);
    assert(output.mode_display == DRIVE_MODE_PROFILE_POSITION);

    input.mode_change_inhibited = false;
    input.requested_mode = DRIVE_MODE_MIT;
    input.current_feedback_valid = false;
    output = drive_mode_step(&manager, &input);
    assert(output.mode_display == DRIVE_MODE_MIT);
    assert(!output.operation_enable_allowed);
    input.torque_active = true;
    input.current_feedback_valid = true;
    output = drive_mode_step(&manager, &input);
    assert(output.operation_enable_allowed);
    assert(output.mit_control_enabled);

    /* A stale high completion input cannot consume a newly accepted target.
     * Completion must go low and then high for that specific target. */
    assert(drive_mode_init(&manager, &config));
    input = (drive_mode_input_t){0};
    input.requested_mode = DRIVE_MODE_PROFILE_POSITION;
    input.trajectory_target_reached = true;
    (void)drive_mode_step(&manager, &input);
    input.pp_setpoint_mode = DRIVE_MODE_PROFILE_POSITION;
    input.target_position_q16_16 = Q16(4);
    input.target_position_epoch = 1U;
    input.pp_request_epoch = 1U;
    input.pp_setpoint_controlword = DRIVE_PDS_CW_NEW_SET_POINT;
    input.controlword = DRIVE_PDS_CW_NEW_SET_POINT;
    output = drive_mode_step(&manager, &input);
    assert(output.pp_new_target);
    input.controlword = 0U;
    output = drive_mode_step(&manager, &input);
    assert(!output.pp_new_target);
    assert((output.statusword_bits & DRIVE_PDS_SW_TARGET_REACHED) == 0U);
    assert(output.position_reference_q16_16 == Q16(4));
    input.trajectory_target_reached = false;
    (void)drive_mode_step(&manager, &input);
    input.trajectory_target_reached = true;
    output = drive_mode_step(&manager, &input);
    assert((output.statusword_bits & DRIVE_PDS_SW_TARGET_REACHED) != 0U);
}

static void test_pp_epoch_atomicity_and_pv_refresh(void)
{
    drive_mode_manager_t manager;
    drive_mode_config_t config = {-Q16(100), Q16(100), Q16(50)};
    drive_mode_input_t input = {0};
    drive_mode_output_t output;

    assert(drive_mode_init(&manager, &config));
    input.requested_mode = DRIVE_MODE_PROFILE_POSITION;
    (void)drive_mode_step(&manager, &input);
    input.pp_setpoint_mode = DRIVE_MODE_PROFILE_POSITION;

    /* A bit-4 edge without a target generation is rejected.  A target that
     * arrives later cannot rescue the already captured request. */
    input.controlword = DRIVE_PDS_CW_NEW_SET_POINT;
    input.pp_setpoint_controlword = DRIVE_PDS_CW_NEW_SET_POINT;
    input.pp_request_epoch = 1U;
    output = drive_mode_step(&manager, &input);
    assert(output.pp_setpoint_rejected);
    assert(!output.pp_new_target);
    assert(output.position_reference_q16_16 == 0);

    /* RPDO2 arrives after the edge: the PP snapshot remains unchanged and a
     * repeated high RPDO1 is idempotent. */
    output = drive_mode_step(&manager, &input);
    assert(!output.pp_setpoint_rejected);
    assert(!output.pp_new_target);

    /* A new low/high handshake captures and consumes the fresh target once. */
    input.controlword = 0U;
    (void)drive_mode_step(&manager, &input);
    input.target_position_q16_16 = Q16(6);
    input.target_position_epoch = 1U;
    input.pp_request_epoch = 2U;
    input.pp_setpoint_controlword = DRIVE_PDS_CW_NEW_SET_POINT;
    input.controlword = DRIVE_PDS_CW_NEW_SET_POINT;
    output = drive_mode_step(&manager, &input);
    assert(output.pp_new_target);
    assert(output.position_reference_q16_16 == Q16(6));
    output = drive_mode_step(&manager, &input);
    assert(!output.pp_new_target);
    assert(output.position_reference_q16_16 == Q16(6));

    /* A new edge that reuses the consumed epoch is rejected, including in
     * relative mode, so it cannot add the old delta twice. */
    input.controlword = 0U;
    (void)drive_mode_step(&manager, &input);
    input.target_position_q16_16 = Q16(2);
    input.pp_request_epoch = 3U;
    input.pp_setpoint_controlword =
        DRIVE_PDS_CW_NEW_SET_POINT | DRIVE_PDS_CW_RELATIVE;
    input.controlword = input.pp_setpoint_controlword;
    output = drive_mode_step(&manager, &input);
    assert(output.pp_setpoint_rejected);
    assert(!output.pp_new_target);
    assert(output.position_reference_q16_16 == Q16(6));

    /* Equality-based freshness remains correct across UINT32_MAX -> 0. */
    manager.pp_consumed_request_epoch = UINT32_MAX;
    manager.pp_consumed_position_epoch = UINT32_MAX;
    input.pp_request_epoch = 0U;
    input.target_position_epoch = 0U;
    input.target_position_q16_16 = -Q16(3);
    input.pp_setpoint_controlword =
        DRIVE_PDS_CW_NEW_SET_POINT |
        DRIVE_PDS_CW_CHANGE_IMMEDIATELY;
    input.controlword = input.pp_setpoint_controlword;
    output = drive_mode_step(&manager, &input);
    assert(output.pp_new_target);
    assert(output.position_reference_q16_16 == -Q16(3));

    /* PV is a latched value with explicit generations.  A mode transition is
     * a freshness barrier, identical-value writes still refresh, and silence
     * retains the last accepted command rather than replaying an older mode. */
    assert(drive_mode_init(&manager, &config));
    input = (drive_mode_input_t){0};
    input.requested_mode = DRIVE_MODE_PROFILE_VELOCITY;
    input.target_velocity_q16_16 = Q16(9);
    input.target_velocity_epoch = 7U;
    input.target_velocity_mode = DRIVE_MODE_PROFILE_VELOCITY;
    output = drive_mode_step(&manager, &input);
    /* Mode SDO -> velocity SDO before one supervisor tick is legitimate. */
    assert(output.velocity_reference_q16_16 == Q16(9));
    assert(output.pv_target_updated);
    input.target_velocity_epoch = 8U;
    output = drive_mode_step(&manager, &input);
    assert(output.pv_target_updated);
    assert(output.velocity_reference_q16_16 == Q16(9));
    input.target_velocity_epoch = 9U;
    output = drive_mode_step(&manager, &input);
    assert(output.pv_target_updated);
    assert(output.velocity_reference_q16_16 == Q16(9));
    output = drive_mode_step(&manager, &input);
    assert(!output.pv_target_updated);
    assert(output.velocity_reference_q16_16 == Q16(9));

    input.requested_mode = DRIVE_MODE_MIT;
    (void)drive_mode_step(&manager, &input);
    input.requested_mode = DRIVE_MODE_PROFILE_VELOCITY;
    output = drive_mode_step(&manager, &input);
    assert(!output.pv_target_updated);
    assert(output.velocity_reference_q16_16 == 0);
    input.target_velocity_epoch = 10U;
    output = drive_mode_step(&manager, &input);
    assert(output.pv_target_updated);
    assert(output.velocity_reference_q16_16 == Q16(9));
}

static void test_cross_rpdo_ordering_is_fail_closed(void)
{
    const cia402_od_config_t od_config = {
        .minimum_position_q16_16 = -Q16(100),
        .maximum_position_q16_16 = Q16(100),
        .maximum_abs_velocity_q16_16 = Q16(200),
        .maximum_abs_mit_torque_q16_16 = Q16(50),
        .maximum_abs_target_torque_per_mille = 1000,
        .maximum_profile_velocity_q16_16 = (uint32_t)Q16(200),
        .maximum_profile_acceleration_q16_16 = (uint32_t)Q16(100),
        .maximum_profile_deceleration_q16_16 = (uint32_t)Q16(100),
        .maximum_mit_kp_uq8_8 = UINT16_C(0x4000),
        .maximum_mit_kd_uq8_8 = UINT16_C(0x1000),
        .maximum_mit_current_uq8_8 = UINT16_C(0x5000),
        .support_pp = true,
        .support_pv = true,
        .support_mit = true};
    const drive_mode_config_t mode_config = {
        -Q16(100), Q16(100), Q16(50)};
    cia402_od_t od;
    drive_mode_manager_t manager;
    drive_mode_input_t input = {0};
    drive_mode_output_t output;
    uint8_t mode = (uint8_t)DRIVE_MODE_PROFILE_POSITION;
    uint8_t controlword[2] = {0U, 0U};
    uint8_t position[4] = {0U, 0U, 10U, 0U};
    uint8_t velocity[4] = {0U, 0U, 5U, 0U};

    assert(cia402_od_init(&od, &od_config));
    assert(drive_mode_init(&manager, &mode_config));
    assert(cia402_od_write(&od, CIA402_OD_INDEX_MODES_OF_OPERATION, 0U,
                           &mode, 1U, false) == CIA402_OD_OK);
    input.requested_mode = od.image.mode_command;
    input.mode_instance_epoch = od.command_sync.mode_instance_epoch;
    (void)drive_mode_step(&manager, &input);

    /* CANopenNode processes RPDO1 before RPDO2.  When both are pending in one
     * 1 ms invocation, the edge therefore snapshots epoch 0; committing the
     * complete target pair afterwards must not retroactively validate it. */
    controlword[0] = (uint8_t)DRIVE_PDS_CW_NEW_SET_POINT;
    assert(cia402_od_write(&od, CIA402_OD_INDEX_CONTROLWORD, 0U,
                           controlword, 2U, false) == CIA402_OD_OK);
    assert(cia402_od_write_target_batch(&od, position, 4U,
                                         velocity, 4U) == CIA402_OD_OK);
    input.controlword = od.image.controlword;
    input.pp_setpoint_controlword = od.command_sync.pp_request_controlword;
    input.pp_setpoint_mode = od.command_sync.pp_request_mode;
    input.pp_setpoint_mode_instance_epoch =
        od.command_sync.pp_request_mode_instance_epoch;
    input.target_position_q16_16 =
        od.command_sync.pp_request_target_position_q16_16;
    input.target_position_epoch =
        od.command_sync.pp_request_target_position_epoch;
    input.pp_request_epoch = od.command_sync.pp_request_epoch;
    output = drive_mode_step(&manager, &input);
    assert(output.pp_setpoint_rejected);
    assert(!output.pp_new_target);

    /* After RPDO2 has been processed, a later low/high RPDO1 captures epoch 1
     * and commits exactly the preloaded target. */
    controlword[0] = 0U;
    assert(cia402_od_write(&od, CIA402_OD_INDEX_CONTROLWORD, 0U,
                           controlword, 2U, false) == CIA402_OD_OK);
    input.controlword = 0U;
    (void)drive_mode_step(&manager, &input);
    controlword[0] = (uint8_t)DRIVE_PDS_CW_NEW_SET_POINT;
    assert(cia402_od_write(&od, CIA402_OD_INDEX_CONTROLWORD, 0U,
                           controlword, 2U, false) == CIA402_OD_OK);
    input.controlword = od.image.controlword;
    input.pp_setpoint_controlword = od.command_sync.pp_request_controlword;
    input.pp_setpoint_mode = od.command_sync.pp_request_mode;
    input.pp_setpoint_mode_instance_epoch =
        od.command_sync.pp_request_mode_instance_epoch;
    input.target_position_q16_16 =
        od.command_sync.pp_request_target_position_q16_16;
    input.target_position_epoch =
        od.command_sync.pp_request_target_position_epoch;
    input.pp_request_epoch = od.command_sync.pp_request_epoch;
    output = drive_mode_step(&manager, &input);
    assert(output.pp_new_target);
    assert(output.position_reference_q16_16 == Q16(10));
}

static void load_drive_input_from_od(drive_mode_input_t *input,
                                     const cia402_od_t *od)
{
    input->requested_mode = od->image.mode_command;
    input->mode_instance_epoch = od->command_sync.mode_instance_epoch;
    input->controlword = od->image.controlword;
    input->pp_setpoint_controlword =
        od->command_sync.pp_request_controlword;
    input->pp_setpoint_mode = od->command_sync.pp_request_mode;
    input->pp_setpoint_mode_instance_epoch =
        od->command_sync.pp_request_mode_instance_epoch;
    input->target_position_q16_16 =
        od->command_sync.pp_request_target_position_q16_16;
    input->target_position_epoch =
        od->command_sync.pp_request_target_position_epoch;
    input->pp_request_epoch = od->command_sync.pp_request_epoch;
    input->target_velocity_q16_16 = od->image.target_velocity_q16_16;
    input->target_velocity_epoch =
        od->command_sync.target_velocity_epoch;
    input->target_velocity_mode = od->command_sync.target_velocity_mode;
    input->target_velocity_mode_instance_epoch =
        od->command_sync.target_velocity_mode_instance_epoch;
}

static void test_mode_instance_epoch_barriers(void)
{
    const cia402_od_config_t od_config = {
        .minimum_position_q16_16 = -Q16(100),
        .maximum_position_q16_16 = Q16(100),
        .maximum_abs_velocity_q16_16 = Q16(200),
        .maximum_abs_mit_torque_q16_16 = Q16(50),
        .maximum_abs_target_torque_per_mille = 1000,
        .maximum_profile_velocity_q16_16 = (uint32_t)Q16(200),
        .maximum_profile_acceleration_q16_16 = (uint32_t)Q16(100),
        .maximum_profile_deceleration_q16_16 = (uint32_t)Q16(100),
        .maximum_mit_kp_uq8_8 = UINT16_C(0x4000),
        .maximum_mit_kd_uq8_8 = UINT16_C(0x1000),
        .maximum_mit_current_uq8_8 = UINT16_C(0x5000),
        .support_pp = true,
        .support_pv = true,
        .support_mit = true};
    const drive_mode_config_t mode_config = {
        -Q16(100), Q16(100), Q16(50)};
    cia402_od_t od;
    drive_mode_manager_t manager;
    drive_mode_input_t input = {0};
    drive_mode_output_t output;
    uint8_t mode;
    uint8_t controlword[2] = {0U, 0U};
    uint8_t position[4] = {0U, 0U, 8U, 0U};
    uint8_t velocity[4] = {0U, 0U, 7U, 0U};

    /* mode -> target -> edge is valid even when all writes precede the first
     * supervisor step, because all snapshots carry mode instance 1. */
    assert(cia402_od_init(&od, &od_config));
    assert(drive_mode_init(&manager, &mode_config));
    mode = (uint8_t)DRIVE_MODE_PROFILE_POSITION;
    assert(cia402_od_write(&od, CIA402_OD_INDEX_MODES_OF_OPERATION, 0U,
                           &mode, 1U, false) == CIA402_OD_OK);
    assert(od.command_sync.mode_instance_epoch == 1U);
    assert(cia402_od_write(&od, CIA402_OD_INDEX_TARGET_POSITION, 0U,
                           position, 4U, false) == CIA402_OD_OK);
    controlword[0] = (uint8_t)DRIVE_PDS_CW_NEW_SET_POINT;
    assert(cia402_od_write(&od, CIA402_OD_INDEX_CONTROLWORD, 0U,
                           controlword, 2U, false) == CIA402_OD_OK);
    load_drive_input_from_od(&input, &od);
    output = drive_mode_step(&manager, &input);
    assert(output.pp_new_target);
    assert(output.position_reference_q16_16 == Q16(8));

    /* A PP request is abandoned when 0x6060 leaves PP before supervision;
     * re-entering PP creates instance 3 and cannot replay instance 1. */
    assert(cia402_od_init(&od, &od_config));
    assert(drive_mode_init(&manager, &mode_config));
    input = (drive_mode_input_t){0};
    controlword[0] = 0U;
    mode = (uint8_t)DRIVE_MODE_PROFILE_POSITION;
    assert(cia402_od_write(&od, CIA402_OD_INDEX_MODES_OF_OPERATION, 0U,
                           &mode, 1U, false) == CIA402_OD_OK);
    assert(cia402_od_write(&od, CIA402_OD_INDEX_TARGET_POSITION, 0U,
                           position, 4U, false) == CIA402_OD_OK);
    controlword[0] = (uint8_t)DRIVE_PDS_CW_NEW_SET_POINT;
    assert(cia402_od_write(&od, CIA402_OD_INDEX_CONTROLWORD, 0U,
                           controlword, 2U, false) == CIA402_OD_OK);
    mode = (uint8_t)DRIVE_MODE_PROFILE_VELOCITY;
    assert(cia402_od_write(&od, CIA402_OD_INDEX_MODES_OF_OPERATION, 0U,
                           &mode, 1U, false) == CIA402_OD_OK);
    load_drive_input_from_od(&input, &od);
    output = drive_mode_step(&manager, &input);
    assert(output.mode_display == DRIVE_MODE_PROFILE_VELOCITY);
    assert(output.velocity_reference_q16_16 == 0);
    controlword[0] = 0U;
    assert(cia402_od_write(&od, CIA402_OD_INDEX_CONTROLWORD, 0U,
                           controlword, 2U, false) == CIA402_OD_OK);
    mode = (uint8_t)DRIVE_MODE_PROFILE_POSITION;
    assert(cia402_od_write(&od, CIA402_OD_INDEX_MODES_OF_OPERATION, 0U,
                           &mode, 1U, false) == CIA402_OD_OK);
    assert(od.command_sync.mode_instance_epoch == 3U);
    load_drive_input_from_od(&input, &od);
    output = drive_mode_step(&manager, &input);
    assert(output.pp_setpoint_rejected);
    assert(!output.pp_new_target);

    /* A successful same-value 0x6060 write is deliberately a new instance.
     * A request captured before it is rejected; one captured after it works. */
    position[2] = 9U;
    assert(cia402_od_write(&od, CIA402_OD_INDEX_TARGET_POSITION, 0U,
                           position, 4U, false) == CIA402_OD_OK);
    controlword[0] = (uint8_t)DRIVE_PDS_CW_NEW_SET_POINT;
    assert(cia402_od_write(&od, CIA402_OD_INDEX_CONTROLWORD, 0U,
                           controlword, 2U, false) == CIA402_OD_OK);
    assert(cia402_od_write(&od, CIA402_OD_INDEX_MODES_OF_OPERATION, 0U,
                           &mode, 1U, false) == CIA402_OD_OK);
    assert(od.command_sync.mode_instance_epoch == 4U);
    load_drive_input_from_od(&input, &od);
    output = drive_mode_step(&manager, &input);
    assert(output.pp_setpoint_rejected);
    assert(!output.pp_new_target);
    controlword[0] = 0U;
    assert(cia402_od_write(&od, CIA402_OD_INDEX_CONTROLWORD, 0U,
                           controlword, 2U, false) == CIA402_OD_OK);
    position[2] = 10U;
    assert(cia402_od_write(&od, CIA402_OD_INDEX_TARGET_POSITION, 0U,
                           position, 4U, false) == CIA402_OD_OK);
    controlword[0] = (uint8_t)DRIVE_PDS_CW_NEW_SET_POINT;
    assert(cia402_od_write(&od, CIA402_OD_INDEX_CONTROLWORD, 0U,
                           controlword, 2U, false) == CIA402_OD_OK);
    load_drive_input_from_od(&input, &od);
    output = drive_mode_step(&manager, &input);
    assert(output.pp_new_target);
    assert(output.position_reference_q16_16 == Q16(10));

    /* PV target written in instance 1 is not replayed after an unseen
     * PV->PP->PV sequence; a post-instance target is accepted immediately. */
    assert(cia402_od_init(&od, &od_config));
    assert(drive_mode_init(&manager, &mode_config));
    input = (drive_mode_input_t){0};
    mode = (uint8_t)DRIVE_MODE_PROFILE_VELOCITY;
    assert(cia402_od_write(&od, CIA402_OD_INDEX_MODES_OF_OPERATION, 0U,
                           &mode, 1U, false) == CIA402_OD_OK);
    assert(cia402_od_write_target_batch(&od, position, 4U,
                                         velocity, 4U) == CIA402_OD_OK);
    mode = (uint8_t)DRIVE_MODE_PROFILE_POSITION;
    assert(cia402_od_write(&od, CIA402_OD_INDEX_MODES_OF_OPERATION, 0U,
                           &mode, 1U, false) == CIA402_OD_OK);
    mode = (uint8_t)DRIVE_MODE_PROFILE_VELOCITY;
    assert(cia402_od_write(&od, CIA402_OD_INDEX_MODES_OF_OPERATION, 0U,
                           &mode, 1U, false) == CIA402_OD_OK);
    load_drive_input_from_od(&input, &od);
    output = drive_mode_step(&manager, &input);
    assert(output.mode_display == DRIVE_MODE_PROFILE_VELOCITY);
    assert(!output.pv_target_updated);
    assert(output.velocity_reference_q16_16 == 0);
    velocity[2] = 8U;
    assert(cia402_od_write_target_batch(&od, position, 4U,
                                         velocity, 4U) == CIA402_OD_OK);
    load_drive_input_from_od(&input, &od);
    output = drive_mode_step(&manager, &input);
    assert(output.pv_target_updated);
    assert(output.velocity_reference_q16_16 == Q16(8));

    /* Mode instance wrap uses equality only, so MAX -> 0 is a new instance. */
    assert(cia402_od_init(&od, &od_config));
    assert(drive_mode_init(&manager, &mode_config));
    input = (drive_mode_input_t){0};
    od.image.mode_command = DRIVE_MODE_PROFILE_POSITION;
    od.command_sync.mode_instance_epoch = UINT32_MAX;
    load_drive_input_from_od(&input, &od);
    (void)drive_mode_step(&manager, &input);
    mode = (uint8_t)DRIVE_MODE_PROFILE_POSITION;
    assert(cia402_od_write(&od, CIA402_OD_INDEX_MODES_OF_OPERATION, 0U,
                           &mode, 1U, false) == CIA402_OD_OK);
    assert(od.command_sync.mode_instance_epoch == 0U);
    position[2] = 11U;
    assert(cia402_od_write(&od, CIA402_OD_INDEX_TARGET_POSITION, 0U,
                           position, 4U, false) == CIA402_OD_OK);
    controlword[0] = (uint8_t)DRIVE_PDS_CW_NEW_SET_POINT;
    assert(cia402_od_write(&od, CIA402_OD_INDEX_CONTROLWORD, 0U,
                           controlword, 2U, false) == CIA402_OD_OK);
    load_drive_input_from_od(&input, &od);
    output = drive_mode_step(&manager, &input);
    assert(manager.active_mode_instance_epoch == 0U);
    assert(output.pp_new_target);
    assert(output.position_reference_q16_16 == Q16(11));
}

static void test_mode_change_inhibit_uses_pds_torque_truth(void)
{
    const cia402_od_config_t od_config = {
        .minimum_position_q16_16 = -Q16(100),
        .maximum_position_q16_16 = Q16(100),
        .maximum_abs_velocity_q16_16 = Q16(200),
        .maximum_abs_mit_torque_q16_16 = Q16(50),
        .maximum_abs_target_torque_per_mille = 1000,
        .maximum_profile_velocity_q16_16 = (uint32_t)Q16(200),
        .maximum_profile_acceleration_q16_16 = (uint32_t)Q16(100),
        .maximum_profile_deceleration_q16_16 = (uint32_t)Q16(100),
        .maximum_mit_kp_uq8_8 = UINT16_C(0x4000),
        .maximum_mit_kd_uq8_8 = UINT16_C(0x1000),
        .maximum_mit_current_uq8_8 = UINT16_C(0x5000),
        .support_pp = true,
        .support_pv = true,
        .support_mit = true};
    const drive_mode_config_t mode_config = {
        -Q16(100), Q16(100), Q16(50)};
    cia402_od_t od;
    drive_mode_manager_t manager;
    drive_mode_input_t input = {0};
    drive_mode_output_t output;
    uint8_t mode = (uint8_t)DRIVE_MODE_PROFILE_POSITION;

    assert(cia402_od_init(&od, &od_config));
    assert(cia402_od_write(&od, CIA402_OD_INDEX_MODES_OF_OPERATION, 0U,
                           &mode, 1U, false) == CIA402_OD_OK);
    assert(od.image.mode_command == DRIVE_MODE_PROFILE_POSITION);
    assert(od.command_sync.mode_instance_epoch == 1U);

    mode = (uint8_t)DRIVE_MODE_PROFILE_VELOCITY;
    /* OP Enabled remains an inhibit even if a transient published
     * torque_enable sample is false. */
    assert(cia402_od_write(&od, CIA402_OD_INDEX_MODES_OF_OPERATION, 0U,
                           &mode, 1U, true) == CIA402_OD_STATE_CONFLICT);
    assert(od.image.mode_command == DRIVE_MODE_PROFILE_POSITION);
    assert(od.command_sync.mode_instance_epoch == 1U);
    /* Quick Stop Active with controlled torque uses the same inhibit. */
    assert(cia402_od_write(&od, CIA402_OD_INDEX_MODES_OF_OPERATION, 0U,
                           &mode, 1U, true) == CIA402_OD_STATE_CONFLICT);
    assert(od.image.mode_command == DRIVE_MODE_PROFILE_POSITION);
    assert(od.command_sync.mode_instance_epoch == 1U);
    /* Once Quick Stop is complete and torque is off, the mode write commits. */
    assert(cia402_od_write(&od, CIA402_OD_INDEX_MODES_OF_OPERATION, 0U,
                           &mode, 1U, false) == CIA402_OD_OK);
    assert(od.image.mode_command == DRIVE_MODE_PROFILE_VELOCITY);
    assert(od.command_sync.mode_instance_epoch == 2U);

    assert(drive_mode_init(&manager, &mode_config));
    input.requested_mode = DRIVE_MODE_PROFILE_POSITION;
    input.mode_instance_epoch = 1U;
    (void)drive_mode_step(&manager, &input);
    input.pp_setpoint_mode = DRIVE_MODE_PROFILE_POSITION;
    input.pp_setpoint_mode_instance_epoch = 1U;
    input.target_position_q16_16 = Q16(4);
    input.target_position_epoch = 1U;
    input.pp_request_epoch = 1U;
    input.pp_setpoint_controlword = DRIVE_PDS_CW_NEW_SET_POINT;
    input.controlword = DRIVE_PDS_CW_NEW_SET_POINT;
    output = drive_mode_step(&manager, &input);
    assert(output.pp_new_target);
    assert(manager.pp_has_active_target);

    input.requested_mode = DRIVE_MODE_PROFILE_VELOCITY;
    input.mode_instance_epoch = 2U;
    input.mode_change_inhibited = true;
    input.torque_active = false; /* OP Enabled transient torque sample. */
    output = drive_mode_step(&manager, &input);
    assert(output.mode_change_rejected);
    assert(manager.active_mode == DRIVE_MODE_PROFILE_POSITION);
    assert(manager.active_mode_instance_epoch == 1U);
    assert(manager.pp_has_active_target); /* No reset on rejection. */

    input.torque_active = true; /* Quick Stop controlled-torque phase. */
    output = drive_mode_step(&manager, &input);
    assert(output.mode_change_rejected);
    assert(manager.active_mode == DRIVE_MODE_PROFILE_POSITION);
    assert(manager.active_mode_instance_epoch == 1U);
    assert(manager.pp_has_active_target);

    input.mode_change_inhibited = false;
    input.torque_active = false; /* Quick Stop complete. */
    output = drive_mode_step(&manager, &input);
    assert(!output.mode_change_rejected);
    assert(output.mode_display == DRIVE_MODE_PROFILE_VELOCITY);
    assert(manager.active_mode_instance_epoch == 2U);
    assert(!manager.pp_has_active_target);
}

static motor_vendor_command_limits_t command_limits(void)
{
    const motor_vendor_command_limits_t limits = {
        -Q16(100), Q16(100), Q16(200), Q16(50),
        UINT16_C(0x4000), UINT16_C(0x1000), UINT16_C(0x5000)};
    return limits;
}

static void test_vendor_canfd_command(void)
{
    const motor_vendor_command_limits_t limits = command_limits();
    motor_vendor_command_t command = {0};
    motor_vendor_command_t decoded;
    motor_vendor_rx_guard_t guard;
    uint8_t frame[MOTOR_VENDOR_CANFD_COMMAND_LENGTH];

    command.flags = MOTOR_VENDOR_CMD_ENABLE;
    command.sequence = UINT16_C(0x1234);
    command.sender_timestamp_us = UINT32_C(0x89ABCDEF);
    command.position_q16_16 = -Q16(2);
    command.velocity_q16_16 = Q16(3);
    command.torque_q16_16 = -Q16(4);
    command.kp_uq8_8 = UINT16_C(0x0100);
    command.kd_uq8_8 = UINT16_C(0x0080);
    command.current_limit_uq8_8 = UINT16_C(0x0200);
    command.mode = DRIVE_MODE_MIT;

    assert(motor_vendor_encode_command(&command, &limits, frame,
                                       sizeof(frame)) ==
           MOTOR_VENDOR_CANFD_OK);
    assert(frame[4] == UINT8_C(0x34));
    assert(frame[5] == UINT8_C(0x12));
    assert(frame[6] == UINT8_C(0xEF));
    assert(frame[9] == UINT8_C(0x89));
    assert(motor_vendor_decode_command(frame, sizeof(frame), &limits,
                                       &decoded) ==
           MOTOR_VENDOR_CANFD_OK);
    assert(decoded.position_q16_16 == command.position_q16_16);
    assert(decoded.torque_q16_16 == command.torque_q16_16);
    assert(decoded.mode == DRIVE_MODE_MIT);

    motor_vendor_rx_guard_init(&guard, 4U);
    assert(motor_vendor_accept_command(&guard, frame, sizeof(frame),
                                       UINT32_C(0xFFFFFFF0), &limits,
                                       &decoded) == MOTOR_VENDOR_CANFD_OK);
    assert(decoded.local_rx_timestamp_us == UINT32_C(0xFFFFFFF0));
    assert(motor_vendor_command_is_fresh(&decoded, UINT32_C(0x00000020),
                                         48U));
    assert(!motor_vendor_command_is_fresh(&decoded, UINT32_C(0x00000021),
                                          48U));
    assert(motor_vendor_accept_command(&guard, frame, sizeof(frame), 0U,
                                       &limits, &decoded) ==
           MOTOR_VENDOR_CANFD_SEQUENCE_DUPLICATE);

    command.sequence = UINT16_C(0x1239); /* forward gap is five, limit is four */
    assert(motor_vendor_encode_command(&command, &limits, frame,
                                       sizeof(frame)) ==
           MOTOR_VENDOR_CANFD_OK);
    assert(motor_vendor_accept_command(&guard, frame, sizeof(frame), 1U,
                                       &limits, &decoded) ==
           MOTOR_VENDOR_CANFD_SEQUENCE_GAP);
    assert(decoded.sequence == UINT16_C(0x1239));
    assert(decoded.local_rx_timestamp_us == 1U);
    assert(guard.last_sequence == UINT16_C(0x1239));
    assert(guard.last_local_rx_timestamp_us == 1U);

    command.sequence = UINT16_C(0x1235);
    assert(motor_vendor_encode_command(&command, &limits, frame,
                                       sizeof(frame)) ==
           MOTOR_VENDOR_CANFD_OK);
    frame[10] ^= UINT8_C(0x01);
    assert(motor_vendor_decode_command(frame, sizeof(frame), &limits,
                                       &decoded) ==
           MOTOR_VENDOR_CANFD_CRC_ERROR);
    assert(motor_vendor_decode_command(frame, sizeof(frame) - 1U, &limits,
                                       &decoded) ==
           MOTOR_VENDOR_CANFD_LENGTH_ERROR);
}

static void test_vendor_canfd_feedback(void)
{
    const motor_vendor_feedback_limits_t limits = {
        -Q16(100), Q16(100), Q16(200), Q16(50), Q16(100),
        (uint32_t)Q16(100)};
    motor_vendor_feedback_t feedback = {0};
    motor_vendor_feedback_t decoded;
    uint8_t frame[MOTOR_VENDOR_CANFD_FEEDBACK_LENGTH];

    feedback.flags = MOTOR_VENDOR_FB_OPERATION_ENABLED |
                     MOTOR_VENDOR_FB_COMMAND_FRESH;
    feedback.sequence = 77U;
    feedback.command_sequence_echo = 76U;
    feedback.local_timestamp_us = 123456U;
    feedback.position_q16_16 = Q16(2);
    feedback.velocity_q16_16 = -Q16(3);
    feedback.torque_q16_16 = Q16(4);
    feedback.iq_q16_16 = -Q16(5);
    feedback.bus_voltage_uq16_16 = (uint32_t)Q16(24);
    feedback.statusword = UINT16_C(0x1234);
    feedback.mode_display = DRIVE_MODE_MIT;
    feedback.pds_state = DRIVE_PDS_OPERATION_ENABLED;
    feedback.active_faults = UINT32_C(0x01020304);
    feedback.latched_faults = UINT32_C(0x05060708);
    feedback.torque_limit_q15 = UINT16_C(30000);

    assert(motor_vendor_encode_feedback(&feedback, &limits, frame,
                                        sizeof(frame)) ==
           MOTOR_VENDOR_CANFD_OK);
    assert(motor_vendor_decode_feedback(frame, sizeof(frame), &limits,
                                        &decoded) ==
           MOTOR_VENDOR_CANFD_OK);
    assert(decoded.sequence == feedback.sequence);
    assert(decoded.velocity_q16_16 == feedback.velocity_q16_16);
    assert(decoded.active_faults == feedback.active_faults);
    assert(decoded.latched_faults == feedback.latched_faults);
}

static void test_safety_derate_latch_and_timeouts(void)
{
    drive_safety_t safety;
    drive_safety_config_t config = drive_safety_default_config();
    drive_safety_input_t input = {0};
    drive_safety_output_t output;

    assert(drive_safety_init(&safety, &config));
    assert(config.gate_inhibit_mask == config.immediate_gate_off_mask);
    assert((config.immediate_gate_off_mask &
            DRIVE_FAULT_POSITION_FEEDBACK) != 0U);
    input.command_watchdog_required = true;
    input.thermal_torque_limit_q15 = DRIVE_SAFETY_FULL_TORQUE_Q15;
    input.supply_torque_limit_q15 = UINT16_C(25000);
    drive_safety_note_valid_command(&safety, UINT32_C(0xFFFFFFF0));
    input.now_us = UINT32_C(0x00000020);
    output = drive_safety_step(&safety, &input);
    assert(output.command_age_us == 48U);
    assert(output.torque_limit_q15 == UINT16_C(25000));

    input.now_us =
        (uint32_t)(UINT32_C(0xFFFFFFF0) + config.command_soft_timeout_us);
    output = drive_safety_step(&safety, &input);
    assert(output.command_soft_timeout);
    assert(output.controlled_stop_request);
    assert(output.gate_enable_allowed);
    assert(output.torque_limit_q15 == config.soft_timeout_torque_limit_q15);

    input.now_us =
        (uint32_t)(UINT32_C(0xFFFFFFF0) + config.command_hard_timeout_us);
    output = drive_safety_step(&safety, &input);
    assert(output.command_hard_timeout);
    assert(output.immediate_gate_off);
    assert(!output.gate_enable_allowed);
    assert((output.latched_faults & DRIVE_FAULT_COMMAND_HARD_TIMEOUT) != 0U);

    /* Reload first; then a safe reset can clear the inactive hard-timeout latch. */
    drive_safety_note_valid_command(&safety, input.now_us);
    input.reset_rising_edge = true;
    input.safe_to_reset = true;
    output = drive_safety_step(&safety, &input);
    assert((output.latched_faults & DRIVE_FAULT_COMMAND_HARD_TIMEOUT) == 0U);

    input.reset_rising_edge = false;
    input.sensed_faults = DRIVE_FAULT_PHASE_OVERCURRENT;
    output = drive_safety_step(&safety, &input);
    assert(output.immediate_gate_off);
    input.sensed_faults = 0U;
    output = drive_safety_step(&safety, &input);
    assert((output.latched_faults & DRIVE_FAULT_PHASE_OVERCURRENT) != 0U);

    /* Watchdog monitoring cannot remain accidentally unarmed while required. */
    assert(drive_safety_init(&safety, &config));
    input = (drive_safety_input_t){0};
    input.command_watchdog_required = true;
    input.thermal_torque_limit_q15 = DRIVE_SAFETY_FULL_TORQUE_Q15;
    input.supply_torque_limit_q15 = DRIVE_SAFETY_FULL_TORQUE_Q15;
    input.now_us = 1000U;
    output = drive_safety_step(&safety, &input);
    assert(safety.command_watchdog_armed);
    assert(output.command_age_us == 0U);
    input.now_us = 1000U + config.command_soft_timeout_us;
    output = drive_safety_step(&safety, &input);
    assert(output.command_soft_timeout);

    /* A controlled-stop fault is not also a gate inhibit. */
    assert(drive_safety_init(&safety, &config));
    input = (drive_safety_input_t){0};
    input.thermal_torque_limit_q15 = DRIVE_SAFETY_FULL_TORQUE_Q15;
    input.supply_torque_limit_q15 = DRIVE_SAFETY_FULL_TORQUE_Q15;
    input.sensed_faults = DRIVE_FAULT_POWER_STAGE_OVERTEMP;
    output = drive_safety_step(&safety, &input);
    assert(output.controlled_stop_request);
    assert(!output.immediate_gate_off);
    assert(output.gate_enable_allowed);
}

static void test_cia402_od_adapter(void)
{
    const cia402_od_config_t config = {
        .minimum_position_q16_16 = -Q16(100),
        .maximum_position_q16_16 = Q16(100),
        .maximum_abs_velocity_q16_16 = Q16(200),
        .maximum_abs_mit_torque_q16_16 = Q16(50),
        .maximum_abs_target_torque_per_mille = 1000,
        .maximum_profile_velocity_q16_16 = (uint32_t)Q16(200),
        .maximum_profile_acceleration_q16_16 = (uint32_t)Q16(100),
        .maximum_profile_deceleration_q16_16 = (uint32_t)Q16(150),
        .maximum_mit_kp_uq8_8 = UINT16_C(0x4000),
        .maximum_mit_kd_uq8_8 = UINT16_C(0x1000),
        .maximum_mit_current_uq8_8 = UINT16_C(0x5000),
        .support_pp = true,
        .support_pv = true,
        .support_mit = true};
    cia402_od_t od;
    uint8_t bytes[4];
    uint8_t position_bytes[4];
    uint8_t velocity_bytes[4];
    size_t length;

    assert(cia402_od_init(&od, &config));
    assert(od.image.profile_deceleration_q16_16 == (uint32_t)Q16(150));

    /* RPDO2's two fields validate before either is published. */
    position_bytes[0] = UINT8_C(0x00);
    position_bytes[1] = UINT8_C(0x00);
    position_bytes[2] = UINT8_C(0x0A);
    position_bytes[3] = UINT8_C(0x00); /* 10.0 */
    velocity_bytes[0] = UINT8_C(0x00);
    velocity_bytes[1] = UINT8_C(0x00);
    velocity_bytes[2] = UINT8_C(0x05);
    velocity_bytes[3] = UINT8_C(0x00); /* 5.0 */
    assert(cia402_od_write_target_batch(&od,
                                         position_bytes,
                                         sizeof(position_bytes),
                                         velocity_bytes,
                                         sizeof(velocity_bytes)) ==
           CIA402_OD_OK);
    assert(od.image.target_position_q16_16 == Q16(10));
    assert(od.image.target_velocity_q16_16 == Q16(5));
    assert(od.command_sync.target_position_epoch == 1U);
    assert(od.command_sync.target_velocity_epoch == 1U);

    position_bytes[2] = UINT8_C(0x14); /* otherwise-valid 20.0 */
    velocity_bytes[2] = UINT8_C(0xC9); /* invalid 201.0 */
    assert(cia402_od_write_target_batch(&od,
                                         position_bytes,
                                         sizeof(position_bytes),
                                         velocity_bytes,
                                         sizeof(velocity_bytes)) ==
           CIA402_OD_VALUE_RANGE);
    assert(od.image.target_position_q16_16 == Q16(10));
    assert(od.image.target_velocity_q16_16 == Q16(5));
    assert(od.command_sync.target_position_epoch == 1U);
    assert(od.command_sync.target_velocity_epoch == 1U);

    /* The PP rising edge captures the preloaded version/value.  A later
     * 0x607A write, even before the next supervisor step, cannot alter it. */
    bytes[0] = UINT8_C(0x10);
    bytes[1] = UINT8_C(0x00);
    assert(cia402_od_write(&od, CIA402_OD_INDEX_CONTROLWORD, 0U,
                           bytes, 2U, false) == CIA402_OD_OK);
    assert(od.command_sync.pp_request_epoch == 1U);
    assert(od.command_sync.pp_request_target_position_epoch == 1U);
    assert(od.command_sync.pp_request_target_position_q16_16 == Q16(10));
    position_bytes[2] = UINT8_C(0x14);
    assert(cia402_od_write(&od, CIA402_OD_INDEX_TARGET_POSITION, 0U,
                           position_bytes, 4U, false) == CIA402_OD_OK);
    assert(od.command_sync.target_position_epoch == 2U);
    assert(od.command_sync.pp_request_target_position_epoch == 1U);
    assert(od.command_sync.pp_request_target_position_q16_16 == Q16(10));
    assert(cia402_od_write(&od, CIA402_OD_INDEX_CONTROLWORD, 0U,
                           bytes, 2U, false) == CIA402_OD_OK);
    assert(od.command_sync.pp_request_epoch == 1U);

    /* Unsigned epoch wrap remains fresh by inequality. */
    od.command_sync.target_position_epoch = UINT32_MAX;
    position_bytes[2] = UINT8_C(0x03);
    assert(cia402_od_write(&od, CIA402_OD_INDEX_TARGET_POSITION, 0U,
                           position_bytes, 4U, false) == CIA402_OD_OK);
    assert(od.command_sync.target_position_epoch == 0U);
    bytes[0] = UINT8_C(0x00);
    assert(cia402_od_write(&od, CIA402_OD_INDEX_CONTROLWORD, 0U,
                           bytes, 2U, false) == CIA402_OD_OK);
    bytes[0] = UINT8_C(0x10);
    assert(cia402_od_write(&od, CIA402_OD_INDEX_CONTROLWORD, 0U,
                           bytes, 2U, false) == CIA402_OD_OK);
    assert(od.command_sync.pp_request_epoch == 2U);
    assert(od.command_sync.pp_request_target_position_epoch == 0U);

    /* Deceleration has its own board limit; it is intentionally allowed to
     * exceed the acceleration limit. */
    bytes[0] = UINT8_C(0x00);
    bytes[1] = UINT8_C(0x00);
    bytes[2] = UINT8_C(0x78); /* 120.0 in Q16.16 */
    bytes[3] = UINT8_C(0x00);
    assert(cia402_od_write(&od, CIA402_OD_INDEX_PROFILE_DECELERATION, 0U,
                           bytes, 4U, false) == CIA402_OD_OK);
    bytes[2] = UINT8_C(0x97); /* 151.0 in Q16.16 */
    assert(cia402_od_write(&od, CIA402_OD_INDEX_PROFILE_DECELERATION, 0U,
                           bytes, 4U, false) == CIA402_OD_VALUE_RANGE);

    bytes[0] = (uint8_t)DRIVE_MODE_MIT;
    assert(cia402_od_write(&od, CIA402_OD_INDEX_MODES_OF_OPERATION, 0U,
                           bytes, 1U, true) == CIA402_OD_STATE_CONFLICT);
    assert(od.command_sync.mode_instance_epoch == 0U);
    assert(cia402_od_write(&od, CIA402_OD_INDEX_MODES_OF_OPERATION, 0U,
                           bytes, 1U, false) == CIA402_OD_OK);
    assert(od.command_sync.mode_instance_epoch == 1U);
    /* Same-value successful writes deliberately create a new instance. */
    assert(cia402_od_write(&od, CIA402_OD_INDEX_MODES_OF_OPERATION, 0U,
                           bytes, 1U, false) == CIA402_OD_OK);
    assert(od.command_sync.mode_instance_epoch == 2U);

    bytes[0] = UINT8_C(0x34);
    bytes[1] = UINT8_C(0x12);
    assert(cia402_od_write(&od, CIA402_OD_INDEX_CONTROLWORD, 0U,
                           bytes, 2U, false) == CIA402_OD_OK);
    memset(bytes, 0, sizeof(bytes));
    assert(cia402_od_read(&od, CIA402_OD_INDEX_CONTROLWORD, 0U, bytes,
                          sizeof(bytes), &length) == CIA402_OD_OK);
    assert(length == 2U);
    assert(bytes[0] == UINT8_C(0x34) && bytes[1] == UINT8_C(0x12));

    bytes[0] = UINT8_C(0x00);
    bytes[1] = UINT8_C(0x01);
    assert(cia402_od_write(&od, CIA402_OD_INDEX_MIT_COMMAND,
                           CIA402_OD_MIT_SUB_KP, bytes, 2U, false) ==
           CIA402_OD_OK);
    assert(od.image.mit_kp_uq8_8 == UINT16_C(0x0100));

    bytes[0] = UINT8_C(0x01);
    bytes[1] = UINT8_C(0x00);
    bytes[2] = UINT8_C(0x33);
    bytes[3] = UINT8_C(0x00); /* Q16.16 value above configured MIT torque. */
    assert(cia402_od_write(&od, CIA402_OD_INDEX_MIT_COMMAND,
                           CIA402_OD_MIT_SUB_TORQUE, bytes, 4U, false) ==
           CIA402_OD_VALUE_RANGE);
    bytes[0] = UINT8_C(0xFF);
    bytes[1] = UINT8_C(0xFF);
    assert(cia402_od_write(&od, CIA402_OD_INDEX_TARGET_TORQUE, 0U,
                           bytes, 2U, false) == CIA402_OD_OK);
    assert(od.image.target_torque_per_mille == -1);
    assert(cia402_od_write(&od, CIA402_OD_INDEX_STATUSWORD, 0U,
                           bytes, 2U, false) == CIA402_OD_READ_ONLY);
}

int main(void)
{
    test_pds_gate_interlock_quick_stop_and_fault_reset();
    test_fault_reset_must_be_a_new_edge();
    test_quick_stop_never_creates_torque_from_switched_on();
    test_pp_pv_and_mit_modes();
    test_pp_epoch_atomicity_and_pv_refresh();
    test_cross_rpdo_ordering_is_fail_closed();
    test_mode_instance_epoch_barriers();
    test_mode_change_inhibit_uses_pds_torque_truth();
    test_vendor_canfd_command();
    test_vendor_canfd_feedback();
    test_safety_derate_latch_and_timeouts();
    test_cia402_od_adapter();
    puts("drive/protocol unit tests passed");
    return 0;
}
