#include "app.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#include "board_config.h"
#include "bsp.h"
#include "canopen_classic.h"
#include "canfd_transport_stm32g4.h"
#include "control_pi.h"
#include "control_trajectory.h"
#include "drive_mode.h"
#include "drive_pds.h"
#include "drive_safety.h"
#include "encoder_as5600.h"
#include "encoder_mt6701.h"
#include "encoder_tracker.h"
#include "mit_controller.h"
#include "motor_foc.h"

#define APP_Q16_SCALE 65536.0f
#define APP_UQ8_SCALE 256.0f
#define APP_CONTROL_PERIOD_S (1.0f / (float)BOARD_CONTROL_FREQUENCY_HZ)
#define APP_SUPERVISOR_PERIOD_S (1.0f / (float)BOARD_SUPERVISOR_FREQUENCY_HZ)
#define APP_SUPERVISOR_DIVIDER \
    (BOARD_CONTROL_FREQUENCY_HZ / BOARD_SUPERVISOR_FREQUENCY_HZ)
#define APP_STARTUP_SENSOR_GRACE_US 100000UL
#define APP_FEEDBACK_PERIOD_US 2000UL
#define APP_BUS_SAMPLE_PERIOD_US 10000UL
#define APP_ENCODER_SAMPLE_PERIOD_US_LAB 1000UL
#define APP_CURRENT_LOOP_STALE_US 500UL
#define APP_SUPERVISOR_LOOP_STALE_US 5000UL
#define APP_TIM2_STALE_US 2000UL
#define APP_ENCODER_DMA_TIMEOUT_US 500UL
#define APP_CYCLES_FROM_US(value_us) \
    ((uint32_t)(((uint64_t)BOARD_SYSCLK_HZ * (uint64_t)(value_us)) / 1000000ULL))
#define APP_CURRENT_LOOP_STALE_CYCLES \
    APP_CYCLES_FROM_US(APP_CURRENT_LOOP_STALE_US)
#define APP_SUPERVISOR_LOOP_STALE_CYCLES \
    APP_CYCLES_FROM_US(APP_SUPERVISOR_LOOP_STALE_US)
#define APP_TIM2_STALE_CYCLES APP_CYCLES_FROM_US(APP_TIM2_STALE_US)
#define APP_ENCODER_DMA_TIMEOUT_CYCLES \
    APP_CYCLES_FROM_US(APP_ENCODER_DMA_TIMEOUT_US)
#define APP_ISR_DEADLINE_CYCLES \
    (BOARD_SYSCLK_HZ / BOARD_CONTROL_FREQUENCY_HZ)

typedef struct {
    motor_foc_t motor;
    encoder_tracker_t position_tracker;
    encoder_mt6701_t mt6701;
    encoder_as5600_t as5600;
    control_pi_t speed_pi;
    control_trajectory_state_t trajectory;
    control_trajectory_limits_t trajectory_limits;
    control_mit_limits_t mit_limits;
    control_mit_command_t mit_command;
    drive_mode_manager_t mode_manager;
    drive_mode_output_t mode_output;
    drive_pds_t pds;
    drive_pds_output_t pds_output;
    drive_safety_t safety;
    drive_safety_output_t safety_output;
    cia402_od_t od;
    motor_vendor_command_limits_t vendor_command_limits;
    motor_vendor_feedback_limits_t vendor_feedback_limits;
    motor_vendor_feedback_t feedback_frame;
    encoder_sample_t pending_encoder;
    bool pending_encoder_valid;
    bool encoder_driver_initialized;
    bool pending_watchdog_reload;
    uint32_t pending_watchdog_timestamp_us;
    uint32_t fast_fault_events;
    uint32_t boot_timestamp_us;
    uint32_t last_encoder_poll_us;
    uint32_t last_bus_sample_us;
    uint32_t last_feedback_tx_us;
    uint32_t last_canopen_background_us;
    uint32_t realtime_monitor_start_cycles;
    uint32_t monitor_current_progress_cycles;
    uint32_t monitor_supervisor_progress_cycles;
    uint32_t monitor_tim2_progress_cycles;
    uint32_t monitor_dwt_cycles;
    uint32_t monitor_dwt_progress_us;
    uint32_t monitor_current_completion_count;
    uint32_t monitor_supervisor_completion_count;
    uint32_t monitor_tim2_us;
    volatile uint32_t current_completion_count;
    volatile uint32_t supervisor_completion_count;
    uint32_t supervisor_divider;
    uint16_t previous_safety_controlword;
    uint16_t last_command_sequence;
    uint16_t feedback_sequence;
    uint16_t last_od_mit_sequence;
    uint8_t previous_vendor_flags;
    float mit_command_current_limit_a;
    float last_position_rad;
    volatile bool current_loop_seen;
    volatile bool supervisor_loop_seen;
    bool realtime_monitor_started;
    bool initialized;
} app_state_t;

static app_state_t app;

static uint32_t critical_enter(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void critical_exit(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

static void latch_fault_event(uint32_t fault_bits)
{
    const uint32_t primask = critical_enter();
    app.fast_fault_events |= fault_bits;
    critical_exit(primask);
}

static float q16_to_float(int32_t value)
{
    return (float)value / APP_Q16_SCALE;
}

static int32_t float_to_q16(float value)
{
    const double scaled = (double)value * 65536.0;
    if (!isfinite(value)) {
        return 0;
    }
    if (scaled >= (double)INT32_MAX) {
        return INT32_MAX;
    }
    if (scaled <= (double)INT32_MIN) {
        return INT32_MIN;
    }
    return (int32_t)llround(scaled);
}

static uint32_t float_to_uq16(float value)
{
    const double scaled = (double)value * 65536.0;
    if (!isfinite(value) || (value <= 0.0f)) {
        return 0U;
    }
    if (scaled >= (double)UINT32_MAX) {
        return UINT32_MAX;
    }
    return (uint32_t)llround(scaled);
}

static uint16_t float_to_uq8(float value)
{
    const float scaled = value * APP_UQ8_SCALE;
    if (!isfinite(value) || (value <= 0.0f)) {
        return 0U;
    }
    if (scaled >= (float)UINT16_MAX) {
        return UINT16_MAX;
    }
    return (uint16_t)lroundf(scaled);
}

static float clampf_local(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static int16_t torque_per_mille(float iq_a)
{
    const float normalized =
        iq_a / BOARD_MOTOR_PEAK_CURRENT_A * 1000.0f;
    const float limited = clampf_local(normalized, -1000.0f, 1000.0f);
    return (int16_t)lroundf(limited);
}

static uint16_t error_code_from_faults(uint32_t faults)
{
    if ((faults & DRIVE_FAULT_PHASE_OVERCURRENT) != 0U) {
        return 0x2310U;
    }
    if ((faults & DRIVE_FAULT_DC_OVERVOLTAGE) != 0U) {
        return 0x3210U;
    }
    if ((faults & DRIVE_FAULT_DC_UNDERVOLTAGE) != 0U) {
        return 0x3220U;
    }
    if ((faults & (DRIVE_FAULT_POWER_STAGE_OVERTEMP |
                   DRIVE_FAULT_MOTOR_OVERTEMP)) != 0U) {
        return 0x4310U;
    }
    if ((faults & (DRIVE_FAULT_CURRENT_FEEDBACK |
                   DRIVE_FAULT_POSITION_FEEDBACK)) != 0U) {
        return 0x7300U;
    }
    if ((faults & DRIVE_FAULT_OVERSPEED) != 0U) {
        return 0x7310U;
    }
    if ((faults & (DRIVE_FAULT_CAN_BUS_OFF |
                   DRIVE_FAULT_COMMAND_SOFT_TIMEOUT |
                   DRIVE_FAULT_COMMAND_HARD_TIMEOUT |
                   DRIVE_FAULT_PROTOCOL)) != 0U) {
        return 0x8110U;
    }
    if (faults != 0U) {
        return 0xFF00U;
    }
    return 0U;
}

static bool configure_motor(void)
{
    motor_foc_config_t config = {0};
    const float phase_voltage_limit =
        BOARD_BUS_NOMINAL_V * 0.95f * 0.57735026919f;

#if BOARD_PROFILE == BOARD_PROFILE_PRODUCTION_3SHUNT_MT6701
    config.strategy = MOTOR_FOC_CURRENT_CLOSED_LOOP;
    config.encoder_stale_timeout_us = BOARD_ENCODER_STALE_US_PRODUCTION;
#else
    config.strategy = MOTOR_FOC_LAB_VOLTAGE_LOOP;
    config.encoder_stale_timeout_us = BOARD_ENCODER_STALE_US_LAB;
#endif
    config.control_period_s = APP_CONTROL_PERIOD_S;
    config.pole_pairs = BOARD_MOTOR_POLE_PAIRS;
    config.encoder_direction = BOARD_ENCODER_DIRECTION;
    config.electrical_zero_rad = BOARD_ELECTRICAL_ZERO_RAD;
    config.electrical_zero_valid = BOARD_ELECTRICAL_ZERO_VALID != 0;
    config.phase_a_current_a_per_count =
        BOARD_PHASE_A_CURRENT_A_PER_COUNT;
    config.phase_b_current_a_per_count =
        BOARD_PHASE_B_CURRENT_A_PER_COUNT;
    config.adc_min_valid_count = BOARD_CURRENT_ADC_MIN_VALID_COUNT;
    config.adc_max_valid_count = BOARD_CURRENT_ADC_MAX_VALID_COUNT;
    config.offset_min_valid_count = BOARD_CURRENT_OFFSET_MIN_COUNT;
    config.offset_max_valid_count = BOARD_CURRENT_OFFSET_MAX_COUNT;
    config.offset_max_span_count = BOARD_CURRENT_OFFSET_MAX_SPAN_COUNT;
    config.offset_calibration_samples = BOARD_CURRENT_OFFSET_SAMPLES;
    config.phase_resistance_ohm = BOARD_MOTOR_RS_OHM;
    config.d_inductance_h = BOARD_MOTOR_LD_H;
    config.q_inductance_h = BOARD_MOTOR_LQ_H;
    config.flux_linkage_wb = BOARD_MOTOR_FLUX_WB;
    config.current_limit_a = BOARD_MOTOR_PEAK_CURRENT_A;
    config.bus_nominal_v = BOARD_BUS_NOMINAL_V;
    config.voltage_utilization = 0.90f;
    config.minimum_pulse_s =
        (float)BOARD_PWM_MIN_PULSE_TICKS / (float)BOARD_SYSCLK_HZ;

    config.d_current_pi.proportional_gain = BOARD_CURRENT_KP_V_PER_A;
    config.d_current_pi.integral_gain_per_s = BOARD_CURRENT_KI_V_PER_AS;
    config.d_current_pi.anti_windup_gain_per_s = BOARD_CURRENT_AW_PER_S;
    config.d_current_pi.output_min = -phase_voltage_limit;
    config.d_current_pi.output_max = phase_voltage_limit;
    config.d_current_pi.integrator_min = -phase_voltage_limit;
    config.d_current_pi.integrator_max = phase_voltage_limit;
    config.d_current_pi.max_timestep_s = APP_CONTROL_PERIOD_S * 1.25f;
    config.q_current_pi = config.d_current_pi;

    config.angle_pll.proportional_gain_per_s = 400.0f;
    config.angle_pll.integral_gain_per_s2 = 40000.0f;
    config.angle_pll.maximum_speed_rad_s = BOARD_ANGLE_PLL_MAX_SPEED_RAD_S;
    config.angle_pll.max_timestep_s = 0.010f;
    return motor_foc_init(&app.motor, &config);
}

static bool configure_outer_control(void)
{
    control_pi_config_t speed = {0};

    speed.proportional_gain = BOARD_SPEED_KP_A_PER_RAD_S;
    speed.integral_gain_per_s = BOARD_SPEED_KI_A_PER_RAD;
    speed.anti_windup_gain_per_s = 50.0f;
#if BOARD_PROFILE == BOARD_PROFILE_PRODUCTION_3SHUNT_MT6701
    speed.output_min = -BOARD_MOTOR_PEAK_CURRENT_A;
    speed.output_max = BOARD_MOTOR_PEAK_CURRENT_A;
    speed.integrator_min = -BOARD_MOTOR_PEAK_CURRENT_A;
    speed.integrator_max = BOARD_MOTOR_PEAK_CURRENT_A;
#else
    speed.output_min = -3.0f;
    speed.output_max = 3.0f;
    speed.integrator_min = -3.0f;
    speed.integrator_max = 3.0f;
#endif
    speed.max_timestep_s = APP_SUPERVISOR_PERIOD_S * 1.5f;
    if (!control_pi_init(&app.speed_pi, &speed) ||
        !control_trajectory_init(&app.trajectory, 0.0f, 0.0f)) {
        return false;
    }

    app.trajectory_limits.maximum_velocity_rad_s =
        BOARD_MAX_PROFILE_SPEED_RAD_S;
    app.trajectory_limits.maximum_acceleration_rad_s2 =
        BOARD_MAX_PROFILE_ACCEL_RAD_S2;
    app.trajectory_limits.maximum_deceleration_rad_s2 =
        BOARD_MAX_PROFILE_DECEL_RAD_S2;
    app.trajectory_limits.position_tolerance_rad =
        BOARD_POSITION_TOLERANCE_RAD;
    app.trajectory_limits.velocity_tolerance_rad_s =
        BOARD_VELOCITY_TOLERANCE_RAD_S;
    app.trajectory_limits.max_timestep_s =
        APP_SUPERVISOR_PERIOD_S * 1.5f;

    app.mit_limits.position_min_rad = -100.0f;
    app.mit_limits.position_max_rad = 100.0f;
    app.mit_limits.velocity_limit_rad_s = BOARD_MAX_PROFILE_SPEED_RAD_S;
    app.mit_limits.kp_limit_nm_per_rad = 50.0f;
    app.mit_limits.kd_limit_nm_s_per_rad = 2.0f;
    app.mit_limits.feedforward_torque_limit_nm =
        BOARD_MOTOR_KT_NM_PER_A * BOARD_MOTOR_PEAK_CURRENT_A;
    app.mit_limits.output_torque_limit_nm =
        BOARD_MOTOR_KT_NM_PER_A * BOARD_MOTOR_PEAK_CURRENT_A;
    app.mit_limits.iq_limit_a = BOARD_MOTOR_PEAK_CURRENT_A;
    app.mit_limits.torque_constant_nm_per_a = BOARD_MOTOR_KT_NM_PER_A;
    app.mit_limits.command_timeout_us = BOARD_MIT_STALE_TIMEOUT_US;
    app.mit_command_current_limit_a = BOARD_MOTOR_PEAK_CURRENT_A;
    return control_mit_limits_are_valid(&app.mit_limits);
}

static bool configure_drive(void)
{
    drive_mode_config_t mode_config;
    drive_pds_config_t pds_config;
    drive_safety_config_t safety_config;
    cia402_od_config_t od_config;

    mode_config.minimum_position_q16_16 = float_to_q16(-100.0f);
    mode_config.maximum_position_q16_16 = float_to_q16(100.0f);
    mode_config.maximum_abs_velocity_q16_16 =
        float_to_q16(BOARD_MAX_PROFILE_SPEED_RAD_S);
    if (!drive_mode_init(&app.mode_manager, &mode_config)) {
        return false;
    }

    pds_config.quick_stop_policy = DRIVE_PDS_QS_DISABLE_VOLTAGE;
    drive_pds_init(&app.pds, &pds_config);

    safety_config = drive_safety_default_config();
    safety_config.command_soft_timeout_us = BOARD_MIT_STALE_TIMEOUT_US;
    safety_config.command_hard_timeout_us = BOARD_MIT_DISABLE_TIMEOUT_US;
    if (!drive_safety_init(&app.safety, &safety_config)) {
        return false;
    }

    memset(&od_config, 0, sizeof(od_config));
    od_config.minimum_position_q16_16 = mode_config.minimum_position_q16_16;
    od_config.maximum_position_q16_16 = mode_config.maximum_position_q16_16;
    od_config.maximum_abs_velocity_q16_16 =
        mode_config.maximum_abs_velocity_q16_16;
    od_config.maximum_abs_mit_torque_q16_16 =
        float_to_q16(BOARD_MOTOR_KT_NM_PER_A *
                     BOARD_MOTOR_PEAK_CURRENT_A);
    od_config.maximum_abs_target_torque_per_mille = 1000;
    od_config.maximum_profile_velocity_q16_16 =
        (uint32_t)float_to_q16(BOARD_MAX_PROFILE_SPEED_RAD_S);
    od_config.maximum_profile_acceleration_q16_16 =
        (uint32_t)float_to_q16(BOARD_MAX_PROFILE_ACCEL_RAD_S2);
    od_config.maximum_profile_deceleration_q16_16 =
        (uint32_t)float_to_q16(BOARD_MAX_PROFILE_DECEL_RAD_S2);
    od_config.maximum_mit_kp_uq8_8 = float_to_uq8(50.0f);
    od_config.maximum_mit_kd_uq8_8 = float_to_uq8(2.0f);
    od_config.maximum_mit_current_uq8_8 =
        float_to_uq8(BOARD_MOTOR_PEAK_CURRENT_A);
    od_config.support_pp = true;
    od_config.support_pv = true;
#if BOARD_PROFILE == BOARD_PROFILE_PRODUCTION_3SHUNT_MT6701
    od_config.support_mit = true;
#else
    od_config.support_mit = false;
#endif
    return cia402_od_init(&app.od, &od_config);
}

static bool configure_transport(void)
{
    app.vendor_command_limits.minimum_position_q16_16 = float_to_q16(-100.0f);
    app.vendor_command_limits.maximum_position_q16_16 = float_to_q16(100.0f);
    app.vendor_command_limits.maximum_abs_velocity_q16_16 =
        float_to_q16(BOARD_MAX_PROFILE_SPEED_RAD_S);
    app.vendor_command_limits.maximum_abs_torque_q16_16 =
        float_to_q16(BOARD_MOTOR_KT_NM_PER_A *
                     BOARD_MOTOR_PEAK_CURRENT_A);
    app.vendor_command_limits.maximum_kp_uq8_8 = float_to_uq8(50.0f);
    app.vendor_command_limits.maximum_kd_uq8_8 = float_to_uq8(2.0f);
    app.vendor_command_limits.maximum_current_uq8_8 =
        float_to_uq8(BOARD_MOTOR_PEAK_CURRENT_A);

    app.vendor_feedback_limits.minimum_position_q16_16 = float_to_q16(-100.0f);
    app.vendor_feedback_limits.maximum_position_q16_16 = float_to_q16(100.0f);
    app.vendor_feedback_limits.maximum_abs_velocity_q16_16 =
        float_to_q16(BOARD_MAX_MECHANICAL_SPEED_RAD_S);
    app.vendor_feedback_limits.maximum_abs_torque_q16_16 =
        float_to_q16(BOARD_MOTOR_KT_NM_PER_A *
                     BOARD_MOTOR_PEAK_CURRENT_A);
    app.vendor_feedback_limits.maximum_abs_iq_q16_16 =
        float_to_q16(BOARD_MOTOR_PEAK_CURRENT_A * 1.2f);
    app.vendor_feedback_limits.maximum_bus_voltage_uq16_16 =
        float_to_uq16(BOARD_BUS_OVERVOLTAGE_V * 1.2f);

    return canfd_transport_init(&hfdcan1,
                                BOARD_MIT_COMMAND_EXT_ID,
                                BOARD_MIT_FEEDBACK_EXT_ID,
                                &app.vendor_command_limits);
}

static bool initialize_encoder_driver(void)
{
#if BOARD_PROFILE == BOARD_PROFILE_PRODUCTION_3SHUNT_MT6701
    return encoder_mt6701_init(&app.mt6701,
                               &hspi3,
                               ENCODER_CS_GPIO_Port,
                               ENCODER_CS_Pin);
#else
    return encoder_as5600_init(&app.as5600, &hi2c1, true);
#endif
}

void app_init(void)
{
    memset(&app, 0, sizeof(app));
    app.boot_timestamp_us = bsp_micros();
    app.last_canopen_background_us = app.boot_timestamp_us;
    if (!configure_motor() || !configure_outer_control() ||
        !configure_drive() || !configure_transport()) {
        Error_Handler();
    }
    app.encoder_driver_initialized = initialize_encoder_driver();
#if BOARD_PROFILE == BOARD_PROFILE_PRODUCTION_3SHUNT_MT6701
    if (app.encoder_driver_initialized) {
        (void)encoder_mt6701_start_dma(&app.mt6701);
    }
#endif
#if BOARD_PROFILE == BOARD_PROFILE_LAB_SIMPLEFOC_AS5600
    /* Lab carrier has no bus divider; the operator-supplied nominal value is used. */
    motor_foc_set_bus_voltage(&app.motor, BOARD_BUS_NOMINAL_V, true);
#endif
    app.initialized = true;
    if (!canopen_classic_init(&hfdcan1,
                              BOARD_CANOPEN_NODE_ID,
                              (uint16_t)(BOARD_CAN_NOMINAL_BITRATE / 1000UL))) {
        app.initialized = false;
        Error_Handler();
    }
}

static void accept_vendor_command(const motor_vendor_command_t *command)
{
    const motor_foc_feedback_t *feedback;
    bool zero_position_edge;
    bool zero_position_safe;
    uint32_t primask;
    uint16_t controlword;

    if ((command == NULL) ||
        (command->mode != (int8_t)DRIVE_MODE_MIT)) {
        latch_fault_event(DRIVE_FAULT_PROTOCOL);
        return;
    }
#if BOARD_PROFILE != BOARD_PROFILE_PRODUCTION_3SHUNT_MT6701
    latch_fault_event(DRIVE_FAULT_PROTOCOL);
    return;
#endif

    zero_position_edge =
        ((command->flags & MOTOR_VENDOR_CMD_ZERO_POSITION) != 0U) &&
        ((app.previous_vendor_flags & MOTOR_VENDOR_CMD_ZERO_POSITION) == 0U);
    /* Record even a rejected edge.  A sender must first release and then
     * deliberately re-assert ZERO_POSITION after the drive is safe. */
    app.previous_vendor_flags = command->flags;
    if (zero_position_edge) {
        primask = critical_enter();
        feedback = motor_foc_feedback(&app.motor);
        zero_position_safe =
            !bsp_gate_is_armed() &&
            (app.pds_output.state != DRIVE_PDS_OPERATION_ENABLED) &&
            (app.pds_output.state != DRIVE_PDS_QUICK_STOP_ACTIVE) &&
            (feedback != NULL) &&
            isfinite(feedback->mechanical_velocity_rad_s) &&
            (fabsf(feedback->mechanical_velocity_rad_s) <=
             BOARD_VELOCITY_TOLERANCE_RAD_S);
        if (zero_position_safe) {
            encoder_tracker_zero_at_current(&app.position_tracker);
        }
        critical_exit(primask);
        if (!zero_position_safe) {
            latch_fault_event(DRIVE_FAULT_PROTOCOL);
            return;
        }
    }

    controlword = ((command->flags & MOTOR_VENDOR_CMD_ENABLE) != 0U)
                      ? UINT16_C(0x000F)
                      : UINT16_C(0x0006);
    if ((command->flags & MOTOR_VENDOR_CMD_QUICK_STOP) != 0U) {
        controlword &= (uint16_t)~DRIVE_PDS_CW_QUICK_STOP;
    }
    if ((command->flags & MOTOR_VENDOR_CMD_CLEAR_FAULT) != 0U) {
        controlword |= DRIVE_PDS_CW_FAULT_RESET;
    }
    app.od.image.controlword = controlword;
    app.od.image.mode_command = (int8_t)DRIVE_MODE_MIT;
    app.od.image.mit_position_q16_16 = command->position_q16_16;
    app.od.image.mit_velocity_q16_16 = command->velocity_q16_16;
    app.od.image.mit_torque_q16_16 = command->torque_q16_16;
    app.od.image.mit_kp_uq8_8 = command->kp_uq8_8;
    app.od.image.mit_kd_uq8_8 = command->kd_uq8_8;
    app.od.image.mit_current_limit_uq8_8 = command->current_limit_uq8_8;
    app.od.image.mit_sequence = command->sequence;

    app.mit_command.position_rad = q16_to_float(command->position_q16_16);
    app.mit_command.velocity_rad_s = q16_to_float(command->velocity_q16_16);
    app.mit_command.feedforward_torque_nm =
        q16_to_float(command->torque_q16_16);
    app.mit_command.kp_nm_per_rad =
        (float)command->kp_uq8_8 / APP_UQ8_SCALE;
    app.mit_command.kd_nm_s_per_rad =
        (float)command->kd_uq8_8 / APP_UQ8_SCALE;
    app.mit_command.timestamp_us = command->local_rx_timestamp_us;
    app.mit_command_current_limit_a =
        fminf((float)command->current_limit_uq8_8 / APP_UQ8_SCALE,
              BOARD_MOTOR_PEAK_CURRENT_A);
    app.last_command_sequence = command->sequence;
    app.last_od_mit_sequence = command->sequence;
    drive_safety_note_valid_command(&app.safety,
                                    command->local_rx_timestamp_us);
}

static void consume_external_inputs(uint32_t now_us)
{
    motor_vendor_command_t command;

    if (canfd_transport_take_command(&command)) {
        accept_vendor_command(&command);
    }
    if (app.pending_watchdog_reload) {
        drive_safety_note_valid_command(&app.safety,
                                        app.pending_watchdog_timestamp_us);
        app.pending_watchdog_reload = false;
    }
    if (app.pending_encoder_valid) {
        const encoder_sample_t sample = app.pending_encoder;
        app.pending_encoder_valid = false;
        const uint32_t primask = critical_enter();
        if (sample.valid) {
            (void)encoder_tracker_update(&app.position_tracker,
                                         sample.raw_count,
                                         sample.counts_per_turn);
        }
        (void)motor_foc_update_encoder(&app.motor,
                                       sample.mechanical_angle_rad,
                                       sample.timestamp_us,
                                       sample.health_flags,
                                       sample.valid);
        critical_exit(primask);
    }

    if ((app.od.image.mode_command == (int8_t)DRIVE_MODE_MIT) &&
        (app.od.image.mit_sequence != app.last_od_mit_sequence)) {
        app.last_od_mit_sequence = app.od.image.mit_sequence;
        app.mit_command.position_rad =
            q16_to_float(app.od.image.mit_position_q16_16);
        app.mit_command.velocity_rad_s =
            q16_to_float(app.od.image.mit_velocity_q16_16);
        app.mit_command.feedforward_torque_nm =
            q16_to_float(app.od.image.mit_torque_q16_16);
        app.mit_command.kp_nm_per_rad =
            (float)app.od.image.mit_kp_uq8_8 / APP_UQ8_SCALE;
        app.mit_command.kd_nm_s_per_rad =
            (float)app.od.image.mit_kd_uq8_8 / APP_UQ8_SCALE;
        app.mit_command.timestamp_us = now_us;
        app.mit_command_current_limit_a =
            fminf((float)app.od.image.mit_current_limit_uq8_8 /
                      APP_UQ8_SCALE,
                  BOARD_MOTOR_PEAK_CURRENT_A);
        drive_safety_note_valid_command(&app.safety, now_us);
    }
}

static uint32_t collect_faults(const motor_foc_feedback_t *feedback,
                               uint32_t now_us,
                               const drive_mode_output_t *mode)
{
    uint32_t faults;
    const uint32_t uptime = now_us - app.boot_timestamp_us;
    const canfd_transport_stats_t can_stats = canfd_transport_stats();

    {
        const uint32_t primask = critical_enter();
        faults = app.fast_fault_events;
        app.fast_fault_events = 0U;
        critical_exit(primask);
    }
    if (bsp_gate_fault_active() || bsp_gate_break_latched()) {
        faults |= DRIVE_FAULT_GATE_DRIVER;
    }
    if (app.pds_output.gate_interlock_fault) {
        faults |= DRIVE_FAULT_GATE_INTERLOCK;
    }
    if (feedback->current_limit_exceeded) {
        faults |= DRIVE_FAULT_PHASE_OVERCURRENT;
    }
    if (feedback->control_math_fault) {
        faults |= DRIVE_FAULT_CONTROL_DEADLINE;
    }
    if ((uptime > APP_STARTUP_SENSOR_GRACE_US) &&
        !feedback->encoder_valid) {
        faults |= DRIVE_FAULT_POSITION_FEEDBACK;
    }
    if (fabsf(feedback->mechanical_velocity_rad_s) >
        BOARD_MAX_MECHANICAL_SPEED_RAD_S) {
        faults |= DRIVE_FAULT_OVERSPEED;
    }
#if BOARD_PROFILE == BOARD_PROFILE_PRODUCTION_3SHUNT_MT6701
    if (feedback->calibration_complete &&
        !feedback->current_feedback_valid) {
        faults |= DRIVE_FAULT_CURRENT_FEEDBACK;
    }
    if ((uptime > APP_STARTUP_SENSOR_GRACE_US) &&
        !feedback->bus_voltage_valid) {
        faults |= DRIVE_FAULT_DC_UNDERVOLTAGE;
    }
#endif
    if (feedback->bus_voltage_valid) {
        if (feedback->bus_voltage_v > BOARD_BUS_OVERVOLTAGE_V) {
            faults |= DRIVE_FAULT_DC_OVERVOLTAGE;
        } else if (feedback->bus_voltage_v < BOARD_BUS_UNDERVOLTAGE_V) {
            faults |= DRIVE_FAULT_DC_UNDERVOLTAGE;
        }
    }
    if (can_stats.bus_off) {
        faults |= DRIVE_FAULT_CAN_BUS_OFF;
    }
    if (mode->mode_change_rejected || mode->pp_setpoint_rejected) {
        faults |= DRIVE_FAULT_PROTOCOL;
    }
    return faults;
}

static void update_dynamic_trajectory_limits(void)
{
    const float profile_velocity =
        q16_to_float((int32_t)app.od.image.profile_velocity_q16_16);
    const float profile_acceleration =
        q16_to_float((int32_t)app.od.image.profile_acceleration_q16_16);
    const float profile_deceleration =
        q16_to_float((int32_t)app.od.image.profile_deceleration_q16_16);

    app.trajectory_limits.maximum_velocity_rad_s =
        clampf_local(profile_velocity,
                     0.01f,
                     BOARD_MAX_PROFILE_SPEED_RAD_S);
    app.trajectory_limits.maximum_acceleration_rad_s2 =
        clampf_local(profile_acceleration,
                     0.1f,
                     BOARD_MAX_PROFILE_ACCEL_RAD_S2);
    app.trajectory_limits.maximum_deceleration_rad_s2 =
        clampf_local(profile_deceleration,
                     0.1f,
                     BOARD_MAX_PROFILE_DECEL_RAD_S2);
}

static float speed_loop(float target_velocity,
                        float actual_velocity,
                        uint16_t torque_limit_q15)
{
    control_pi_result_t result;
    const float derate =
        (float)torque_limit_q15 / (float)DRIVE_SAFETY_FULL_TORQUE_Q15;

    target_velocity = clampf_local(target_velocity,
                                   -BOARD_MAX_PROFILE_SPEED_RAD_S,
                                   BOARD_MAX_PROFILE_SPEED_RAD_S);
    if (!control_pi_step(&app.speed_pi,
                         target_velocity - actual_velocity,
                         0.0f,
                         APP_SUPERVISOR_PERIOD_S,
                         &result)) {
        latch_fault_event(DRIVE_FAULT_CONTROL_DEADLINE);
        return 0.0f;
    }
    return result.output * derate;
}

static motor_foc_command_t compute_motion_command(
    const motor_foc_feedback_t *feedback,
    float actual_position_rad,
    bool quick_stop)
{
    motor_foc_command_t command = {0};
    float effort = 0.0f;
    bool motion_step_ok = true;

    if (!app.pds_output.torque_enable) {
        if (!control_pi_reset(&app.speed_pi, 0.0f, 0.0f) ||
            !control_trajectory_init(&app.trajectory,
                                     actual_position_rad,
                                     feedback->mechanical_velocity_rad_s)) {
            latch_fault_event(DRIVE_FAULT_CONTROL_DEADLINE);
            bsp_gate_disarm();
        }
        return command;
    }

    command.enable = true;
    update_dynamic_trajectory_limits();
    if (quick_stop) {
        motion_step_ok = control_trajectory_step_pv(
            &app.trajectory,
            &app.trajectory_limits,
            0.0f,
            APP_SUPERVISOR_PERIOD_S);
        if (motion_step_ok) {
            effort = speed_loop(app.trajectory.velocity_rad_s,
                                feedback->mechanical_velocity_rad_s,
                                app.safety_output.torque_limit_q15);
        }
    } else if (app.mode_output.mode_display ==
               (int8_t)DRIVE_MODE_PROFILE_POSITION) {
        float velocity_reference;
        const float target =
            q16_to_float(app.mode_output.position_reference_q16_16);
        motion_step_ok = control_trajectory_step_pp(
            &app.trajectory,
            &app.trajectory_limits,
            target,
            APP_SUPERVISOR_PERIOD_S);
        if (motion_step_ok) {
            velocity_reference = app.trajectory.velocity_rad_s +
                                 BOARD_POSITION_KP_PER_S *
                                     (app.trajectory.position_rad -
                                      actual_position_rad);
            effort = speed_loop(velocity_reference,
                                feedback->mechanical_velocity_rad_s,
                                app.safety_output.torque_limit_q15);
        }
    } else if (app.mode_output.mode_display ==
               (int8_t)DRIVE_MODE_PROFILE_VELOCITY) {
        const float target =
            q16_to_float(app.mode_output.velocity_reference_q16_16);
        motion_step_ok = control_trajectory_step_pv(
            &app.trajectory,
            &app.trajectory_limits,
            target,
            APP_SUPERVISOR_PERIOD_S);
        if (motion_step_ok) {
            effort = speed_loop(app.trajectory.velocity_rad_s,
                                feedback->mechanical_velocity_rad_s,
                                app.safety_output.torque_limit_q15);
        }
    } else if (app.mode_output.mit_control_enabled) {
        control_mit_feedback_t mit_feedback;
        control_mit_output_t mit_output;
        const float safety_limit =
            (float)app.safety_output.torque_limit_q15 /
            (float)DRIVE_SAFETY_FULL_TORQUE_Q15;

        mit_feedback.position_rad = actual_position_rad;
        mit_feedback.velocity_rad_s = feedback->mechanical_velocity_rad_s;
        if (control_mit_compute(&app.mit_limits,
                                &app.mit_command,
                                &mit_feedback,
                                bsp_micros(),
                                &mit_output) == CONTROL_MIT_STATUS_TIMEOUT) {
            effort = 0.0f;
        } else {
            effort = clampf_local(mit_output.iq_command_a,
                                  -app.mit_command_current_limit_a,
                                  app.mit_command_current_limit_a) *
                     safety_limit;
        }
    } else {
        command.enable = false;
    }

    if (!motion_step_ok || !isfinite(effort)) {
        memset(&command, 0, sizeof(command));
        latch_fault_event(DRIVE_FAULT_CONTROL_DEADLINE);
        bsp_gate_disarm();
        return command;
    }

#if BOARD_PROFILE == BOARD_PROFILE_PRODUCTION_3SHUNT_MT6701
    command.iq_reference_a = effort;
#else
    command.lab_q_voltage_v = effort;
#endif
    return command;
}

static void publish_feedback(const motor_foc_feedback_t *feedback,
                             float position_rad,
                             uint32_t now_us)
{
    cia402_od_feedback_t od_feedback;
    motor_vendor_feedback_t frame = {0};
    const float torque_nm =
        feedback->iq_current_a * BOARD_MOTOR_KT_NM_PER_A;

    od_feedback.error_code =
        error_code_from_faults(app.safety_output.effective_faults);
    od_feedback.statusword = app.pds_output.statusword;
    od_feedback.mode_display = app.mode_output.mode_display;
    od_feedback.position_actual_q16_16 = float_to_q16(position_rad);
    od_feedback.velocity_actual_q16_16 =
        float_to_q16(feedback->mechanical_velocity_rad_s);
    od_feedback.torque_actual_per_mille =
        torque_per_mille(feedback->iq_current_a);
    cia402_od_publish_feedback(&app.od, &od_feedback);

    frame.sequence = app.feedback_sequence++;
    frame.command_sequence_echo = app.last_command_sequence;
    frame.local_timestamp_us = now_us;
    frame.position_q16_16 = od_feedback.position_actual_q16_16;
    frame.velocity_q16_16 = od_feedback.velocity_actual_q16_16;
    frame.torque_q16_16 = float_to_q16(torque_nm);
    frame.iq_q16_16 = float_to_q16(feedback->iq_current_a);
    frame.bus_voltage_uq16_16 = float_to_uq16(feedback->bus_voltage_v);
    frame.statusword = app.pds_output.statusword;
    frame.mode_display = app.mode_output.mode_display;
    frame.pds_state = (uint8_t)app.pds_output.state;
    frame.active_faults = app.safety_output.active_faults;
    frame.latched_faults = app.safety_output.latched_faults;
    frame.torque_limit_q15 = app.safety_output.torque_limit_q15;
    if (app.pds_output.torque_enable) {
        frame.flags |= MOTOR_VENDOR_FB_OPERATION_ENABLED;
    }
    if (app.safety_output.warning) {
        frame.flags |= MOTOR_VENDOR_FB_WARNING;
    }
    if (app.trajectory.target_reached) {
        frame.flags |= MOTOR_VENDOR_FB_TARGET_REACHED;
    }
    if (!app.safety_output.command_soft_timeout) {
        frame.flags |= MOTOR_VENDOR_FB_COMMAND_FRESH;
    }
    if (app.safety_output.torque_limit_q15 < DRIVE_SAFETY_FULL_TORQUE_Q15) {
        frame.flags |= MOTOR_VENDOR_FB_DERATED;
    }
    app.feedback_frame = frame;
}

static void supervisor_step(uint32_t now_us)
{
    motor_foc_feedback_t feedback_snapshot;
    const motor_foc_feedback_t *feedback = &feedback_snapshot;
    drive_mode_input_t mode_input = {0};
    drive_pds_input_t pds_input = {0};
    drive_safety_input_t safety_input = {0};
    motor_foc_command_t motor_command;
    uint32_t sensed_faults;
    float position_rad;
    bool reset_edge;
    bool quick_stop_complete;
    bool hardware_ready;

    /* Apply complete RPDOs before taking the command/feedback snapshot used
     * by this 1 kHz control transaction. */
    canopen_classic_process_1ms();
    consume_external_inputs(now_us);
    {
        const uint32_t primask = critical_enter();
        feedback_snapshot = *motor_foc_feedback(&app.motor);
        position_rad = BOARD_ENCODER_DIRECTION *
                       encoder_tracker_position_rad(&app.position_tracker);
        critical_exit(primask);
    }
    app.last_position_rad = position_rad;

    mode_input.requested_mode = app.od.image.mode_command;
    mode_input.mode_instance_epoch =
        app.od.command_sync.mode_instance_epoch;
    mode_input.controlword = app.od.image.controlword;
    mode_input.pp_setpoint_controlword =
        app.od.command_sync.pp_request_controlword;
    mode_input.pp_setpoint_mode = app.od.command_sync.pp_request_mode;
    mode_input.pp_setpoint_mode_instance_epoch =
        app.od.command_sync.pp_request_mode_instance_epoch;
    mode_input.actual_position_q16_16 = float_to_q16(position_rad);
    mode_input.target_position_q16_16 =
        app.od.command_sync.pp_request_target_position_q16_16;
    mode_input.target_position_epoch =
        app.od.command_sync.pp_request_target_position_epoch;
    mode_input.pp_request_epoch = app.od.command_sync.pp_request_epoch;
    mode_input.target_velocity_q16_16 = app.od.image.target_velocity_q16_16;
    mode_input.target_velocity_epoch =
        app.od.command_sync.target_velocity_epoch;
    mode_input.target_velocity_mode =
        app.od.command_sync.target_velocity_mode;
    mode_input.target_velocity_mode_instance_epoch =
        app.od.command_sync.target_velocity_mode_instance_epoch;
    mode_input.torque_active = app.pds_output.torque_enable;
    mode_input.mode_change_inhibited =
        (app.pds_output.state == DRIVE_PDS_OPERATION_ENABLED) ||
        app.pds_output.torque_enable;
    mode_input.current_feedback_valid = feedback->current_feedback_valid;
    mode_input.trajectory_target_reached = app.trajectory.target_reached;
    mode_input.trajectory_internal_limit = false;
    app.mode_output = drive_mode_step(&app.mode_manager, &mode_input);

    reset_edge =
        ((app.od.image.controlword & DRIVE_PDS_CW_FAULT_RESET) != 0U) &&
        ((app.previous_safety_controlword & DRIVE_PDS_CW_FAULT_RESET) == 0U);
    app.previous_safety_controlword = app.od.image.controlword;
    if (reset_edge && !bsp_gate_fault_active() && !bsp_gate_is_armed() &&
        (fabsf(feedback->mechanical_velocity_rad_s) <
         BOARD_VELOCITY_TOLERANCE_RAD_S)) {
        (void)bsp_gate_clear_break_latch();
        (void)canfd_transport_clear_bus_off_latch();
    }
    sensed_faults = collect_faults(feedback, now_us, &app.mode_output);
    if (reset_edge && !bsp_gate_fault_active() && !bsp_gate_is_armed()) {
        /* The PDS owns this internal latch.  Suppress its old published value
         * for one physically-safe reset transaction so both the safety latch
         * and PDS latch can clear together; otherwise each would keep the
         * other permanently in Fault. */
        sensed_faults &= ~DRIVE_FAULT_GATE_INTERLOCK;
    }
    safety_input.now_us = now_us;
    safety_input.sensed_faults = sensed_faults;
    safety_input.thermal_torque_limit_q15 = DRIVE_SAFETY_FULL_TORQUE_Q15;
    safety_input.supply_torque_limit_q15 = DRIVE_SAFETY_FULL_TORQUE_Q15;
    safety_input.command_watchdog_required =
        app.mode_output.mode_display == (int8_t)DRIVE_MODE_MIT &&
        (app.pds_output.state == DRIVE_PDS_OPERATION_ENABLED ||
         app.pds_output.state == DRIVE_PDS_QUICK_STOP_ACTIVE);
    safety_input.reset_rising_edge = reset_edge;
    safety_input.safe_to_reset =
        (sensed_faults == 0U) &&
        (fabsf(feedback->mechanical_velocity_rad_s) <
         BOARD_VELOCITY_TOLERANCE_RAD_S) &&
        !bsp_gate_fault_active();
    app.safety_output = drive_safety_step(&app.safety, &safety_input);

    quick_stop_complete =
        (fabsf(feedback->mechanical_velocity_rad_s) <
         BOARD_VELOCITY_TOLERANCE_RAD_S) &&
        (fabsf(feedback->iq_current_a) < 0.05f);
#if BOARD_PROFILE == BOARD_PROFILE_PRODUCTION_3SHUNT_MT6701
    hardware_ready = feedback->encoder_valid &&
                     feedback->current_feedback_valid &&
                     feedback->bus_voltage_valid &&
                     (BOARD_ELECTRICAL_ZERO_VALID != 0);
#else
    hardware_ready = feedback->encoder_valid &&
                     feedback->bus_voltage_valid &&
                     (BOARD_ELECTRICAL_ZERO_VALID != 0);
#endif
    hardware_ready = hardware_ready && !bsp_gate_fault_active() &&
                     !feedback->control_math_fault;

    pds_input.controlword = app.od.image.controlword;
    pds_input.mode_status_bits = app.mode_output.statusword_bits;
    pds_input.hardware_ready = hardware_ready;
    pds_input.external_fault_active =
        app.safety_output.effective_faults != 0U;
    pds_input.fault_reaction_complete =
        quick_stop_complete || app.safety_output.immediate_gate_off;
    pds_input.fault_reaction_requires_immediate_gate_off =
        app.safety_output.immediate_gate_off;
    pds_input.quick_stop_complete = quick_stop_complete;
    pds_input.gate_enable_allowed = app.safety_output.gate_enable_allowed;
    pds_input.gate_feedback_enabled = bsp_gate_is_armed();
    pds_input.operation_enable_allowed =
        app.mode_output.operation_enable_allowed;
    pds_input.warning = app.safety_output.warning;
    pds_input.remote = true;
    app.pds_output = drive_pds_step(&app.pds, &pds_input);

    if (app.pds_output.gate_enable_request &&
        app.safety_output.gate_enable_allowed) {
        bsp_gate_arm();
    } else {
        bsp_gate_disarm();
    }

    motor_command = compute_motion_command(
        feedback,
        position_rad,
        app.pds_output.quick_stop_request ||
            app.safety_output.quick_stop_request ||
            app.mode_output.halt_active);
    {
        const uint32_t primask = critical_enter();
        motor_foc_set_command(&app.motor, &motor_command);
        critical_exit(primask);
    }
    publish_feedback(feedback, position_rad, now_us);
    canopen_classic_report_drive_faults(
        app.safety_output.effective_faults,
        error_code_from_faults(app.safety_output.effective_faults));
}

void app_current_sample_irq(uint16_t phase_a_raw,
                            uint16_t phase_b_raw,
                            uint32_t timestamp_us)
{
    const uint32_t now_us = timestamp_us;
    const uint32_t start_cycles = DWT->CYCCNT;
    const motor_foc_feedback_t *feedback;

    if (!app.initialized) {
        bsp_gate_disarm();
        return;
    }
#if BOARD_PROFILE == BOARD_PROFILE_PRODUCTION_3SHUNT_MT6701
    {
        encoder_sample_t encoder_sample;
        if (encoder_mt6701_take_dma_sample(&app.mt6701, &encoder_sample)) {
            if (encoder_sample.valid) {
                (void)encoder_tracker_update(&app.position_tracker,
                                             encoder_sample.raw_count,
                                             encoder_sample.counts_per_turn);
            }
            (void)motor_foc_update_encoder(&app.motor,
                                           encoder_sample.mechanical_angle_rad,
                                           encoder_sample.timestamp_us,
                                           encoder_sample.health_flags,
                                           encoder_sample.valid);
        }
    }
#endif
    motor_foc_fast_step(&app.motor, phase_a_raw, phase_b_raw, now_us);
    feedback = motor_foc_feedback(&app.motor);
    bsp_pwm_write(feedback->duty_a,
                  feedback->duty_b,
                  feedback->duty_c);
    if (app.motor.command.enable && !feedback->output_enabled) {
        /* The fast loop has already fallen back to a neutral vector.  Remove
         * gate drive in the same carrier interrupt instead of waiting up to
         * one supervisor period for a lost mandatory feedback source. */
        bsp_gate_disarm();
        if (!feedback->encoder_valid) {
            latch_fault_event(DRIVE_FAULT_POSITION_FEEDBACK);
        }
#if BOARD_PROFILE == BOARD_PROFILE_PRODUCTION_3SHUNT_MT6701
        if (feedback->calibration_complete &&
            !feedback->current_feedback_valid) {
            latch_fault_event(DRIVE_FAULT_CURRENT_FEEDBACK);
        }
#endif
        if (!feedback->bus_voltage_valid) {
            latch_fault_event(DRIVE_FAULT_DC_UNDERVOLTAGE);
        }
    }
    if (feedback->current_limit_exceeded || feedback->control_math_fault) {
        bsp_gate_disarm();
        if (feedback->current_limit_exceeded) {
            latch_fault_event(DRIVE_FAULT_PHASE_OVERCURRENT);
        }
        if (feedback->control_math_fault) {
            latch_fault_event(DRIVE_FAULT_CONTROL_DEADLINE);
        }
    }

    ++app.supervisor_divider;
    if (app.supervisor_divider >= APP_SUPERVISOR_DIVIDER) {
        app.supervisor_divider = 0U;
        SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
    }
    if ((DWT->CYCCNT - start_cycles) >= APP_ISR_DEADLINE_CYCLES) {
        latch_fault_event(DRIVE_FAULT_CONTROL_DEADLINE);
        bsp_gate_disarm();
    }
#if BOARD_PROFILE == BOARD_PROFILE_PRODUCTION_3SHUNT_MT6701
    (void)encoder_mt6701_start_dma(&app.mt6701);
#endif
    ++app.current_completion_count;
    app.current_loop_seen = true;
}

void app_supervisor_irq(uint32_t timestamp_us)
{
    const uint32_t start_cycles = DWT->CYCCNT;

    if (!app.initialized) {
        return;
    }
    supervisor_step(timestamp_us);
    if ((DWT->CYCCNT - start_cycles) >=
        (BOARD_SYSCLK_HZ / BOARD_SUPERVISOR_FREQUENCY_HZ)) {
        latch_fault_event(DRIVE_FAULT_CONTROL_DEADLINE);
        bsp_gate_disarm();
    }
    ++app.supervisor_completion_count;
    app.supervisor_loop_seen = true;
}

void app_break_irq(void)
{
    latch_fault_event(DRIVE_FAULT_GATE_DRIVER);
}

void app_adc_sync_fault_irq(void)
{
    /* A dual-ADC completion mismatch invalidates both phase-current samples
     * and means the 20 kHz control deadline was not met deterministically. */
    latch_fault_event(DRIVE_FAULT_CURRENT_FEEDBACK |
                      DRIVE_FAULT_CONTROL_DEADLINE);
}

static void poll_encoder(uint32_t now_us)
{
#if BOARD_PROFILE == BOARD_PROFILE_PRODUCTION_3SHUNT_MT6701
    bool dma_timed_out = false;

    if (app.encoder_driver_initialized) {
        const uint32_t now_cycles = bsp_cycle_counter();
        const uint32_t primask = critical_enter();

        dma_timed_out = encoder_mt6701_dma_timed_out(
            &app.mt6701, now_cycles, APP_ENCODER_DMA_TIMEOUT_CYCLES);
        if (dma_timed_out) {
            /* Establish the drive interlock before IRQs may run again.  A
             * pending PendSV therefore observes the fault and cannot re-arm
             * the bridge while the blocking HAL abort executes below. */
            app.fast_fault_events |= DRIVE_FAULT_POSITION_FEEDBACK;
            bsp_gate_disarm();
        }
        critical_exit(primask);
    }
    if (dma_timed_out) {
        (void)encoder_mt6701_recover_dma_timeout(
            &app.mt6701,
            bsp_cycle_counter(),
            APP_ENCODER_DMA_TIMEOUT_CYCLES);
    }
    if (!app.encoder_driver_initialized &&
        ((now_us - app.last_encoder_poll_us) >= 100000UL)) {
        app.last_encoder_poll_us = now_us;
        app.encoder_driver_initialized = initialize_encoder_driver();
        if (app.encoder_driver_initialized) {
            (void)encoder_mt6701_start_dma(&app.mt6701);
        }
    }
#else
    encoder_sample_t sample = {0};
    bool valid;

    if ((now_us - app.last_encoder_poll_us) <
        APP_ENCODER_SAMPLE_PERIOD_US_LAB) {
        return;
    }
    app.last_encoder_poll_us = now_us;
    if (!app.encoder_driver_initialized) {
        app.encoder_driver_initialized = initialize_encoder_driver();
        if (!app.encoder_driver_initialized) {
            return;
        }
    }
    valid = encoder_as5600_read(&app.as5600, now_us, &sample);
    sample.valid = valid;
    {
        const uint32_t primask = critical_enter();
        app.pending_encoder = sample;
        app.pending_encoder_valid = true;
        critical_exit(primask);
    }
#endif
}

static void poll_bus_voltage(uint32_t now_us)
{
#if BOARD_PROFILE == BOARD_PROFILE_PRODUCTION_3SHUNT_MT6701
    float bus_voltage = 0.0f;
    bool valid;

    if ((now_us - app.last_bus_sample_us) < APP_BUS_SAMPLE_PERIOD_US) {
        return;
    }
    app.last_bus_sample_us = now_us;
    valid = bsp_bus_voltage_read(&bus_voltage);
    {
        const uint32_t primask = critical_enter();
        motor_foc_set_bus_voltage(&app.motor, bus_voltage, valid);
        critical_exit(primask);
    }
#else
    (void)now_us;
#endif
}

static void transmit_feedback(uint32_t now_us)
{
    motor_vendor_feedback_t frame;
    uint32_t primask;

    if ((now_us - app.last_feedback_tx_us) < APP_FEEDBACK_PERIOD_US) {
        return;
    }
    app.last_feedback_tx_us = now_us;
    primask = critical_enter();
    frame = app.feedback_frame;
    critical_exit(primask);
    (void)canfd_transport_send_feedback(&frame,
                                        &app.vendor_feedback_limits);
}

static bool realtime_loops_are_healthy(uint32_t now_us, uint32_t now_cycles)
{
    uint32_t current_completion_count;
    uint32_t supervisor_completion_count;
    bool current_fresh;
    bool supervisor_fresh;
    bool tim2_fresh;
    bool dwt_fresh;
    uint32_t primask;

    if (!app.realtime_monitor_started) {
        return false;
    }

    primask = critical_enter();
    current_completion_count = app.current_completion_count;
    supervisor_completion_count = app.supervisor_completion_count;
    critical_exit(primask);

    if (current_completion_count != app.monitor_current_completion_count) {
        app.monitor_current_completion_count = current_completion_count;
        app.monitor_current_progress_cycles = now_cycles;
    }
    if (supervisor_completion_count !=
        app.monitor_supervisor_completion_count) {
        app.monitor_supervisor_completion_count = supervisor_completion_count;
        app.monitor_supervisor_progress_cycles = now_cycles;
    }
    if (now_us != app.monitor_tim2_us) {
        app.monitor_tim2_us = now_us;
        app.monitor_tim2_progress_cycles = now_cycles;
    }
    if (now_cycles != app.monitor_dwt_cycles) {
        app.monitor_dwt_cycles = now_cycles;
        app.monitor_dwt_progress_us = now_us;
    }

    current_fresh = app.current_loop_seen &&
                    ((uint32_t)(now_cycles -
                                app.monitor_current_progress_cycles) <=
                     APP_CURRENT_LOOP_STALE_CYCLES);
    supervisor_fresh = app.supervisor_loop_seen &&
                       ((uint32_t)(now_cycles -
                                   app.monitor_supervisor_progress_cycles) <=
                        APP_SUPERVISOR_LOOP_STALE_CYCLES);
    tim2_fresh =
        ((uint32_t)(now_cycles - app.monitor_tim2_progress_cycles) <=
         APP_TIM2_STALE_CYCLES);
    dwt_fresh =
        ((uint32_t)(now_us - app.monitor_dwt_progress_us) <=
         APP_TIM2_STALE_US);

    if ((!app.current_loop_seen || !app.supervisor_loop_seen) &&
        ((uint32_t)(now_cycles - app.realtime_monitor_start_cycles) <=
         APP_SUPERVISOR_LOOP_STALE_CYCLES)) {
        /* bsp_start_realtime() runs after app_init(); allow the first carrier
         * and supervisor ticks to arrive, but do not feed IWDG yet. */
        return false;
    }
    if (!current_fresh || !supervisor_fresh || !tim2_fresh || !dwt_fresh) {
        /* This path also catches a stopped ADC trigger/PendSV source, which
         * cannot self-report a deadline fault from the missing interrupt.
         * DWT and TIM2 cross-monitor one another, so a frozen time base cannot
         * leave MIT commands, sensor ages, or ISR tickets falsely fresh. */
        bsp_gate_disarm();
        latch_fault_event(DRIVE_FAULT_CONTROL_DEADLINE);
        return false;
    }
    return true;
}

void app_background(void)
{
    const uint32_t now_us = bsp_micros();
    const uint32_t now_cycles = bsp_cycle_counter();
    const uint32_t canopen_elapsed_us =
        now_us - app.last_canopen_background_us;
    canopen_classic_status_t canopen_status;
    uint32_t health_now_us;
    uint32_t health_now_cycles;

    if (!app.initialized) {
        return;
    }
    if (!app.realtime_monitor_started) {
        /* main() calls bsp_start_realtime() immediately before entering this
         * loop, so this timestamp excludes CubeMX/peripheral/CANopen init
         * latency from the missing-interrupt grace interval. */
        uint32_t primask = critical_enter();
        app.monitor_current_completion_count = app.current_completion_count;
        app.monitor_supervisor_completion_count =
            app.supervisor_completion_count;
        critical_exit(primask);
        app.realtime_monitor_start_cycles = now_cycles;
        app.monitor_current_progress_cycles = now_cycles;
        app.monitor_supervisor_progress_cycles = now_cycles;
        app.monitor_tim2_progress_cycles = now_cycles;
        app.monitor_tim2_us = now_us;
        app.monitor_dwt_cycles = now_cycles;
        app.monitor_dwt_progress_us = now_us;
        app.realtime_monitor_started = true;
    }
    app.last_canopen_background_us = now_us;
    canopen_classic_background(canopen_elapsed_us);
    canopen_status = canopen_classic_status();
    if (canopen_status.application_reset_requested) {
        /* NMT reset-node is honored only after a deterministic hard gate-off.
         * Communication reset is handled inside the CANopen adapter. */
        bsp_fatal_shutdown();
        __DSB();
        NVIC_SystemReset();
    }
    poll_encoder(now_us);
    poll_bus_voltage(now_us);
    transmit_feedback(now_us);
    health_now_us = bsp_micros();
    health_now_cycles = bsp_cycle_counter();
    if (realtime_loops_are_healthy(health_now_us, health_now_cycles)) {
        bsp_watchdog_refresh();
    }
}

cia402_od_result_t app_cia402_od_write(uint16_t index,
                                      uint8_t subindex,
                                      const uint8_t *data,
                                      size_t data_length)
{
    cia402_od_result_t result;
    uint32_t primask;
    uint16_t mit_sequence_before;
    bool mode_change_inhibited;

    if (!app.initialized) {
        return CIA402_OD_STATE_CONFLICT;
    }
    primask = critical_enter();
    mit_sequence_before = app.od.image.mit_sequence;
    mode_change_inhibited =
        (app.pds_output.state == DRIVE_PDS_OPERATION_ENABLED) ||
        app.pds_output.torque_enable;
    result = cia402_od_write(&app.od,
                             index,
                             subindex,
                             data,
                             data_length,
                             mode_change_inhibited);
    /* Subindex 7 is the commit marker for the complete MIT record.  Profile,
     * controlword and partial MIT writes must never keep stale torque alive. */
    if ((result == CIA402_OD_OK) &&
        (index == CIA402_OD_INDEX_MIT_COMMAND) &&
        (subindex == CIA402_OD_MIT_SUB_SEQUENCE) &&
        (app.od.image.mit_sequence != mit_sequence_before)) {
        app.pending_watchdog_timestamp_us = bsp_micros();
        app.pending_watchdog_reload = true;
    }
    critical_exit(primask);
    return result;
}

cia402_od_result_t app_cia402_od_read(uint16_t index,
                                     uint8_t subindex,
                                     uint8_t *data,
                                     size_t data_capacity,
                                     size_t *data_length)
{
    cia402_od_result_t result;
    const uint32_t primask = critical_enter();
    result = cia402_od_read(&app.od,
                            index,
                            subindex,
                            data,
                            data_capacity,
                            data_length);
    critical_exit(primask);
    return result;
}

cia402_od_result_t app_cia402_od_write_target_batch(
    const uint8_t *target_position_data,
    size_t target_position_length,
    const uint8_t *target_velocity_data,
    size_t target_velocity_length)
{
    cia402_od_result_t result;
    uint32_t primask;

    if (!app.initialized) {
        return CIA402_OD_STATE_CONFLICT;
    }
    primask = critical_enter();
    result = cia402_od_write_target_batch(&app.od,
                                           target_position_data,
                                           target_position_length,
                                           target_velocity_data,
                                           target_velocity_length);
    critical_exit(primask);
    return result;
}

void app_note_canopen_command(uint32_t local_rx_timestamp_us)
{
    const uint32_t primask = critical_enter();
    app.pending_watchdog_timestamp_us = local_rx_timestamp_us;
    app.pending_watchdog_reload = true;
    critical_exit(primask);
}

void app_note_canopen_protocol_fault(void)
{
    latch_fault_event(DRIVE_FAULT_PROTOCOL);
}
