#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include <stdint.h>

#define BOARD_SYSCLK_HZ                 170000000UL
#define BOARD_PWM_FREQUENCY_HZ              20000UL
#define BOARD_CONTROL_FREQUENCY_HZ          20000UL
#define BOARD_SUPERVISOR_FREQUENCY_HZ        1000UL
#define BOARD_PWM_PERIOD_TICKS               4250UL
#define BOARD_PWM_MIN_PULSE_TICKS             340UL
#define BOARD_PWM_DEADTIME_TICKS               34UL

#define BOARD_CAN_NOMINAL_BITRATE          500000UL
#define BOARD_CAN_DATA_BITRATE            2000000UL

#ifndef BOARD_CANOPEN_NODE_ID
#define BOARD_CANOPEN_NODE_ID 1U
#endif
#if (BOARD_CANOPEN_NODE_ID < 1U) || (BOARD_CANOPEN_NODE_ID > 127U)
#error "BOARD_CANOPEN_NODE_ID must be in the CANopen range 1..127"
#endif
#if (BOARD_CONTROL_FREQUENCY_HZ % BOARD_SUPERVISOR_FREQUENCY_HZ) != 0U
#error "Control frequency must be an integer multiple of supervisor frequency"
#endif
#if BOARD_PWM_PERIOD_TICKS != \
    (BOARD_SYSCLK_HZ / (2UL * BOARD_PWM_FREQUENCY_HZ))
#error "BOARD_PWM_PERIOD_TICKS does not match center-aligned PWM frequency"
#endif

/* Extended identifiers reserved for the non-CiA-1301 MIT fast channel. */
#define BOARD_MIT_COMMAND_EXT_ID  (0x18FF1000UL | BOARD_CANOPEN_NODE_ID)
#define BOARD_MIT_FEEDBACK_EXT_ID (0x18FF1800UL | BOARD_CANOPEN_NODE_ID)

/* Production CAN-FD timing requires the specified 8 MHz crystal on PF0/PF1. */
#ifndef BOARD_USE_8MHZ_HSE
#define BOARD_USE_8MHZ_HSE 1
#endif

/* These IDs must be preprocessor macros because they select compiled hardware
 * paths with #if.  Enum constants are not visible to the preprocessor. */
#define BOARD_PROFILE_PRODUCTION_3SHUNT_MT6701 0U
#define BOARD_PROFILE_LAB_SIMPLEFOC_AS5600     1U
typedef uint8_t board_profile_t;

#ifndef BOARD_PROFILE
#define BOARD_PROFILE BOARD_PROFILE_PRODUCTION_3SHUNT_MT6701
#endif
#if (BOARD_PROFILE != BOARD_PROFILE_PRODUCTION_3SHUNT_MT6701) && \
    (BOARD_PROFILE != BOARD_PROFILE_LAB_SIMPLEFOC_AS5600)
#error "BOARD_PROFILE selects an unsupported hardware profile"
#endif

/* The current bench has no CAN transceiver. Keep the complete CAN stack in
 * the production image, but do not initialize pins, message RAM, interrupts,
 * or protocol watchdogs in the AS5600 lab image. */
#ifndef BOARD_CAN_ENABLED
#if BOARD_PROFILE == BOARD_PROFILE_PRODUCTION_3SHUNT_MT6701
#define BOARD_CAN_ENABLED 1
#else
#define BOARD_CAN_ENABLED 0
#endif
#endif

#if BOARD_PROFILE == BOARD_PROFILE_PRODUCTION_3SHUNT_MT6701
#define BOARD_GATE_DRIVER_PWM_INPUTS 6U
#define BOARD_HARDWARE_I2C1_ENABLED   1U
#define BOARD_I2C1_USE_HSI            0U
#define BOARD_I2C1_TIMING             0x00F02B86UL
#else
#define BOARD_GATE_DRIVER_PWM_INPUTS 3U
#define BOARD_HARDWARE_I2C1_ENABLED   0U
#define BOARD_I2C1_USE_HSI            0U
#define BOARD_I2C1_TIMING             0x10320309UL
#endif

#define BOARD_STATUS_LED_ACTIVE_HIGH          1

#define BOARD_MOTOR_POLE_PAIRS              7.0f
#define BOARD_MOTOR_RS_OHM                  2.55f
#define BOARD_MOTOR_LD_H                    0.00086f
#define BOARD_MOTOR_LQ_H                    0.00086f
#define BOARD_MOTOR_FLUX_WB                 0.0035f
#define BOARD_MOTOR_KT_NM_PER_A             0.03675f
#define BOARD_MOTOR_CONTINUOUS_CURRENT_A    0.50f
#define BOARD_MOTOR_PEAK_CURRENT_A          1.00f
#define BOARD_BUS_NOMINAL_V                 12.0f
#define BOARD_BUS_UNDERVOLTAGE_V             7.5f
#define BOARD_BUS_OVERVOLTAGE_V             16.0f

#define BOARD_MIT_STALE_TIMEOUT_US          5000UL
#define BOARD_MIT_DISABLE_TIMEOUT_US       20000UL

/*
 * These three values are board measurements, not motor data. Production
 * builds intentionally remain interlocked until they are supplied from the
 * actual current-amplifier and bus-divider schematic and calibration record.
 */
#ifndef BOARD_PHASE_CURRENT_A_PER_COUNT
#define BOARD_PHASE_CURRENT_A_PER_COUNT       0.0f
#endif
#ifndef BOARD_PHASE_A_CURRENT_A_PER_COUNT
#define BOARD_PHASE_A_CURRENT_A_PER_COUNT BOARD_PHASE_CURRENT_A_PER_COUNT
#endif
#ifndef BOARD_PHASE_B_CURRENT_A_PER_COUNT
#define BOARD_PHASE_B_CURRENT_A_PER_COUNT BOARD_PHASE_CURRENT_A_PER_COUNT
#endif
#ifndef BOARD_BUS_VOLTAGE_V_PER_COUNT
#define BOARD_BUS_VOLTAGE_V_PER_COUNT         0.0f
#endif
#define BOARD_BUS_ADC_MIN_VALID_COUNT             8U
#define BOARD_BUS_ADC_MAX_VALID_COUNT          4087U
#ifndef BOARD_ELECTRICAL_ZERO_VALID
#define BOARD_ELECTRICAL_ZERO_VALID 0
#endif
#ifndef BOARD_ELECTRICAL_ZERO_RAD
#define BOARD_ELECTRICAL_ZERO_RAD              0.0f
#endif
#ifndef BOARD_ENCODER_DIRECTION
#define BOARD_ENCODER_DIRECTION                1.0f
#endif

#define BOARD_CURRENT_OFFSET_SAMPLES          2048U
#define BOARD_CURRENT_ADC_MIN_VALID_COUNT         8U
#define BOARD_CURRENT_ADC_MAX_VALID_COUNT      4087U
#define BOARD_CURRENT_OFFSET_MIN_COUNT          512U
#define BOARD_CURRENT_OFFSET_MAX_COUNT         3584U
#define BOARD_CURRENT_OFFSET_MAX_SPAN_COUNT      64U
#define BOARD_ENCODER_STALE_US_PRODUCTION     2000UL
/* The GPIO-I2C AS5600 path has measured recovered transactions just over
 * 5 ms while the 20 kHz PWM ISR is active. Keep a bounded margin below the
 * angle PLL's independent 10 ms re-acquisition limit. */
#define BOARD_ENCODER_STALE_US_LAB            8000UL

/* Bench-only degraded policy: AS5600 STATUS.ML may be used for the guarded
 * low-voltage Lab commissioning sequence when it is the only health flag.
 * The warning remains visible in the ST-Link status; Bus, NoMagnet,
 * MagnetStrong, stale or combined health faults remain invalid. */
#define BOARD_LAB_ACCEPT_WEAK_MAGNET              1U

/* Conservative GPIO-I2C timing for the current harness. The bit-bang backend
 * deliberately keeps a full data-hold half-cycle plus the next bit's setup
 * half-cycle, so the effective write clock is about 133 kHz. Production
 * hardware I2C remains independent. */
#define BOARD_LAB_AS5600_GPIO_I2C_HZ          200000UL

/* Reject impossible AS5600 count jumps before they reach either the angle
 * tracker or the electrical commutation angle. 60 rad/s is three times the
 * Lab command ceiling; the fixed count allowance covers quantization and
 * software-I2C scheduling jitter without admitting a gross 0xFFF bus frame. */
#define BOARD_LAB_AS5600_MAX_STEP_SPEED_RAD_S     60.0f
#define BOARD_LAB_AS5600_STEP_MARGIN_COUNTS          16U
#define BOARD_LAB_AS5600_STARTUP_MATCH_COUNTS         64U

/* One-shot ST-Link commissioning limits for the no-current-sensor lab board.
 * With Rs=2.55 ohm, 1.2 V is a deliberately conservative standstill vector;
 * it is not a measured current limit and must not be presented as torque
 * control. Every run command also has a hard duration cap. */
#define BOARD_LAB_MIN_ALIGNMENT_V               0.30f
#define BOARD_LAB_MAX_VOLTAGE_V                 1.20f
#define BOARD_LAB_MAX_VELOCITY_RAD_S           20.0f
#define BOARD_LAB_MAX_OPEN_LOOP_VOLTAGE_V        0.30f
#define BOARD_LAB_MAX_OPEN_LOOP_ELECTRICAL_RAD_S 10.0f
#define BOARD_LAB_MAX_ABS_POSITION_RAD        100.0f
#define BOARD_LAB_ENCODER_WAIT_TIMEOUT_US   1000000UL
#define BOARD_LAB_ALIGN_ZERO_TIME_US        1000000UL
#define BOARD_LAB_ALIGN_QUADRATURE_TIME_US  1000000UL
#define BOARD_LAB_ALIGN_RETURN_TIME_US      1000000UL
#define BOARD_LAB_MAX_ALIGNMENT_REPEAT_ERROR_RAD 0.05f
#define BOARD_LAB_DEFAULT_RUN_TIME_US       2000000UL
#define BOARD_LAB_MAX_RUN_TIME_US          10000000UL
#define BOARD_LAB_MAX_OPEN_LOOP_RUN_TIME_US 1000000UL
#define BOARD_LAB_SPEED_KP_V_PER_RAD_S          0.20f
#define BOARD_LAB_SPEED_KI_V_PER_RAD             0.30f
#define BOARD_LAB_VELOCITY_STICTION_COMPENSATION_V 0.20f
#define BOARD_LAB_VELOCITY_STICTION_FADE_SPEED_RAD_S 0.75f
#define BOARD_LAB_POSITION_STICTION_COMPENSATION_V 0.40f
#define BOARD_LAB_POSITION_STICTION_MIN_SCALE       0.70f
#define BOARD_LAB_POSITION_STICTION_ZERO_ERROR_RAD  0.003f
#define BOARD_LAB_POSITION_STICTION_FULL_ERROR_RAD  0.020f
#define BOARD_LAB_POSITION_STICTION_FADE_SPEED_RAD_S 0.75f
#define BOARD_LAB_POSITION_BREAKAWAY_RESET_SPEED_RAD_S 0.25f
#define BOARD_LAB_STICTION_FULL_TARGET_RAD_S       0.5f
#define BOARD_LAB_POSITION_KP_PER_S               3.0f
#define BOARD_LAB_ANGLE_PLL_KP_PER_S              80.0f
#define BOARD_LAB_ANGLE_PLL_KI_PER_S2           1600.0f

#define BOARD_CURRENT_KP_V_PER_A                0.60f
#define BOARD_CURRENT_KI_V_PER_AS            1800.0f
#define BOARD_CURRENT_AW_PER_S                2000.0f
#define BOARD_SPEED_KP_A_PER_RAD_S              0.005f
#define BOARD_SPEED_KI_A_PER_RAD                0.10f
#define BOARD_POSITION_KP_PER_S                 15.0f

#define BOARD_MAX_MECHANICAL_SPEED_RAD_S       300.0f
/* Must remain above the trip threshold so the PLL does not mathematically
 * hide an overspeed before collect_faults() can observe it. */
#define BOARD_ANGLE_PLL_MAX_SPEED_RAD_S         450.0f
#define BOARD_MAX_PROFILE_SPEED_RAD_S          100.0f
#define BOARD_MAX_PROFILE_ACCEL_RAD_S2         300.0f
#define BOARD_MAX_PROFILE_DECEL_RAD_S2         400.0f
#define BOARD_POSITION_TOLERANCE_RAD             0.002f
#define BOARD_VELOCITY_TOLERANCE_RAD_S           0.05f

#endif
