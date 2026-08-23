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
#define BOARD_ENCODER_STALE_US_LAB            5000UL

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
