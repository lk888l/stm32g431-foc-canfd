#ifndef APP_PROTOCOL_MOTOR_VENDOR_CANFD_H
#define APP_PROTOCOL_MOTOR_VENDOR_CANFD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Manufacturer-specific CAN-FD transport for a tightly timed MIT command.
 * This payload is NOT CiA 1301 (CANopen FD), does not implement its services,
 * and must not be advertised as interoperable CANopen FD. It may coexist with
 * a separately implemented CANopen/CiA-402 channel after CAN-ID allocation is
 * reviewed for the deployed network.
 *
 * Signed motion quantities use Q16.16. Gains/current use unsigned Q8.8.
 * Every multi-byte field is serialized explicitly in little-endian order.
 */

#define MOTOR_VENDOR_CANFD_COMMAND_LENGTH ((size_t)32U)
#define MOTOR_VENDOR_CANFD_FEEDBACK_LENGTH ((size_t)48U)
#define MOTOR_VENDOR_CANFD_MAGIC UINT8_C(0x4D)
#define MOTOR_VENDOR_CANFD_VERSION UINT8_C(1)
#define MOTOR_VENDOR_CANFD_COMMAND_TYPE UINT8_C(0x10)
#define MOTOR_VENDOR_CANFD_FEEDBACK_TYPE UINT8_C(0x11)

enum {
    MOTOR_VENDOR_CMD_ENABLE = UINT8_C(1) << 0,
    MOTOR_VENDOR_CMD_QUICK_STOP = UINT8_C(1) << 1,
    MOTOR_VENDOR_CMD_CLEAR_FAULT = UINT8_C(1) << 2,
    MOTOR_VENDOR_CMD_ZERO_POSITION = UINT8_C(1) << 3
};

#define MOTOR_VENDOR_CMD_ALLOWED_FLAGS UINT8_C(0x0F)

enum {
    MOTOR_VENDOR_FB_OPERATION_ENABLED = UINT8_C(1) << 0,
    MOTOR_VENDOR_FB_WARNING = UINT8_C(1) << 1,
    MOTOR_VENDOR_FB_TARGET_REACHED = UINT8_C(1) << 2,
    MOTOR_VENDOR_FB_COMMAND_FRESH = UINT8_C(1) << 3,
    MOTOR_VENDOR_FB_DERATED = UINT8_C(1) << 4
};

#define MOTOR_VENDOR_FB_ALLOWED_FLAGS UINT8_C(0x1F)

typedef enum {
    MOTOR_VENDOR_CANFD_OK = 0,
    MOTOR_VENDOR_CANFD_NULL_ARGUMENT,
    MOTOR_VENDOR_CANFD_LENGTH_ERROR,
    MOTOR_VENDOR_CANFD_MAGIC_ERROR,
    MOTOR_VENDOR_CANFD_VERSION_ERROR,
    MOTOR_VENDOR_CANFD_TYPE_ERROR,
    MOTOR_VENDOR_CANFD_FLAGS_ERROR,
    MOTOR_VENDOR_CANFD_RESERVED_ERROR,
    MOTOR_VENDOR_CANFD_CRC_ERROR,
    MOTOR_VENDOR_CANFD_RANGE_ERROR,
    MOTOR_VENDOR_CANFD_SEQUENCE_DUPLICATE,
    MOTOR_VENDOR_CANFD_SEQUENCE_OLD,
    MOTOR_VENDOR_CANFD_SEQUENCE_GAP
} motor_vendor_canfd_result_t;

typedef struct {
    int32_t minimum_position_q16_16;
    int32_t maximum_position_q16_16;
    int32_t maximum_abs_velocity_q16_16;
    int32_t maximum_abs_torque_q16_16;
    uint16_t maximum_kp_uq8_8;
    uint16_t maximum_kd_uq8_8;
    uint16_t maximum_current_uq8_8;
} motor_vendor_command_limits_t;

typedef struct {
    uint8_t flags;
    uint16_t sequence;
    uint32_t sender_timestamp_us;
    uint32_t local_rx_timestamp_us;
    int32_t position_q16_16;
    int32_t velocity_q16_16;
    int32_t torque_q16_16;
    uint16_t kp_uq8_8;
    uint16_t kd_uq8_8;
    uint16_t current_limit_uq8_8;
    int8_t mode;
} motor_vendor_command_t;

typedef struct {
    bool initialized;
    uint16_t last_sequence;
    uint16_t maximum_forward_gap;
    uint32_t last_local_rx_timestamp_us;
} motor_vendor_rx_guard_t;

typedef struct {
    int32_t minimum_position_q16_16;
    int32_t maximum_position_q16_16;
    int32_t maximum_abs_velocity_q16_16;
    int32_t maximum_abs_torque_q16_16;
    int32_t maximum_abs_iq_q16_16;
    uint32_t maximum_bus_voltage_uq16_16;
} motor_vendor_feedback_limits_t;

typedef struct {
    uint8_t flags;
    uint16_t sequence;
    uint16_t command_sequence_echo;
    uint32_t local_timestamp_us;
    int32_t position_q16_16;
    int32_t velocity_q16_16;
    int32_t torque_q16_16;
    int32_t iq_q16_16;
    uint32_t bus_voltage_uq16_16;
    uint16_t statusword;
    int8_t mode_display;
    uint8_t pds_state;
    uint32_t active_faults;
    uint32_t latched_faults;
    uint16_t torque_limit_q15;
} motor_vendor_feedback_t;

motor_vendor_canfd_result_t motor_vendor_encode_command(
    const motor_vendor_command_t *command,
    const motor_vendor_command_limits_t *limits,
    uint8_t *frame,
    size_t frame_length);

motor_vendor_canfd_result_t motor_vendor_decode_command(
    const uint8_t *frame,
    size_t frame_length,
    const motor_vendor_command_limits_t *limits,
    motor_vendor_command_t *command);

void motor_vendor_rx_guard_init(motor_vendor_rx_guard_t *guard,
                                uint16_t maximum_forward_gap);

/* Decode, check monotonic sequence, then stamp with receiver-local time. */
motor_vendor_canfd_result_t motor_vendor_accept_command(
    motor_vendor_rx_guard_t *guard,
    const uint8_t *frame,
    size_t frame_length,
    uint32_t local_rx_timestamp_us,
    const motor_vendor_command_limits_t *limits,
    motor_vendor_command_t *command);

bool motor_vendor_command_is_fresh(const motor_vendor_command_t *command,
                                   uint32_t local_now_us,
                                   uint32_t maximum_age_us);
bool motor_vendor_rx_guard_is_fresh(const motor_vendor_rx_guard_t *guard,
                                    uint32_t local_now_us,
                                    uint32_t maximum_age_us);

motor_vendor_canfd_result_t motor_vendor_encode_feedback(
    const motor_vendor_feedback_t *feedback,
    const motor_vendor_feedback_limits_t *limits,
    uint8_t *frame,
    size_t frame_length);

motor_vendor_canfd_result_t motor_vendor_decode_feedback(
    const uint8_t *frame,
    size_t frame_length,
    const motor_vendor_feedback_limits_t *limits,
    motor_vendor_feedback_t *feedback);

uint16_t motor_vendor_crc16_ccitt(const uint8_t *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif /* APP_PROTOCOL_MOTOR_VENDOR_CANFD_H */
