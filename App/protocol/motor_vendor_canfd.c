#include "motor_vendor_canfd.h"

#include <limits.h>
#include <stddef.h>

#define COMMAND_CRC_OFFSET ((size_t)30U)
#define FEEDBACK_CRC_OFFSET ((size_t)46U)
#define MOTOR_VENDOR_MODE_PP INT8_C(1)
#define MOTOR_VENDOR_MODE_PV INT8_C(3)
#define MOTOR_VENDOR_MODE_MIT INT8_MIN
#define MOTOR_VENDOR_MAX_PDS_STATE UINT8_C(7)
#define MOTOR_VENDOR_MAX_TORQUE_LIMIT_Q15 UINT16_C(32767)

static void put_u16_le(uint8_t *destination, uint16_t value)
{
    destination[0] = (uint8_t)value;
    destination[1] = (uint8_t)(value >> 8);
}

static void put_u32_le(uint8_t *destination, uint32_t value)
{
    destination[0] = (uint8_t)value;
    destination[1] = (uint8_t)(value >> 8);
    destination[2] = (uint8_t)(value >> 16);
    destination[3] = (uint8_t)(value >> 24);
}

static uint16_t get_u16_le(const uint8_t *source)
{
    return (uint16_t)((uint16_t)source[0] |
                      ((uint16_t)source[1] << 8));
}

static uint32_t get_u32_le(const uint8_t *source)
{
    return (uint32_t)source[0] | ((uint32_t)source[1] << 8) |
           ((uint32_t)source[2] << 16) | ((uint32_t)source[3] << 24);
}

static void put_i32_le(uint8_t *destination, int32_t value)
{
    put_u32_le(destination, (uint32_t)value);
}

static int32_t get_i32_le(const uint8_t *source)
{
    const uint32_t value = get_u32_le(source);
    if (value <= (uint32_t)INT32_MAX) {
        return (int32_t)value;
    }
    /* Avoid relying on implementation-defined unsigned-to-signed conversion. */
    return (int32_t)(-INT32_C(1) - (int32_t)(UINT32_MAX - value));
}

static int8_t get_i8(uint8_t value)
{
    if (value <= (uint8_t)INT8_MAX) {
        return (int8_t)value;
    }
    return (int8_t)(-INT8_C(1) - (int8_t)(UINT8_MAX - value));
}

uint16_t motor_vendor_crc16_ccitt(const uint8_t *data, size_t length)
{
    uint16_t crc = UINT16_C(0xFFFF);
    size_t index;

    if ((data == NULL) && (length != 0U)) {
        return 0U;
    }
    for (index = 0U; index < length; ++index) {
        uint8_t bit;
        crc ^= (uint16_t)((uint16_t)data[index] << 8);
        for (bit = 0U; bit < 8U; ++bit) {
            crc = ((crc & UINT16_C(0x8000)) != 0U)
                      ? (uint16_t)((uint16_t)(crc << 1) ^ UINT16_C(0x1021))
                      : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static bool mode_is_valid(int8_t mode)
{
    return (mode == MOTOR_VENDOR_MODE_PP) ||
           (mode == MOTOR_VENDOR_MODE_PV) ||
           (mode == MOTOR_VENDOR_MODE_MIT);
}

static bool signed_in_symmetric_range(int32_t value, int32_t maximum)
{
    return (maximum >= 0) && (value <= maximum) && (value >= -maximum);
}

static bool command_limits_are_valid(
    const motor_vendor_command_limits_t *limits)
{
    return (limits != NULL) &&
           (limits->minimum_position_q16_16 <=
            limits->maximum_position_q16_16) &&
           (limits->maximum_abs_velocity_q16_16 >= 0) &&
           (limits->maximum_abs_torque_q16_16 >= 0);
}

static bool command_in_range(const motor_vendor_command_t *command,
                             const motor_vendor_command_limits_t *limits)
{
    return command_limits_are_valid(limits) &&
           ((command->flags & (uint8_t)~MOTOR_VENDOR_CMD_ALLOWED_FLAGS) ==
            0U) &&
           mode_is_valid(command->mode) &&
           (command->position_q16_16 >=
            limits->minimum_position_q16_16) &&
           (command->position_q16_16 <=
            limits->maximum_position_q16_16) &&
           signed_in_symmetric_range(
               command->velocity_q16_16,
               limits->maximum_abs_velocity_q16_16) &&
           signed_in_symmetric_range(command->torque_q16_16,
                                     limits->maximum_abs_torque_q16_16) &&
           (command->kp_uq8_8 <= limits->maximum_kp_uq8_8) &&
           (command->kd_uq8_8 <= limits->maximum_kd_uq8_8) &&
           (command->current_limit_uq8_8 <=
            limits->maximum_current_uq8_8);
}

motor_vendor_canfd_result_t motor_vendor_encode_command(
    const motor_vendor_command_t *command,
    const motor_vendor_command_limits_t *limits,
    uint8_t *frame,
    size_t frame_length)
{
    uint16_t crc;

    if ((command == NULL) || (limits == NULL) || (frame == NULL)) {
        return MOTOR_VENDOR_CANFD_NULL_ARGUMENT;
    }
    if (frame_length != MOTOR_VENDOR_CANFD_COMMAND_LENGTH) {
        return MOTOR_VENDOR_CANFD_LENGTH_ERROR;
    }
    if (!command_in_range(command, limits)) {
        return MOTOR_VENDOR_CANFD_RANGE_ERROR;
    }

    frame[0] = MOTOR_VENDOR_CANFD_MAGIC;
    frame[1] = MOTOR_VENDOR_CANFD_VERSION;
    frame[2] = MOTOR_VENDOR_CANFD_COMMAND_TYPE;
    frame[3] = command->flags;
    put_u16_le(&frame[4], command->sequence);
    put_u32_le(&frame[6], command->sender_timestamp_us);
    put_i32_le(&frame[10], command->position_q16_16);
    put_i32_le(&frame[14], command->velocity_q16_16);
    put_i32_le(&frame[18], command->torque_q16_16);
    put_u16_le(&frame[22], command->kp_uq8_8);
    put_u16_le(&frame[24], command->kd_uq8_8);
    put_u16_le(&frame[26], command->current_limit_uq8_8);
    frame[28] = (uint8_t)command->mode;
    frame[29] = 0U;
    crc = motor_vendor_crc16_ccitt(frame, COMMAND_CRC_OFFSET);
    put_u16_le(&frame[COMMAND_CRC_OFFSET], crc);
    return MOTOR_VENDOR_CANFD_OK;
}

motor_vendor_canfd_result_t motor_vendor_decode_command(
    const uint8_t *frame,
    size_t frame_length,
    const motor_vendor_command_limits_t *limits,
    motor_vendor_command_t *command)
{
    motor_vendor_command_t decoded;

    if ((frame == NULL) || (limits == NULL) || (command == NULL)) {
        return MOTOR_VENDOR_CANFD_NULL_ARGUMENT;
    }
    if (frame_length != MOTOR_VENDOR_CANFD_COMMAND_LENGTH) {
        return MOTOR_VENDOR_CANFD_LENGTH_ERROR;
    }
    if (frame[0] != MOTOR_VENDOR_CANFD_MAGIC) {
        return MOTOR_VENDOR_CANFD_MAGIC_ERROR;
    }
    if (frame[1] != MOTOR_VENDOR_CANFD_VERSION) {
        return MOTOR_VENDOR_CANFD_VERSION_ERROR;
    }
    if (frame[2] != MOTOR_VENDOR_CANFD_COMMAND_TYPE) {
        return MOTOR_VENDOR_CANFD_TYPE_ERROR;
    }
    if ((frame[3] & (uint8_t)~MOTOR_VENDOR_CMD_ALLOWED_FLAGS) != 0U) {
        return MOTOR_VENDOR_CANFD_FLAGS_ERROR;
    }
    if (frame[29] != 0U) {
        return MOTOR_VENDOR_CANFD_RESERVED_ERROR;
    }
    if (get_u16_le(&frame[COMMAND_CRC_OFFSET]) !=
        motor_vendor_crc16_ccitt(frame, COMMAND_CRC_OFFSET)) {
        return MOTOR_VENDOR_CANFD_CRC_ERROR;
    }

    decoded.flags = frame[3];
    decoded.sequence = get_u16_le(&frame[4]);
    decoded.sender_timestamp_us = get_u32_le(&frame[6]);
    decoded.local_rx_timestamp_us = 0U;
    decoded.position_q16_16 = get_i32_le(&frame[10]);
    decoded.velocity_q16_16 = get_i32_le(&frame[14]);
    decoded.torque_q16_16 = get_i32_le(&frame[18]);
    decoded.kp_uq8_8 = get_u16_le(&frame[22]);
    decoded.kd_uq8_8 = get_u16_le(&frame[24]);
    decoded.current_limit_uq8_8 = get_u16_le(&frame[26]);
    decoded.mode = get_i8(frame[28]);
    if (!command_in_range(&decoded, limits)) {
        return MOTOR_VENDOR_CANFD_RANGE_ERROR;
    }
    *command = decoded;
    return MOTOR_VENDOR_CANFD_OK;
}

void motor_vendor_rx_guard_init(motor_vendor_rx_guard_t *guard,
                                uint16_t maximum_forward_gap)
{
    if (guard == NULL) {
        return;
    }
    guard->initialized = false;
    guard->last_sequence = 0U;
    guard->maximum_forward_gap = maximum_forward_gap;
    guard->last_local_rx_timestamp_us = 0U;
}

motor_vendor_canfd_result_t motor_vendor_accept_command(
    motor_vendor_rx_guard_t *guard,
    const uint8_t *frame,
    size_t frame_length,
    uint32_t local_rx_timestamp_us,
    const motor_vendor_command_limits_t *limits,
    motor_vendor_command_t *command)
{
    motor_vendor_command_t decoded;
    motor_vendor_canfd_result_t result;

    if ((guard == NULL) || (command == NULL)) {
        return MOTOR_VENDOR_CANFD_NULL_ARGUMENT;
    }
    result = motor_vendor_decode_command(frame, frame_length, limits, &decoded);
    if (result != MOTOR_VENDOR_CANFD_OK) {
        return result;
    }

    if (guard->initialized) {
        const uint16_t delta =
            (uint16_t)(decoded.sequence - guard->last_sequence);
        if (delta == 0U) {
            return MOTOR_VENDOR_CANFD_SEQUENCE_DUPLICATE;
        }
        if (delta >= UINT16_C(0x8000)) {
            return MOTOR_VENDOR_CANFD_SEQUENCE_OLD;
        }
        if ((guard->maximum_forward_gap != 0U) &&
            (delta > guard->maximum_forward_gap)) {
            /* A forward gap is diagnostic, not a malformed command.  Finish
             * the same validated/stamped commit path so callers that accept
             * gaps never observe an uninitialized output structure. */
            result = MOTOR_VENDOR_CANFD_SEQUENCE_GAP;
        }
    }

    decoded.local_rx_timestamp_us = local_rx_timestamp_us;
    guard->initialized = true;
    guard->last_sequence = decoded.sequence;
    guard->last_local_rx_timestamp_us = local_rx_timestamp_us;
    *command = decoded;
    return result;
}

bool motor_vendor_command_is_fresh(const motor_vendor_command_t *command,
                                   uint32_t local_now_us,
                                   uint32_t maximum_age_us)
{
    return (command != NULL) && (maximum_age_us <= (uint32_t)INT32_MAX) &&
           ((uint32_t)(local_now_us - command->local_rx_timestamp_us) <=
            maximum_age_us);
}

bool motor_vendor_rx_guard_is_fresh(const motor_vendor_rx_guard_t *guard,
                                    uint32_t local_now_us,
                                    uint32_t maximum_age_us)
{
    return (guard != NULL) && guard->initialized &&
           (maximum_age_us <= (uint32_t)INT32_MAX) &&
           ((uint32_t)(local_now_us - guard->last_local_rx_timestamp_us) <=
            maximum_age_us);
}

static bool feedback_limits_are_valid(
    const motor_vendor_feedback_limits_t *limits)
{
    return (limits != NULL) &&
           (limits->minimum_position_q16_16 <=
            limits->maximum_position_q16_16) &&
           (limits->maximum_abs_velocity_q16_16 >= 0) &&
           (limits->maximum_abs_torque_q16_16 >= 0) &&
           (limits->maximum_abs_iq_q16_16 >= 0);
}

static bool feedback_in_range(const motor_vendor_feedback_t *feedback,
                              const motor_vendor_feedback_limits_t *limits)
{
    return feedback_limits_are_valid(limits) &&
           ((feedback->flags & (uint8_t)~MOTOR_VENDOR_FB_ALLOWED_FLAGS) ==
            0U) &&
           (feedback->position_q16_16 >=
            limits->minimum_position_q16_16) &&
           (feedback->position_q16_16 <=
            limits->maximum_position_q16_16) &&
           signed_in_symmetric_range(
               feedback->velocity_q16_16,
               limits->maximum_abs_velocity_q16_16) &&
           signed_in_symmetric_range(feedback->torque_q16_16,
                                     limits->maximum_abs_torque_q16_16) &&
           signed_in_symmetric_range(feedback->iq_q16_16,
                                     limits->maximum_abs_iq_q16_16) &&
           (feedback->bus_voltage_uq16_16 <=
            limits->maximum_bus_voltage_uq16_16) &&
           (feedback->pds_state <= MOTOR_VENDOR_MAX_PDS_STATE) &&
           (feedback->torque_limit_q15 <=
            MOTOR_VENDOR_MAX_TORQUE_LIMIT_Q15) &&
           ((feedback->mode_display == 0) ||
            mode_is_valid(feedback->mode_display));
}

motor_vendor_canfd_result_t motor_vendor_encode_feedback(
    const motor_vendor_feedback_t *feedback,
    const motor_vendor_feedback_limits_t *limits,
    uint8_t *frame,
    size_t frame_length)
{
    uint16_t crc;

    if ((feedback == NULL) || (limits == NULL) || (frame == NULL)) {
        return MOTOR_VENDOR_CANFD_NULL_ARGUMENT;
    }
    if (frame_length != MOTOR_VENDOR_CANFD_FEEDBACK_LENGTH) {
        return MOTOR_VENDOR_CANFD_LENGTH_ERROR;
    }
    if (!feedback_in_range(feedback, limits)) {
        return MOTOR_VENDOR_CANFD_RANGE_ERROR;
    }

    frame[0] = MOTOR_VENDOR_CANFD_MAGIC;
    frame[1] = MOTOR_VENDOR_CANFD_VERSION;
    frame[2] = MOTOR_VENDOR_CANFD_FEEDBACK_TYPE;
    frame[3] = feedback->flags;
    put_u16_le(&frame[4], feedback->sequence);
    put_u16_le(&frame[6], feedback->command_sequence_echo);
    put_u32_le(&frame[8], feedback->local_timestamp_us);
    put_i32_le(&frame[12], feedback->position_q16_16);
    put_i32_le(&frame[16], feedback->velocity_q16_16);
    put_i32_le(&frame[20], feedback->torque_q16_16);
    put_i32_le(&frame[24], feedback->iq_q16_16);
    put_u32_le(&frame[28], feedback->bus_voltage_uq16_16);
    put_u16_le(&frame[32], feedback->statusword);
    frame[34] = (uint8_t)feedback->mode_display;
    frame[35] = feedback->pds_state;
    put_u32_le(&frame[36], feedback->active_faults);
    put_u32_le(&frame[40], feedback->latched_faults);
    put_u16_le(&frame[44], feedback->torque_limit_q15);
    crc = motor_vendor_crc16_ccitt(frame, FEEDBACK_CRC_OFFSET);
    put_u16_le(&frame[FEEDBACK_CRC_OFFSET], crc);
    return MOTOR_VENDOR_CANFD_OK;
}

motor_vendor_canfd_result_t motor_vendor_decode_feedback(
    const uint8_t *frame,
    size_t frame_length,
    const motor_vendor_feedback_limits_t *limits,
    motor_vendor_feedback_t *feedback)
{
    motor_vendor_feedback_t decoded;

    if ((frame == NULL) || (limits == NULL) || (feedback == NULL)) {
        return MOTOR_VENDOR_CANFD_NULL_ARGUMENT;
    }
    if (frame_length != MOTOR_VENDOR_CANFD_FEEDBACK_LENGTH) {
        return MOTOR_VENDOR_CANFD_LENGTH_ERROR;
    }
    if (frame[0] != MOTOR_VENDOR_CANFD_MAGIC) {
        return MOTOR_VENDOR_CANFD_MAGIC_ERROR;
    }
    if (frame[1] != MOTOR_VENDOR_CANFD_VERSION) {
        return MOTOR_VENDOR_CANFD_VERSION_ERROR;
    }
    if (frame[2] != MOTOR_VENDOR_CANFD_FEEDBACK_TYPE) {
        return MOTOR_VENDOR_CANFD_TYPE_ERROR;
    }
    if ((frame[3] & (uint8_t)~MOTOR_VENDOR_FB_ALLOWED_FLAGS) != 0U) {
        return MOTOR_VENDOR_CANFD_FLAGS_ERROR;
    }
    if (get_u16_le(&frame[FEEDBACK_CRC_OFFSET]) !=
        motor_vendor_crc16_ccitt(frame, FEEDBACK_CRC_OFFSET)) {
        return MOTOR_VENDOR_CANFD_CRC_ERROR;
    }

    decoded.flags = frame[3];
    decoded.sequence = get_u16_le(&frame[4]);
    decoded.command_sequence_echo = get_u16_le(&frame[6]);
    decoded.local_timestamp_us = get_u32_le(&frame[8]);
    decoded.position_q16_16 = get_i32_le(&frame[12]);
    decoded.velocity_q16_16 = get_i32_le(&frame[16]);
    decoded.torque_q16_16 = get_i32_le(&frame[20]);
    decoded.iq_q16_16 = get_i32_le(&frame[24]);
    decoded.bus_voltage_uq16_16 = get_u32_le(&frame[28]);
    decoded.statusword = get_u16_le(&frame[32]);
    decoded.mode_display = get_i8(frame[34]);
    decoded.pds_state = frame[35];
    decoded.active_faults = get_u32_le(&frame[36]);
    decoded.latched_faults = get_u32_le(&frame[40]);
    decoded.torque_limit_q15 = get_u16_le(&frame[44]);
    if (!feedback_in_range(&decoded, limits)) {
        return MOTOR_VENDOR_CANFD_RANGE_ERROR;
    }
    *feedback = decoded;
    return MOTOR_VENDOR_CANFD_OK;
}
