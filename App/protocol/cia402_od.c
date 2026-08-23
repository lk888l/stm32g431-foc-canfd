#include "cia402_od.h"

#include <limits.h>
#include <stddef.h>

#define CIA402_MODE_PP INT8_C(1)
#define CIA402_MODE_PV INT8_C(3)
#define CIA402_MODE_MIT INT8_MIN
#define CIA402_SUPPORTED_MODE_PP (UINT32_C(1) << 0)
#define CIA402_SUPPORTED_MODE_PV (UINT32_C(1) << 2)
#define CIA402_SUPPORTED_MODE_MIT_MANUFACTURER (UINT32_C(1) << 31)
#define CIA402_CONTROLWORD_NEW_SET_POINT UINT16_C(0x0010)

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

static int16_t get_i16_le(const uint8_t *source)
{
    const uint16_t value = get_u16_le(source);
    if (value <= (uint16_t)INT16_MAX) {
        return (int16_t)value;
    }
    /* Avoid implementation-defined conversion for the negative half-range. */
    return (int16_t)(-INT16_C(1) -
                     (int16_t)(UINT16_MAX - value));
}

static int8_t get_i8(uint8_t value)
{
    if (value <= (uint8_t)INT8_MAX) {
        return (int8_t)value;
    }
    return (int8_t)(-INT8_C(1) - (int8_t)(UINT8_MAX - value));
}

static int32_t get_i32_le(const uint8_t *source)
{
    const uint32_t value = get_u32_le(source);
    if (value <= (uint32_t)INT32_MAX) {
        return (int32_t)value;
    }
    return (int32_t)(-INT32_C(1) - (int32_t)(UINT32_MAX - value));
}

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

static void put_i32_le(uint8_t *destination, int32_t value)
{
    put_u32_le(destination, (uint32_t)value);
}

static bool mode_supported(const cia402_od_t *od, int8_t mode)
{
    return ((mode == CIA402_MODE_PP) && od->config.support_pp) ||
           ((mode == CIA402_MODE_PV) && od->config.support_pv) ||
           ((mode == CIA402_MODE_MIT) && od->config.support_mit);
}

static bool signed_in_symmetric_range_i32(int32_t value, int32_t maximum)
{
    return (maximum >= 0) && (value >= -maximum) && (value <= maximum);
}

static bool signed_in_symmetric_range_i16(int16_t value, int16_t maximum)
{
    return (maximum >= 0) && (value >= (int16_t)-maximum) &&
           (value <= maximum);
}

bool cia402_od_init(cia402_od_t *od, const cia402_od_config_t *config)
{
    uint32_t supported = 0U;

    if ((od == NULL) || (config == NULL) ||
        (config->minimum_position_q16_16 >
         config->maximum_position_q16_16) ||
        (config->maximum_abs_velocity_q16_16 < 0) ||
        (config->maximum_abs_mit_torque_q16_16 < 0) ||
        (config->maximum_abs_target_torque_per_mille < 0) ||
        (config->maximum_profile_velocity_q16_16 == 0U) ||
        (config->maximum_profile_acceleration_q16_16 == 0U) ||
        (config->maximum_profile_deceleration_q16_16 == 0U)) {
        return false;
    }

    od->config = *config;
    od->image = (cia402_od_image_t){0};
    od->command_sync = (cia402_od_command_sync_t){0};
    if (config->support_pp) {
        supported |= CIA402_SUPPORTED_MODE_PP;
    }
    if (config->support_pv) {
        supported |= CIA402_SUPPORTED_MODE_PV;
    }
    if (config->support_mit) {
        /* Local documentation defines bit 31; it is not a standardized mode bit. */
        supported |= CIA402_SUPPORTED_MODE_MIT_MANUFACTURER;
    }
    od->image.supported_modes = supported;
    od->image.profile_velocity_q16_16 =
        config->maximum_profile_velocity_q16_16;
    od->image.profile_acceleration_q16_16 =
        config->maximum_profile_acceleration_q16_16;
    od->image.profile_deceleration_q16_16 =
        config->maximum_profile_deceleration_q16_16;
    return true;
}

static cia402_od_result_t require_length(size_t actual, size_t expected)
{
    return (actual == expected) ? CIA402_OD_OK : CIA402_OD_LENGTH_ERROR;
}

static bool target_position_is_valid(const cia402_od_t *od, int32_t value)
{
    return (value >= od->config.minimum_position_q16_16) &&
           (value <= od->config.maximum_position_q16_16);
}

static bool target_velocity_is_valid(const cia402_od_t *od, int32_t value)
{
    return signed_in_symmetric_range_i32(
        value, od->config.maximum_abs_velocity_q16_16);
}

static uint32_t next_epoch(uint32_t epoch)
{
    /* Unsigned wrap is intentional.  Consumers use inequality rather than
     * ordering, so UINT32_MAX -> 0 remains a fresh generation. */
    return epoch + UINT32_C(1);
}

static cia402_od_result_t write_mit(cia402_od_t *od,
                                    uint8_t subindex,
                                    const uint8_t *data,
                                    size_t length)
{
    int32_t i32;
    uint16_t u16;

    switch (subindex) {
    case CIA402_OD_MIT_SUB_COUNT:
        return CIA402_OD_READ_ONLY;
    case CIA402_OD_MIT_SUB_POSITION:
        if (require_length(length, 4U) != CIA402_OD_OK) {
            return CIA402_OD_LENGTH_ERROR;
        }
        i32 = get_i32_le(data);
        if ((i32 < od->config.minimum_position_q16_16) ||
            (i32 > od->config.maximum_position_q16_16)) {
            return CIA402_OD_VALUE_RANGE;
        }
        od->image.mit_position_q16_16 = i32;
        return CIA402_OD_OK;
    case CIA402_OD_MIT_SUB_VELOCITY:
        if (require_length(length, 4U) != CIA402_OD_OK) {
            return CIA402_OD_LENGTH_ERROR;
        }
        i32 = get_i32_le(data);
        if (!signed_in_symmetric_range_i32(
                i32, od->config.maximum_abs_velocity_q16_16)) {
            return CIA402_OD_VALUE_RANGE;
        }
        od->image.mit_velocity_q16_16 = i32;
        return CIA402_OD_OK;
    case CIA402_OD_MIT_SUB_TORQUE:
        if (require_length(length, 4U) != CIA402_OD_OK) {
            return CIA402_OD_LENGTH_ERROR;
        }
        i32 = get_i32_le(data);
        if (!signed_in_symmetric_range_i32(
                i32, od->config.maximum_abs_mit_torque_q16_16)) {
            return CIA402_OD_VALUE_RANGE;
        }
        od->image.mit_torque_q16_16 = i32;
        return CIA402_OD_OK;
    case CIA402_OD_MIT_SUB_KP:
    case CIA402_OD_MIT_SUB_KD:
    case CIA402_OD_MIT_SUB_CURRENT_LIMIT:
    case CIA402_OD_MIT_SUB_SEQUENCE:
        if (require_length(length, 2U) != CIA402_OD_OK) {
            return CIA402_OD_LENGTH_ERROR;
        }
        u16 = get_u16_le(data);
        if (((subindex == CIA402_OD_MIT_SUB_KP) &&
             (u16 > od->config.maximum_mit_kp_uq8_8)) ||
            ((subindex == CIA402_OD_MIT_SUB_KD) &&
             (u16 > od->config.maximum_mit_kd_uq8_8)) ||
            ((subindex == CIA402_OD_MIT_SUB_CURRENT_LIMIT) &&
             (u16 > od->config.maximum_mit_current_uq8_8))) {
            return CIA402_OD_VALUE_RANGE;
        }
        if (subindex == CIA402_OD_MIT_SUB_KP) {
            od->image.mit_kp_uq8_8 = u16;
        } else if (subindex == CIA402_OD_MIT_SUB_KD) {
            od->image.mit_kd_uq8_8 = u16;
        } else if (subindex == CIA402_OD_MIT_SUB_CURRENT_LIMIT) {
            od->image.mit_current_limit_uq8_8 = u16;
        } else {
            od->image.mit_sequence = u16;
        }
        return CIA402_OD_OK;
    default:
        return CIA402_OD_SUBINDEX_NOT_FOUND;
    }
}

cia402_od_result_t cia402_od_write(cia402_od_t *od,
                                   uint16_t index,
                                   uint8_t subindex,
                                   const uint8_t *data,
                                   size_t data_length,
                                   bool mode_change_inhibited)
{
    int32_t i32;
    uint32_t u32;
    int16_t i16;
    int8_t i8;

    if ((od == NULL) || (data == NULL)) {
        return CIA402_OD_NULL_ARGUMENT;
    }
    if ((index != CIA402_OD_INDEX_MIT_COMMAND) && (subindex != 0U)) {
        return CIA402_OD_SUBINDEX_NOT_FOUND;
    }

    switch (index) {
    case CIA402_OD_INDEX_CONTROLWORD:
    {
        uint16_t controlword;
        if (require_length(data_length, 2U) != CIA402_OD_OK) {
            return CIA402_OD_LENGTH_ERROR;
        }
        controlword = get_u16_le(data);
        if (((controlword & CIA402_CONTROLWORD_NEW_SET_POINT) != 0U) &&
            ((od->image.controlword &
              CIA402_CONTROLWORD_NEW_SET_POINT) == 0U)) {
            /* Freeze the exact target generation, value and PP option bits at
             * the 0->1 write.  A target arriving later cannot retroactively
             * make this request valid or change a relative command. */
            od->command_sync.pp_request_epoch =
                next_epoch(od->command_sync.pp_request_epoch);
            od->command_sync.pp_request_target_position_epoch =
                od->command_sync.target_position_epoch;
            od->command_sync.pp_request_target_position_q16_16 =
                od->image.target_position_q16_16;
            od->command_sync.pp_request_controlword = controlword;
            od->command_sync.pp_request_mode = od->image.mode_command;
            od->command_sync.pp_request_mode_instance_epoch =
                od->command_sync.mode_instance_epoch;
        }
        od->image.controlword = controlword;
        return CIA402_OD_OK;
    }

    case CIA402_OD_INDEX_MODES_OF_OPERATION:
        if (require_length(data_length, 1U) != CIA402_OD_OK) {
            return CIA402_OD_LENGTH_ERROR;
        }
        if (mode_change_inhibited) {
            return CIA402_OD_STATE_CONFLICT;
        }
        i8 = get_i8(data[0]);
        if (!mode_supported(od, i8)) {
            return CIA402_OD_VALUE_RANGE;
        }
        /* Every successful 0x6060 write, including a same-value write,
         * creates a new mode instance.  This is the command freshness barrier
         * for leave/re-enter and explicit reinitialization sequences. */
        od->image.mode_command = i8;
        od->command_sync.mode_instance_epoch =
            next_epoch(od->command_sync.mode_instance_epoch);
        return CIA402_OD_OK;

    case CIA402_OD_INDEX_TARGET_POSITION:
        if (require_length(data_length, 4U) != CIA402_OD_OK) {
            return CIA402_OD_LENGTH_ERROR;
        }
        i32 = get_i32_le(data);
        if (!target_position_is_valid(od, i32)) {
            return CIA402_OD_VALUE_RANGE;
        }
        od->image.target_position_q16_16 = i32;
        od->command_sync.target_position_epoch =
            next_epoch(od->command_sync.target_position_epoch);
        return CIA402_OD_OK;

    case CIA402_OD_INDEX_TARGET_VELOCITY:
        if (require_length(data_length, 4U) != CIA402_OD_OK) {
            return CIA402_OD_LENGTH_ERROR;
        }
        i32 = get_i32_le(data);
        if (!target_velocity_is_valid(od, i32)) {
            return CIA402_OD_VALUE_RANGE;
        }
        od->image.target_velocity_q16_16 = i32;
        od->command_sync.target_velocity_epoch =
            next_epoch(od->command_sync.target_velocity_epoch);
        od->command_sync.target_velocity_mode = od->image.mode_command;
        od->command_sync.target_velocity_mode_instance_epoch =
            od->command_sync.mode_instance_epoch;
        return CIA402_OD_OK;

    case CIA402_OD_INDEX_TARGET_TORQUE:
        if (require_length(data_length, 2U) != CIA402_OD_OK) {
            return CIA402_OD_LENGTH_ERROR;
        }
        i16 = get_i16_le(data);
        if (!signed_in_symmetric_range_i16(
                i16, od->config.maximum_abs_target_torque_per_mille)) {
            return CIA402_OD_VALUE_RANGE;
        }
        od->image.target_torque_per_mille = i16;
        return CIA402_OD_OK;

    case CIA402_OD_INDEX_PROFILE_VELOCITY:
    case CIA402_OD_INDEX_PROFILE_ACCELERATION:
    case CIA402_OD_INDEX_PROFILE_DECELERATION:
        if (require_length(data_length, 4U) != CIA402_OD_OK) {
            return CIA402_OD_LENGTH_ERROR;
        }
        u32 = get_u32_le(data);
        if ((u32 == 0U) ||
            ((index == CIA402_OD_INDEX_PROFILE_VELOCITY) &&
             (u32 > od->config.maximum_profile_velocity_q16_16)) ||
            ((index == CIA402_OD_INDEX_PROFILE_ACCELERATION) &&
             (u32 > od->config.maximum_profile_acceleration_q16_16)) ||
            ((index == CIA402_OD_INDEX_PROFILE_DECELERATION) &&
             (u32 > od->config.maximum_profile_deceleration_q16_16))) {
            return CIA402_OD_VALUE_RANGE;
        }
        if (index == CIA402_OD_INDEX_PROFILE_VELOCITY) {
            od->image.profile_velocity_q16_16 = u32;
        } else if (index == CIA402_OD_INDEX_PROFILE_ACCELERATION) {
            od->image.profile_acceleration_q16_16 = u32;
        } else {
            od->image.profile_deceleration_q16_16 = u32;
        }
        return CIA402_OD_OK;

    case CIA402_OD_INDEX_MIT_COMMAND:
        if (!od->config.support_mit) {
            return CIA402_OD_OBJECT_NOT_FOUND;
        }
        return write_mit(od, subindex, data, data_length);

    case CIA402_OD_INDEX_ERROR_CODE:
    case CIA402_OD_INDEX_STATUSWORD:
    case CIA402_OD_INDEX_MODES_DISPLAY:
    case CIA402_OD_INDEX_POSITION_ACTUAL:
    case CIA402_OD_INDEX_VELOCITY_ACTUAL:
    case CIA402_OD_INDEX_TORQUE_ACTUAL:
    case CIA402_OD_INDEX_SUPPORTED_MODES:
        return CIA402_OD_READ_ONLY;
    default:
        return CIA402_OD_OBJECT_NOT_FOUND;
    }
}

cia402_od_result_t cia402_od_write_target_batch(
    cia402_od_t *od,
    const uint8_t *target_position_data,
    size_t target_position_length,
    const uint8_t *target_velocity_data,
    size_t target_velocity_length)
{
    int32_t target_position;
    int32_t target_velocity;

    if ((od == NULL) || (target_position_data == NULL) ||
        (target_velocity_data == NULL)) {
        return CIA402_OD_NULL_ARGUMENT;
    }
    if ((require_length(target_position_length, 4U) != CIA402_OD_OK) ||
        (require_length(target_velocity_length, 4U) != CIA402_OD_OK)) {
        return CIA402_OD_LENGTH_ERROR;
    }

    target_position = get_i32_le(target_position_data);
    target_velocity = get_i32_le(target_velocity_data);
    if (!target_position_is_valid(od, target_position) ||
        !target_velocity_is_valid(od, target_velocity)) {
        return CIA402_OD_VALUE_RANGE;
    }

    od->image.target_position_q16_16 = target_position;
    od->image.target_velocity_q16_16 = target_velocity;
    od->command_sync.target_position_epoch =
        next_epoch(od->command_sync.target_position_epoch);
    od->command_sync.target_velocity_epoch =
        next_epoch(od->command_sync.target_velocity_epoch);
    od->command_sync.target_velocity_mode = od->image.mode_command;
    od->command_sync.target_velocity_mode_instance_epoch =
        od->command_sync.mode_instance_epoch;
    return CIA402_OD_OK;
}

static cia402_od_result_t read_mit(const cia402_od_t *od,
                                   uint8_t subindex,
                                   uint8_t *data,
                                   size_t capacity,
                                   size_t *length)
{
    if (subindex == CIA402_OD_MIT_SUB_COUNT) {
        if (capacity < 1U) {
            return CIA402_OD_LENGTH_ERROR;
        }
        data[0] = CIA402_OD_MIT_HIGHEST_SUBINDEX;
        *length = 1U;
        return CIA402_OD_OK;
    }
    if ((subindex >= CIA402_OD_MIT_SUB_POSITION) &&
        (subindex <= CIA402_OD_MIT_SUB_TORQUE)) {
        int32_t value;
        if (capacity < 4U) {
            return CIA402_OD_LENGTH_ERROR;
        }
        value = (subindex == CIA402_OD_MIT_SUB_POSITION)
                    ? od->image.mit_position_q16_16
                    : ((subindex == CIA402_OD_MIT_SUB_VELOCITY)
                           ? od->image.mit_velocity_q16_16
                           : od->image.mit_torque_q16_16);
        put_i32_le(data, value);
        *length = 4U;
        return CIA402_OD_OK;
    }
    if ((subindex >= CIA402_OD_MIT_SUB_KP) &&
        (subindex <= CIA402_OD_MIT_SUB_SEQUENCE)) {
        uint16_t value;
        if (capacity < 2U) {
            return CIA402_OD_LENGTH_ERROR;
        }
        if (subindex == CIA402_OD_MIT_SUB_KP) {
            value = od->image.mit_kp_uq8_8;
        } else if (subindex == CIA402_OD_MIT_SUB_KD) {
            value = od->image.mit_kd_uq8_8;
        } else if (subindex == CIA402_OD_MIT_SUB_CURRENT_LIMIT) {
            value = od->image.mit_current_limit_uq8_8;
        } else {
            value = od->image.mit_sequence;
        }
        put_u16_le(data, value);
        *length = 2U;
        return CIA402_OD_OK;
    }
    return CIA402_OD_SUBINDEX_NOT_FOUND;
}

cia402_od_result_t cia402_od_read(const cia402_od_t *od,
                                  uint16_t index,
                                  uint8_t subindex,
                                  uint8_t *data,
                                  size_t data_capacity,
                                  size_t *data_length)
{
    uint32_t u32 = 0U;
    uint16_t u16 = 0U;
    int32_t i32 = 0;
    size_t required = 0U;

    if ((od == NULL) || (data == NULL) || (data_length == NULL)) {
        return CIA402_OD_NULL_ARGUMENT;
    }
    *data_length = 0U;
    if ((index != CIA402_OD_INDEX_MIT_COMMAND) && (subindex != 0U)) {
        return CIA402_OD_SUBINDEX_NOT_FOUND;
    }
    if (index == CIA402_OD_INDEX_MIT_COMMAND) {
        if (!od->config.support_mit) {
            return CIA402_OD_OBJECT_NOT_FOUND;
        }
        return read_mit(od, subindex, data, data_capacity, data_length);
    }

    switch (index) {
    case CIA402_OD_INDEX_MODES_OF_OPERATION:
        if (data_capacity < 1U) {
            return CIA402_OD_LENGTH_ERROR;
        }
        data[0] = (uint8_t)od->image.mode_command;
        *data_length = 1U;
        return CIA402_OD_OK;
    case CIA402_OD_INDEX_MODES_DISPLAY:
        if (data_capacity < 1U) {
            return CIA402_OD_LENGTH_ERROR;
        }
        data[0] = (uint8_t)od->image.mode_display;
        *data_length = 1U;
        return CIA402_OD_OK;
    case CIA402_OD_INDEX_ERROR_CODE:
        u16 = od->image.error_code;
        required = 2U;
        break;
    case CIA402_OD_INDEX_CONTROLWORD:
        u16 = od->image.controlword;
        required = 2U;
        break;
    case CIA402_OD_INDEX_STATUSWORD:
        u16 = od->image.statusword;
        required = 2U;
        break;
    case CIA402_OD_INDEX_TARGET_TORQUE:
        u16 = (uint16_t)od->image.target_torque_per_mille;
        required = 2U;
        break;
    case CIA402_OD_INDEX_TORQUE_ACTUAL:
        u16 = (uint16_t)od->image.torque_actual_per_mille;
        required = 2U;
        break;
    case CIA402_OD_INDEX_POSITION_ACTUAL:
        i32 = od->image.position_actual_q16_16;
        required = 4U;
        break;
    case CIA402_OD_INDEX_VELOCITY_ACTUAL:
        i32 = od->image.velocity_actual_q16_16;
        required = 4U;
        break;
    case CIA402_OD_INDEX_TARGET_POSITION:
        i32 = od->image.target_position_q16_16;
        required = 4U;
        break;
    case CIA402_OD_INDEX_TARGET_VELOCITY:
        i32 = od->image.target_velocity_q16_16;
        required = 4U;
        break;
    case CIA402_OD_INDEX_PROFILE_VELOCITY:
        u32 = od->image.profile_velocity_q16_16;
        required = 4U;
        break;
    case CIA402_OD_INDEX_PROFILE_ACCELERATION:
        u32 = od->image.profile_acceleration_q16_16;
        required = 4U;
        break;
    case CIA402_OD_INDEX_PROFILE_DECELERATION:
        u32 = od->image.profile_deceleration_q16_16;
        required = 4U;
        break;
    case CIA402_OD_INDEX_SUPPORTED_MODES:
        u32 = od->image.supported_modes;
        required = 4U;
        break;
    default:
        return CIA402_OD_OBJECT_NOT_FOUND;
    }

    if (data_capacity < required) {
        return CIA402_OD_LENGTH_ERROR;
    }
    if (required == 2U) {
        put_u16_le(data, u16);
    } else if ((index == CIA402_OD_INDEX_POSITION_ACTUAL) ||
               (index == CIA402_OD_INDEX_VELOCITY_ACTUAL) ||
               (index == CIA402_OD_INDEX_TARGET_POSITION) ||
               (index == CIA402_OD_INDEX_TARGET_VELOCITY)) {
        put_i32_le(data, i32);
    } else {
        put_u32_le(data, u32);
    }
    *data_length = required;
    return CIA402_OD_OK;
}

void cia402_od_publish_feedback(cia402_od_t *od,
                                const cia402_od_feedback_t *feedback)
{
    if ((od == NULL) || (feedback == NULL)) {
        return;
    }
    od->image.error_code = feedback->error_code;
    od->image.statusword = feedback->statusword;
    od->image.mode_display = feedback->mode_display;
    od->image.position_actual_q16_16 =
        feedback->position_actual_q16_16;
    od->image.velocity_actual_q16_16 =
        feedback->velocity_actual_q16_16;
    od->image.torque_actual_per_mille =
        feedback->torque_actual_per_mille;
}
