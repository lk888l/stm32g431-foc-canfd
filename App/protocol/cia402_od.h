#ifndef APP_PROTOCOL_CIA402_OD_H
#define APP_PROTOCOL_CIA402_OD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Portable object image; a CANopen stack adapter may bind these indices. */
enum {
    CIA402_OD_INDEX_ERROR_CODE = 0x603FU,
    CIA402_OD_INDEX_CONTROLWORD = 0x6040U,
    CIA402_OD_INDEX_STATUSWORD = 0x6041U,
    CIA402_OD_INDEX_MODES_OF_OPERATION = 0x6060U,
    CIA402_OD_INDEX_MODES_DISPLAY = 0x6061U,
    CIA402_OD_INDEX_POSITION_ACTUAL = 0x6064U,
    CIA402_OD_INDEX_VELOCITY_ACTUAL = 0x606CU,
    CIA402_OD_INDEX_TARGET_TORQUE = 0x6071U,
    CIA402_OD_INDEX_TORQUE_ACTUAL = 0x6077U,
    CIA402_OD_INDEX_TARGET_POSITION = 0x607AU,
    CIA402_OD_INDEX_PROFILE_VELOCITY = 0x6081U,
    CIA402_OD_INDEX_PROFILE_ACCELERATION = 0x6083U,
    CIA402_OD_INDEX_PROFILE_DECELERATION = 0x6084U,
    CIA402_OD_INDEX_TARGET_VELOCITY = 0x60FFU,
    CIA402_OD_INDEX_SUPPORTED_MODES = 0x6502U,
    /* Manufacturer-specific MIT command record, subindices documented below. */
    CIA402_OD_INDEX_MIT_COMMAND = 0x2000U
};

enum {
    CIA402_OD_MIT_SUB_COUNT = 0,
    CIA402_OD_MIT_SUB_POSITION = 1,
    CIA402_OD_MIT_SUB_VELOCITY = 2,
    CIA402_OD_MIT_SUB_TORQUE = 3,
    CIA402_OD_MIT_SUB_KP = 4,
    CIA402_OD_MIT_SUB_KD = 5,
    CIA402_OD_MIT_SUB_CURRENT_LIMIT = 6,
    CIA402_OD_MIT_SUB_SEQUENCE = 7,
    CIA402_OD_MIT_HIGHEST_SUBINDEX = 7
};

typedef enum {
    CIA402_OD_OK = 0,
    CIA402_OD_NULL_ARGUMENT,
    CIA402_OD_OBJECT_NOT_FOUND,
    CIA402_OD_SUBINDEX_NOT_FOUND,
    CIA402_OD_READ_ONLY,
    CIA402_OD_LENGTH_ERROR,
    CIA402_OD_VALUE_RANGE,
    CIA402_OD_STATE_CONFLICT
} cia402_od_result_t;

typedef struct {
    int32_t minimum_position_q16_16;
    int32_t maximum_position_q16_16;
    int32_t maximum_abs_velocity_q16_16;
    int32_t maximum_abs_mit_torque_q16_16;
    int16_t maximum_abs_target_torque_per_mille;
    uint32_t maximum_profile_velocity_q16_16;
    uint32_t maximum_profile_acceleration_q16_16;
    uint32_t maximum_profile_deceleration_q16_16;
    uint16_t maximum_mit_kp_uq8_8;
    uint16_t maximum_mit_kd_uq8_8;
    uint16_t maximum_mit_current_uq8_8;
    bool support_pp;
    bool support_pv;
    bool support_mit;
} cia402_od_config_t;

typedef struct {
    uint16_t error_code;
    uint16_t controlword;
    uint16_t statusword;
    int8_t mode_command;
    int8_t mode_display;
    int32_t position_actual_q16_16;
    int32_t velocity_actual_q16_16;
    int16_t target_torque_per_mille;
    int16_t torque_actual_per_mille;
    int32_t target_position_q16_16;
    uint32_t profile_velocity_q16_16;
    uint32_t profile_acceleration_q16_16;
    uint32_t profile_deceleration_q16_16;
    int32_t target_velocity_q16_16;
    uint32_t supported_modes;
    int32_t mit_position_q16_16;
    int32_t mit_velocity_q16_16;
    int32_t mit_torque_q16_16;
    uint16_t mit_kp_uq8_8;
    uint16_t mit_kd_uq8_8;
    uint16_t mit_current_limit_uq8_8;
    uint16_t mit_sequence;
} cia402_od_image_t;

/* Command-generation metadata is kept outside the public OD image.  It makes
 * command edges versioned transactions instead of relying on whichever OD
 * values happen to be visible when the 1 kHz supervisor next runs. */
typedef struct {
    uint32_t mode_instance_epoch;
    uint32_t target_position_epoch;
    uint32_t target_velocity_epoch;
    uint32_t pp_request_epoch;
    uint32_t pp_request_target_position_epoch;
    int32_t pp_request_target_position_q16_16;
    uint16_t pp_request_controlword;
    int8_t pp_request_mode;
    uint32_t pp_request_mode_instance_epoch;
    int8_t target_velocity_mode;
    uint32_t target_velocity_mode_instance_epoch;
} cia402_od_command_sync_t;

typedef struct {
    cia402_od_config_t config;
    cia402_od_image_t image;
    cia402_od_command_sync_t command_sync;
} cia402_od_t;

typedef struct {
    uint16_t error_code;
    uint16_t statusword;
    int8_t mode_display;
    int32_t position_actual_q16_16;
    int32_t velocity_actual_q16_16;
    int16_t torque_actual_per_mille;
} cia402_od_feedback_t;

bool cia402_od_init(cia402_od_t *od, const cia402_od_config_t *config);

/*
 * Byte access is explicitly little-endian. Callers must serialize access with
 * the control task (or copy the image in a short critical section).
 */
cia402_od_result_t cia402_od_write(cia402_od_t *od,
                                   uint16_t index,
                                   uint8_t subindex,
                                   const uint8_t *data,
                                   size_t data_length,
                                   bool mode_change_inhibited);

/* Validate and commit the fixed RPDO2 pair (0x607A, 0x60FF) as one
 * transaction.  Neither value nor either epoch changes if any field is
 * malformed or out of range. */
cia402_od_result_t cia402_od_write_target_batch(
    cia402_od_t *od,
    const uint8_t *target_position_data,
    size_t target_position_length,
    const uint8_t *target_velocity_data,
    size_t target_velocity_length);

cia402_od_result_t cia402_od_read(const cia402_od_t *od,
                                  uint16_t index,
                                  uint8_t subindex,
                                  uint8_t *data,
                                  size_t data_capacity,
                                  size_t *data_length);

void cia402_od_publish_feedback(cia402_od_t *od,
                                const cia402_od_feedback_t *feedback);

#ifdef __cplusplus
}
#endif

#endif /* APP_PROTOCOL_CIA402_OD_H */
