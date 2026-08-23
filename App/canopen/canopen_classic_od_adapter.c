#include "canopen_classic_od_adapter.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "cia402_od.h"

/* Kept as explicit declarations to avoid coupling CANopenNode to app internals. */
extern cia402_od_result_t app_cia402_od_write(uint16_t index,
                                              uint8_t subindex,
                                              const uint8_t *data,
                                              size_t data_length);
extern cia402_od_result_t app_cia402_od_read(uint16_t index,
                                             uint8_t subindex,
                                             uint8_t *data,
                                             size_t data_capacity,
                                             size_t *data_length);
extern cia402_od_result_t app_cia402_od_write_target_batch(
    const uint8_t *target_position_data,
    size_t target_position_length,
    const uint8_t *target_velocity_data,
    size_t target_velocity_length);
extern void app_note_canopen_protocol_fault(void);

static const uint16_t boundIndices[] = {
    0x2000U, 0x603FU, 0x6040U, 0x6041U, 0x6060U, 0x6061U,
    0x6064U, 0x606CU, 0x6071U, 0x6077U, 0x607AU, 0x6081U,
    0x6083U, 0x6084U, 0x60FFU, 0x6502U,
};

static OD_extension_t extensions[sizeof(boundIndices) / sizeof(boundIndices[0])];
static volatile bool controlPermission;

typedef struct {
    bool processActive;
    bool positionSeen;
    uint8_t positionData[4];
    OD_stream_t positionStream;
    uint32_t cycleRejections;
} rpdo_target_stage_t;

static rpdo_target_stage_t rpdoTargetStage;

static ODR_t translate_result(cia402_od_result_t result)
{
    switch (result) {
    case CIA402_OD_OK:
        return ODR_OK;
    case CIA402_OD_NULL_ARGUMENT:
        return ODR_GENERAL;
    case CIA402_OD_OBJECT_NOT_FOUND:
        return ODR_IDX_NOT_EXIST;
    case CIA402_OD_SUBINDEX_NOT_FOUND:
        return ODR_SUB_NOT_EXIST;
    case CIA402_OD_READ_ONLY:
        return ODR_READONLY;
    case CIA402_OD_LENGTH_ERROR:
        return ODR_TYPE_MISMATCH;
    case CIA402_OD_VALUE_RANGE:
        return ODR_INVALID_VALUE;
    case CIA402_OD_STATE_CONFLICT:
        return ODR_DATA_DEV_STATE;
    default:
        return ODR_GENERAL;
    }
}

static void reject_rpdo_target_stage(void)
{
    rpdoTargetStage.positionSeen = false;
    rpdoTargetStage.cycleRejections++;
    app_note_canopen_protocol_fault();
}

static ODR_t commit_original(OD_stream_t *stream,
                             const uint8_t *data,
                             OD_size_t count)
{
    OD_stream_t copy = *stream;
    OD_size_t written = 0U;
    ODR_t result;

    copy.dataOffset = 0U;
    result = OD_writeOriginal(&copy, data, count, &written);
    if ((result != ODR_OK) || (written != count)) {
        return (result == ODR_OK) ? ODR_GENERAL : result;
    }
    return ODR_OK;
}

static ODR_t write_rpdo_target_pair(OD_stream_t *stream,
                                    const void *buffer,
                                    OD_size_t count,
                                    OD_size_t *countWritten)
{
    cia402_od_result_t result;
    ODR_t originalResult;

    if (count != 4U) {
        reject_rpdo_target_stage();
        return ODR_TYPE_MISMATCH;
    }

    if (stream->index == CIA402_OD_INDEX_TARGET_POSITION) {
        if (rpdoTargetStage.positionSeen) {
            reject_rpdo_target_stage();
            return ODR_GENERAL;
        }
        memcpy(rpdoTargetStage.positionData, buffer, 4U);
        rpdoTargetStage.positionStream = *stream;
        rpdoTargetStage.positionSeen = true;
        *countWritten = count;
        return ODR_OK;
    }

    if ((stream->index != CIA402_OD_INDEX_TARGET_VELOCITY) ||
        !rpdoTargetStage.positionSeen) {
        reject_rpdo_target_stage();
        return ODR_GENERAL;
    }

    result = app_cia402_od_write_target_batch(rpdoTargetStage.positionData,
                                               4U,
                                               (const uint8_t *)buffer,
                                               4U);
    if (result != CIA402_OD_OK) {
        reject_rpdo_target_stage();
        return translate_result(result);
    }

    /* Both original writes are structurally fixed by the static OD.  They
     * happen only after application validation so the diagnostic backing
     * image cannot expose a half-valid command to control logic. */
    originalResult = commit_original(&rpdoTargetStage.positionStream,
                                     rpdoTargetStage.positionData,
                                     4U);
    if (originalResult == ODR_OK) {
        originalResult = commit_original(stream,
                                         (const uint8_t *)buffer,
                                         4U);
    }
    rpdoTargetStage.positionSeen = false;
    if (originalResult != ODR_OK) {
        rpdoTargetStage.cycleRejections++;
        app_note_canopen_protocol_fault();
        return originalResult;
    }

    *countWritten = count;
    return ODR_OK;
}

static ODR_t read_application(OD_stream_t *stream,
                              void *buffer,
                              OD_size_t count,
                              OD_size_t *countRead)
{
    size_t length = 0U;
    cia402_od_result_t result;

    if ((stream == NULL) || (buffer == NULL) || (countRead == NULL)) {
        return ODR_GENERAL;
    }
    *countRead = 0U;
    if (stream->dataOffset != 0U) {
        return ODR_TYPE_MISMATCH;
    }
    result = app_cia402_od_read(stream->index,
                                stream->subIndex,
                                (uint8_t *)buffer,
                                (size_t)count,
                                &length);
    if (result != CIA402_OD_OK) {
        return translate_result(result);
    }
    if (length > (size_t)count) {
        return ODR_DATA_LONG;
    }
    *countRead = (OD_size_t)length;
    return ODR_OK;
}

static ODR_t write_application(OD_stream_t *stream,
                               const void *buffer,
                               OD_size_t count,
                               OD_size_t *countWritten)
{
    cia402_od_result_t result;
    OD_size_t originalWritten = 0U;
    ODR_t originalResult;

    if ((stream == NULL) || (buffer == NULL) || (countWritten == NULL)) {
        return ODR_GENERAL;
    }
    *countWritten = 0U;
    if ((stream->dataOffset != 0U) || (count != stream->dataLength)) {
        return ODR_TYPE_MISMATCH;
    }
    if (rpdoTargetStage.processActive &&
        ((stream->index == CIA402_OD_INDEX_TARGET_POSITION) ||
         (stream->index == CIA402_OD_INDEX_TARGET_VELOCITY))) {
        return write_rpdo_target_pair(stream, buffer, count, countWritten);
    }
    if ((stream->index == CIA402_OD_INDEX_CONTROLWORD) &&
        (count == 2U) && !controlPermission &&
        ((((const uint8_t *)buffer)[0] != 0U) ||
         (((const uint8_t *)buffer)[1] != 0U))) {
        return ODR_DATA_DEV_STATE;
    }
    result = app_cia402_od_write(stream->index,
                                 stream->subIndex,
                                 (const uint8_t *)buffer,
                                 (size_t)count);
    if (result != CIA402_OD_OK) {
        return translate_result(result);
    }

    /* Preserve a coherent backing image for diagnostics and odOrig access.
     * Control logic still consumes the validated application image. */
    originalResult = OD_writeOriginal(stream, buffer, count, &originalWritten);
    if ((originalResult != ODR_OK) || (originalWritten != count)) {
        return (originalResult == ODR_OK) ? ODR_GENERAL : originalResult;
    }
    *countWritten = count;
    return ODR_OK;
}

bool canopen_classic_od_bind(OD_t *od)
{
    if (od == NULL) {
        return false;
    }
    memset(extensions, 0, sizeof(extensions));
    memset(&rpdoTargetStage, 0, sizeof(rpdoTargetStage));
    controlPermission = false;
    for (size_t index = 0U; index < (sizeof(boundIndices) / sizeof(boundIndices[0])); ++index) {
        OD_entry_t *entry = OD_find(od, boundIndices[index]);
        if (entry == NULL) {
            return false;
        }
        extensions[index].object = NULL;
        extensions[index].read = read_application;
        extensions[index].write = write_application;
        if (OD_extension_init(entry, &extensions[index]) != ODR_OK) {
            return false;
        }
    }
    return true;
}

void canopen_classic_od_rpdo_process_begin(void)
{
    if (rpdoTargetStage.processActive || rpdoTargetStage.positionSeen) {
        reject_rpdo_target_stage();
    }
    rpdoTargetStage.processActive = true;
    rpdoTargetStage.positionSeen = false;
    rpdoTargetStage.cycleRejections = 0U;
}

uint32_t canopen_classic_od_rpdo_process_end(void)
{
    if (rpdoTargetStage.positionSeen) {
        reject_rpdo_target_stage();
    }
    rpdoTargetStage.processActive = false;
    return rpdoTargetStage.cycleRejections;
}

void canopen_classic_od_set_control_permission(bool permitted)
{
    controlPermission = permitted;
}

bool canopen_classic_od_force_disable(void)
{
    static const uint8_t disabledControlword[2] = {0U, 0U};
    return app_cia402_od_write(CIA402_OD_INDEX_CONTROLWORD,
                               0U,
                               disabledControlword,
                               sizeof(disabledControlword)) == CIA402_OD_OK;
}
