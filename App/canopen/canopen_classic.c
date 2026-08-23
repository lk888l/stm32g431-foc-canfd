#include "canopen_classic.h"

#include <string.h>

#include "301/CO_NMT_Heartbeat.h"
#include "CANopen.h"
#include "OD.h"
#include "canopen_classic_od_adapter.h"

#define CANOPEN_FIRST_HEARTBEAT_MS UINT16_C(250)
#define CANOPEN_SDO_TIMEOUT_MS UINT16_C(500)
/* 0x1016 represents milliseconds in 16 bits.  Clamping only above its
 * maximum preserves every legal heartbeat timeout after a stalled task while
 * preventing an extreme elapsed value from wrapping stack timers. */
#define CANOPEN_BACKGROUND_MAX_ELAPSED_US UINT32_C(65535000)
#define CANOPEN_DRIVE_FAULT_ERROR_BIT UINT8_C(0x40)

#if CO_CONFIG_EM_ERR_STATUS_BITS_COUNT <= CANOPEN_DRIVE_FAULT_ERROR_BIT
#error "CANopen EMCY status-bit storage does not include the drive-fault bit"
#endif

static struct {
    CO_t *co;
    FDCAN_HandleTypeDef *fdcan;
    uint8_t nodeId;
    uint16_t bitrateKbit;
    canopen_classic_status_t status;
    bool heartbeatFaultPending;
    volatile bool controlPermission;
    volatile uint32_t reportedDriveFaults;
    volatile uint16_t reportedDriveErrorCode;
    volatile bool initialized;
} classic;

extern void canopen_classic_driver_poll(CO_CANmodule_t *CANmodule);

static void revoke_control_permission(void)
{
    const bool wasPermitted = classic.controlPermission;

    /* Close the OD write gate first.  The deferred 1 kHz PDO context may
     * pre-empt background processing, so this ordering can only reject an
     * extra command, never admit one after supervision has been revoked. */
    canopen_classic_od_set_control_permission(false);
    classic.controlPermission = false;
    if (wasPermitted) {
        (void)canopen_classic_od_force_disable();
    }
}

static void heartbeat_failed(uint8_t nodeId, uint8_t index, void *object)
{
    (void)nodeId;
    (void)index;
    (void)object;
    classic.heartbeatFaultPending = true;
    classic.status.heartbeat_active = false;
    revoke_control_permission();
}

static void heartbeat_started(uint8_t nodeId, uint8_t index, void *object)
{
    (void)nodeId;
    (void)index;
    (void)object;
    classic.heartbeatFaultPending = false;
}

static void heartbeat_nmt_changed(uint8_t nodeId,
                                  uint8_t index,
                                  CO_NMT_internalState_t nmtState,
                                  void *object)
{
    (void)nodeId;
    (void)index;
    (void)object;
    if (nmtState != CO_NMT_OPERATIONAL) {
        classic.status.heartbeat_active = false;
        revoke_control_permission();
    }
}

static canopen_classic_state_t translate_nmt_state(CO_NMT_internalState_t state)
{
    switch (state) {
    case CO_NMT_OPERATIONAL:
        return CANOPEN_CLASSIC_STATE_OPERATIONAL;
    case CO_NMT_STOPPED:
        return CANOPEN_CLASSIC_STATE_STOPPED;
    case CO_NMT_PRE_OPERATIONAL:
    case CO_NMT_INITIALIZING:
    default:
        return CANOPEN_CLASSIC_STATE_PRE_OPERATIONAL;
    }
}

static void refresh_control_permission(void)
{
    CO_NMT_internalState_t remoteNmtState = CO_NMT_UNKNOWN;
    const CO_HBconsumer_state_t heartbeatState =
        CO_HBconsumer_getState(classic.co->HBcons, 0U);
    const bool heartbeatConfigured =
        heartbeatState != CO_HBconsumer_UNCONFIGURED;
    const bool heartbeatActive =
        (heartbeatState == CO_HBconsumer_ACTIVE) &&
        (CO_HBconsumer_getNmtState(classic.co->HBcons,
                                   0U,
                                   &remoteNmtState) == 0) &&
        (remoteNmtState == CO_NMT_OPERATIONAL);
    const bool permitted =
        (classic.status.state == CANOPEN_CLASSIC_STATE_OPERATIONAL) &&
        heartbeatConfigured && heartbeatActive &&
        !classic.heartbeatFaultPending &&
        !classic.status.application_reset_requested;

    classic.status.heartbeat_configured = heartbeatConfigured;
    classic.status.heartbeat_active = heartbeatActive;
    if (!permitted) {
        revoke_control_permission();
    } else {
        classic.controlPermission = true;
        canopen_classic_od_set_control_permission(true);
    }
}

static bool configure_communication(void)
{
    CO_ReturnError_t error;
    uint32_t errorInfo = 0U;
    const uint16_t nmtControl = CO_NMT_ERR_ON_BUSOFF_HB;

    CO_CANsetConfigurationMode(classic.fdcan);
    error = CO_CANinit(classic.co, classic.fdcan, classic.bitrateKbit);
    if (error != CO_ERROR_NO) {
        classic.status.stack_error = (int32_t)error;
        classic.status.state = CANOPEN_CLASSIC_STATE_ERROR;
        return false;
    }
    error = CO_CANopenInit(classic.co,
                           NULL,
                           NULL,
                           OD,
                           NULL,
                           nmtControl,
                           CANOPEN_FIRST_HEARTBEAT_MS,
                           CANOPEN_SDO_TIMEOUT_MS,
                           CANOPEN_SDO_TIMEOUT_MS,
                           false,
                           classic.nodeId,
                           &errorInfo);
    if (error != CO_ERROR_NO) {
        classic.status.initialization_error_info = errorInfo;
        classic.status.stack_error = (int32_t)error;
        classic.status.state = CANOPEN_CLASSIC_STATE_ERROR;
        return false;
    }
    if (!canopen_classic_od_bind(OD)) {
        classic.status.initialization_error_info = UINT32_C(0xFFFFFFFF);
        classic.status.stack_error = (int32_t)CO_ERROR_OD_PARAMETERS;
        classic.status.state = CANOPEN_CLASSIC_STATE_ERROR;
        return false;
    }
    CO_HBconsumer_initCallbackHeartbeatStarted(classic.co->HBcons,
                                                0U,
                                                NULL,
                                                heartbeat_started);
    CO_HBconsumer_initCallbackTimeout(classic.co->HBcons,
                                      0U,
                                      NULL,
                                      heartbeat_failed);
    CO_HBconsumer_initCallbackRemoteReset(classic.co->HBcons,
                                          0U,
                                          NULL,
                                          heartbeat_failed);
    CO_HBconsumer_initCallbackNmtChanged(classic.co->HBcons,
                                         0U,
                                         NULL,
                                         heartbeat_nmt_changed);
    error = CO_CANopenInitPDO(classic.co,
                              classic.co->em,
                              OD,
                              classic.nodeId,
                              &errorInfo);
    if (error != CO_ERROR_NO) {
        classic.status.initialization_error_info = errorInfo;
        classic.status.stack_error = (int32_t)error;
        classic.status.state = CANOPEN_CLASSIC_STATE_ERROR;
        return false;
    }

    classic.heartbeatFaultPending = false;
    classic.controlPermission = false;
    classic.reportedDriveFaults = 0U;
    classic.reportedDriveErrorCode = 0U;
    classic.status.reported_drive_faults = 0U;
    classic.status.rpdo_transaction_rejections = 0U;
    classic.status.heartbeat_configured = false;
    classic.status.heartbeat_active = false;
    canopen_classic_od_set_control_permission(false);
    CO_CANsetNormalMode(classic.co->CANmodule);
    classic.status.initialization_error_info = errorInfo;
    classic.status.stack_error = (int32_t)CO_ERROR_NO;
    classic.status.state = CANOPEN_CLASSIC_STATE_PRE_OPERATIONAL;
    return true;
}

bool canopen_classic_init(FDCAN_HandleTypeDef *fdcan,
                          uint8_t node_id,
                          uint16_t bitrate_kbit)
{
    if ((fdcan == NULL) || (node_id < 1U) || (node_id > 127U) ||
        (bitrate_kbit != CANOPEN_CLASSIC_REQUIRED_BITRATE_KBIT)) {
        return false;
    }

    memset(&classic, 0, sizeof(classic));
    classic.fdcan = fdcan;
    classic.nodeId = node_id;
    classic.bitrateKbit = bitrate_kbit;
    classic.co = CO_new(NULL, NULL);
    if (classic.co == NULL) {
        classic.status.stack_error = (int32_t)CO_ERROR_OUT_OF_MEMORY;
        classic.status.state = CANOPEN_CLASSIC_STATE_ERROR;
        return false;
    }
    classic.initialized = configure_communication();
    return classic.initialized;
}

void canopen_classic_process_1ms(void)
{
    if (!classic.initialized || classic.status.application_reset_requested) {
        return;
    }
    canopen_classic_od_rpdo_process_begin();
    CO_process_RPDO(classic.co, false, UINT32_C(1000), NULL);
    classic.status.rpdo_transaction_rejections +=
        canopen_classic_od_rpdo_process_end();
    CO_process_TPDO(classic.co, false, UINT32_C(1000), NULL);
    canopen_classic_driver_poll(classic.co->CANmodule);
}

void canopen_classic_background(uint32_t elapsed_us)
{
    CO_NMT_reset_cmd_t reset;

    if (!classic.initialized || classic.status.application_reset_requested) {
        return;
    }
    if (elapsed_us > CANOPEN_BACKGROUND_MAX_ELAPSED_US) {
        elapsed_us = CANOPEN_BACKGROUND_MAX_ELAPSED_US;
    }
    canopen_classic_driver_poll(classic.co->CANmodule);
    reset = CO_process(classic.co, false, elapsed_us, NULL);
    if (reset == CO_RESET_COMM) {
        revoke_control_permission();
        classic.initialized = false;
        classic.status.communication_resets++;
        classic.initialized = configure_communication();
    } else if (reset == CO_RESET_APP) {
        /* The motor application owns the ordered gate-off and MCU reset. */
        revoke_control_permission();
        classic.status.application_reset_requested = true;
    }
    if (classic.initialized) {
        classic.status.state = translate_nmt_state(
            CO_NMT_getInternalState(classic.co->NMT));
        refresh_control_permission();
    }
    canopen_classic_driver_poll(classic.co->CANmodule);
}

void canopen_classic_report_drive_faults(uint32_t effective_faults,
                                         uint16_t cia402_error_code)
{
    uint16_t emergencyCode = cia402_error_code;

    if (!classic.initialized || (classic.co == NULL) ||
        (classic.co->em == NULL)) {
        return;
    }
    if ((effective_faults != 0U) && (emergencyCode == 0U)) {
        emergencyCode = CO_EMC_DEVICE_SPECIFIC;
    }
    if ((effective_faults == classic.reportedDriveFaults) &&
        ((effective_faults == 0U) ||
         (emergencyCode == classic.reportedDriveErrorCode))) {
        return;
    }

    /* One manufacturer status bit represents the complete application fault
     * mask.  Re-arm it when the nonzero mask/code changes so the next EMCY
     * carries the current 32-bit mask instead of stale first-fault data. */
    if (classic.reportedDriveFaults != 0U) {
        CO_errorReset(classic.co->em,
                      CANOPEN_DRIVE_FAULT_ERROR_BIT,
                      classic.reportedDriveFaults);
    }
    if (effective_faults != 0U) {
        CO_errorReport(classic.co->em,
                       CANOPEN_DRIVE_FAULT_ERROR_BIT,
                       emergencyCode,
                       effective_faults);
    }
    classic.reportedDriveFaults = effective_faults;
    classic.reportedDriveErrorCode =
        (effective_faults == 0U) ? 0U : emergencyCode;
    classic.status.reported_drive_faults = effective_faults;
}

canopen_classic_status_t canopen_classic_status(void)
{
    return classic.status;
}

bool canopen_classic_is_operational(void)
{
    return classic.initialized &&
           (classic.status.state == CANOPEN_CLASSIC_STATE_OPERATIONAL) &&
           classic.status.heartbeat_configured &&
           classic.status.heartbeat_active &&
           classic.controlPermission &&
           !classic.status.application_reset_requested;
}
