/* CANopenNode v4 classic-CAN driver over STM32G4 FDCAN. */
#include "301/CO_driver.h"

#include <string.h>

#include "stm32g4xx_hal.h"
#include "bsp.h"

#define CANOPEN_CLASSIC_BITRATE_KBIT UINT16_C(500)
#define CANOPEN_CLASSIC_IDENT_MASK UINT16_C(0x07FF)
#define CANOPEN_CLASSIC_RTR_BIT UINT16_C(0x0800)

static CO_CANmodule_t *activeModule;

uint32_t canopen_classic_irq_lock(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    __DMB();
    return primask;
}

void canopen_classic_irq_unlock(uint32_t previousPrimask)
{
    __DMB();
    __set_PRIMASK(previousPrimask);
}

static FDCAN_HandleTypeDef *fdcan_handle(const CO_CANmodule_t *module)
{
    return (module == NULL) ? NULL : (FDCAN_HandleTypeDef *)module->CANptr;
}

static bool fdcan_send(CO_CANmodule_t *module, const CO_CANtx_t *buffer)
{
    FDCAN_TxHeaderTypeDef header = {0};
    FDCAN_HandleTypeDef *fdcan = fdcan_handle(module);
    bool sent;

    if ((fdcan == NULL) || (buffer == NULL) || !module->CANnormal ||
        (buffer->DLC > 8U) || !bsp_fdcan_tx_try_lock()) {
        return false;
    }

    if (HAL_FDCAN_GetTxFifoFreeLevel(fdcan) == 0U) {
        bsp_fdcan_tx_unlock();
        return false;
    }

    header.Identifier = (uint32_t)(buffer->ident & CANOPEN_CLASSIC_IDENT_MASK);
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = buffer->rtr ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
    header.DataLength = (uint32_t)buffer->DLC;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0U;

    sent = HAL_FDCAN_AddMessageToTxFifoQ(fdcan, &header, buffer->data) == HAL_OK;
    bsp_fdcan_tx_unlock();
    return sent;
}

static void drain_pending_tx(CO_CANmodule_t *module)
{
    if ((module == NULL) || !module->CANnormal || (module->CANtxCount == 0U)) {
        return;
    }

    CO_LOCK_CAN_SEND(module);
    for (uint16_t index = 0U; index < module->txSize; ++index) {
        CO_CANtx_t *buffer = &module->txArray[index];
        if (!buffer->bufferFull) {
            continue;
        }
        if (!fdcan_send(module, buffer)) {
            break; /* Preserve CANopenNode's priority-by-buffer-index order. */
        }
        buffer->bufferFull = false;
        module->CANtxCount--;
        module->bufferInhibitFlag = buffer->syncFlag;
        module->firstCANtxMessage = false;
        /* Bound the all-interrupt-masked CO_LOCK_CAN_SEND section.  The
         * cooperative driver is polled several times per millisecond, so one
         * queued frame per call preserves deterministic current-loop latency
         * without starving CANopen traffic. */
        break;
    }
    CO_UNLOCK_CAN_SEND(module);
}

void CO_CANsetConfigurationMode(void *CANptr)
{
    /* FDCAN1 is shared with a vendor-specific FD data plane.  Its timing and
     * message RAM are owned by the BSP, so a CANopen communication reset must
     * not stop or reconfigure the peripheral. */
    if ((activeModule != NULL) && (activeModule->CANptr == CANptr)) {
        activeModule->CANnormal = false;
    }
}

void CO_CANsetNormalMode(CO_CANmodule_t *CANmodule)
{
    if (CANmodule != NULL) {
        CANmodule->CANnormal = true;
        activeModule = CANmodule;
        drain_pending_tx(CANmodule);
    }
}

CO_ReturnError_t CO_CANmodule_init(CO_CANmodule_t *CANmodule,
                                   void *CANptr,
                                   CO_CANrx_t rxArray[],
                                   uint16_t rxSize,
                                   CO_CANtx_t txArray[],
                                   uint16_t txSize,
                                   uint16_t CANbitRate)
{
    if ((CANmodule == NULL) || (CANptr == NULL) || (rxArray == NULL) ||
        (txArray == NULL) || (rxSize == 0U) || (txSize == 0U) ||
        (CANbitRate != CANOPEN_CLASSIC_BITRATE_KBIT)) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    memset(CANmodule, 0, sizeof(*CANmodule));
    CANmodule->CANptr = CANptr;
    CANmodule->rxArray = rxArray;
    CANmodule->rxSize = rxSize;
    CANmodule->txArray = txArray;
    CANmodule->txSize = txSize;
    /* The shared BSP accepts all 11-bit data frames into FIFO0. */
    CANmodule->useCANrxFilters = false;
    CANmodule->firstCANtxMessage = true;

    for (uint16_t index = 0U; index < rxSize; ++index) {
        rxArray[index].ident = 0U;
        rxArray[index].mask = UINT16_MAX;
        rxArray[index].object = NULL;
        rxArray[index].CANrx_callback = NULL;
    }
    for (uint16_t index = 0U; index < txSize; ++index) {
        memset(&txArray[index], 0, sizeof(txArray[index]));
    }

    activeModule = CANmodule;
    return CO_ERROR_NO;
}

void CO_CANmodule_disable(CO_CANmodule_t *CANmodule)
{
    if (CANmodule != NULL) {
        CANmodule->CANnormal = false;
    }
}

CO_ReturnError_t CO_CANrxBufferInit(CO_CANmodule_t *CANmodule,
                                    uint16_t index,
                                    uint16_t ident,
                                    uint16_t mask,
                                    bool_t rtr,
                                    void *object,
                                    void (*CANrx_callback)(void *object,
                                                           void *message))
{
    if ((CANmodule == NULL) || (index >= CANmodule->rxSize) ||
        (object == NULL) || (CANrx_callback == NULL)) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    CO_CANrx_t *buffer = &CANmodule->rxArray[index];
    buffer->object = object;
    buffer->CANrx_callback = CANrx_callback;
    buffer->ident = (uint16_t)(ident & CANOPEN_CLASSIC_IDENT_MASK);
    if (rtr) {
        buffer->ident |= CANOPEN_CLASSIC_RTR_BIT;
    }
    buffer->mask = (uint16_t)((mask & CANOPEN_CLASSIC_IDENT_MASK) |
                              CANOPEN_CLASSIC_RTR_BIT);
    return CO_ERROR_NO;
}

CO_CANtx_t *CO_CANtxBufferInit(CO_CANmodule_t *CANmodule,
                               uint16_t index,
                               uint16_t ident,
                               bool_t rtr,
                               uint8_t noOfBytes,
                               bool_t syncFlag)
{
    if ((CANmodule == NULL) || (index >= CANmodule->txSize) ||
        (ident > CANOPEN_CLASSIC_IDENT_MASK) || (noOfBytes > 8U)) {
        return NULL;
    }

    CO_CANtx_t *buffer = &CANmodule->txArray[index];
    buffer->ident = ident;
    buffer->DLC = noOfBytes;
    buffer->rtr = rtr;
    buffer->bufferFull = false;
    buffer->syncFlag = syncFlag;
    return buffer;
}

CO_ReturnError_t CO_CANsend(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer)
{
    CO_ReturnError_t result = CO_ERROR_NO;

    if ((CANmodule == NULL) || (buffer == NULL)) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }
    if (buffer->bufferFull) {
        if (!CANmodule->firstCANtxMessage) {
            CANmodule->CANerrorStatus |= CO_CAN_ERRTX_OVERFLOW;
        }
        result = CO_ERROR_TX_OVERFLOW;
    }

    CO_LOCK_CAN_SEND(CANmodule);
    if (!buffer->bufferFull && fdcan_send(CANmodule, buffer)) {
        CANmodule->bufferInhibitFlag = buffer->syncFlag;
        CANmodule->firstCANtxMessage = false;
    } else if (!buffer->bufferFull) {
        buffer->bufferFull = true;
        CANmodule->CANtxCount++;
    }
    CO_UNLOCK_CAN_SEND(CANmodule);
    return result;
}

void CO_CANclearPendingSyncPDOs(CO_CANmodule_t *CANmodule)
{
    bool_t deleted = false;

    if (CANmodule == NULL) {
        return;
    }
    CO_LOCK_CAN_SEND(CANmodule);
    for (uint16_t index = 0U; index < CANmodule->txSize; ++index) {
        CO_CANtx_t *buffer = &CANmodule->txArray[index];
        if (buffer->bufferFull && buffer->syncFlag) {
            buffer->bufferFull = false;
            CANmodule->CANtxCount--;
            deleted = true;
        }
    }
    CANmodule->bufferInhibitFlag = false;
    CO_UNLOCK_CAN_SEND(CANmodule);
    if (deleted) {
        CANmodule->CANerrorStatus |= CO_CAN_ERRTX_PDO_LATE;
    }
}

void CO_CANmodule_process(CO_CANmodule_t *CANmodule)
{
    FDCAN_ErrorCountersTypeDef counters = {0};
    FDCAN_ProtocolStatusTypeDef protocol = {0};
    uint16_t status;
    uint32_t errorSnapshot;
    FDCAN_HandleTypeDef *fdcan;

    if (CANmodule == NULL) {
        return;
    }
    drain_pending_tx(CANmodule);
    fdcan = fdcan_handle(CANmodule);
    if ((fdcan == NULL) ||
        (HAL_FDCAN_GetErrorCounters(fdcan, &counters) != HAL_OK) ||
        (HAL_FDCAN_GetProtocolStatus(fdcan, &protocol) != HAL_OK)) {
        CANmodule->txDriverErrors++;
        return;
    }

    errorSnapshot = ((counters.TxErrorCnt & UINT32_C(0xFF)) << 24) |
                    ((counters.RxErrorCnt & UINT32_C(0xFF)) << 16) |
                    ((protocol.BusOff & UINT32_C(1)) << 1) |
                    (protocol.Warning & UINT32_C(1));
    if (errorSnapshot == CANmodule->errOld) {
        return;
    }
    CANmodule->errOld = errorSnapshot;
    status = (uint16_t)(CANmodule->CANerrorStatus &
                        (CO_CAN_ERRTX_OVERFLOW | CO_CAN_ERRTX_PDO_LATE |
                         CO_CAN_ERRRX_OVERFLOW));
    if (protocol.Warning != 0U) {
        status |= CO_CAN_ERRTX_WARNING | CO_CAN_ERRRX_WARNING;
    }
    if ((protocol.ErrorPassive != 0U) || (counters.RxErrorPassive != 0U)) {
        status |= CO_CAN_ERRTX_PASSIVE | CO_CAN_ERRRX_PASSIVE;
    }
    if (protocol.BusOff != 0U) {
        status |= CO_CAN_ERRTX_BUS_OFF;
    }
    CANmodule->CANerrorStatus = status;
}

/* Strong definition replacing the transport layer's weak classic-CAN hook. */
void canopen_classic_receive_isr(uint16_t identifier,
                                 const uint8_t *data,
                                 uint8_t dataLength)
{
    CO_CANmodule_t *module = activeModule;
    CO_CANrxMsg_t message = {0};

    if ((module == NULL) || !module->CANnormal || (data == NULL) ||
        (identifier > CANOPEN_CLASSIC_IDENT_MASK) || (dataLength > 8U)) {
        if (module != NULL) {
            module->rxDropped++;
        }
        return;
    }

    message.ident = identifier;
    message.DLC = dataLength;
    memcpy(message.data, data, dataLength);
    for (uint16_t index = 0U; index < module->rxSize; ++index) {
        CO_CANrx_t *buffer = &module->rxArray[index];
        if ((((message.ident ^ buffer->ident) & buffer->mask) == 0U) &&
            (buffer->CANrx_callback != NULL)) {
            buffer->CANrx_callback(buffer->object, &message);
            return;
        }
    }
    module->rxDropped++;
}

/* Called from cooperative CANopen processing to share FIFO space fairly with
 * the vendor-specific CAN-FD path. */
void canopen_classic_driver_poll(CO_CANmodule_t *CANmodule)
{
    drain_pending_tx(CANmodule);
}
