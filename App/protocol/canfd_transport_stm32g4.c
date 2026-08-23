#include "canfd_transport_stm32g4.h"

#include <stddef.h>
#include <string.h>

typedef struct {
    FDCAN_HandleTypeDef *fdcan;
    uint32_t command_id;
    uint32_t feedback_id;
    motor_vendor_command_limits_t limits;
    motor_vendor_rx_guard_t guard;
    motor_vendor_command_t pending_command;
    volatile bool pending;
    canfd_transport_stats_t stats;
    bool initialized;
} canfd_transport_context_t;

static canfd_transport_context_t transport;

void canopen_classic_receive_isr(uint16_t identifier,
                                 const uint8_t *data,
                                 uint8_t data_length) __attribute__((weak));

void canopen_classic_receive_isr(uint16_t identifier,
                                 const uint8_t *data,
                                 uint8_t data_length)
{
    (void)identifier;
    (void)data;
    (void)data_length;
}

static uint8_t dlc_to_bytes(uint32_t dlc)
{
    static const uint8_t length[16] = {
        0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U,
        8U, 12U, 16U, 20U, 24U, 32U, 48U, 64U
    };
    return length[dlc & 0x0FU];
}

bool canfd_transport_init(FDCAN_HandleTypeDef *fdcan,
                          uint32_t command_extended_id,
                          uint32_t feedback_extended_id,
                          const motor_vendor_command_limits_t *limits)
{
    if ((fdcan == NULL) || (limits == NULL) ||
        (command_extended_id > 0x1FFFFFFFU) ||
        (feedback_extended_id > 0x1FFFFFFFU)) {
        return false;
    }
    memset(&transport, 0, sizeof(transport));
    transport.fdcan = fdcan;
    transport.command_id = command_extended_id;
    transport.feedback_id = feedback_extended_id;
    transport.limits = *limits;
    motor_vendor_rx_guard_init(&transport.guard, 1024U);
    transport.stats.last_protocol_result = MOTOR_VENDOR_CANFD_OK;
    transport.initialized = true;
    return true;
}

bool canfd_transport_take_command(motor_vendor_command_t *command)
{
    uint32_t primask;

    if ((command == NULL) || !transport.initialized) {
        return false;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    if (!transport.pending) {
        if (primask == 0U) {
            __enable_irq();
        }
        return false;
    }
    *command = transport.pending_command;
    transport.pending = false;
    if (primask == 0U) {
        __enable_irq();
    }
    return true;
}

bool canfd_transport_send_feedback(const motor_vendor_feedback_t *feedback,
                                   const motor_vendor_feedback_limits_t *limits)
{
    uint8_t frame[MOTOR_VENDOR_CANFD_FEEDBACK_LENGTH];
    FDCAN_TxHeaderTypeDef header = {0};
    motor_vendor_canfd_result_t result;
    HAL_StatusTypeDef tx_status;

    if (!transport.initialized || (feedback == NULL) || (limits == NULL)) {
        return false;
    }
    result = motor_vendor_encode_feedback(feedback,
                                          limits,
                                          frame,
                                          sizeof(frame));
    if (result != MOTOR_VENDOR_CANFD_OK) {
        transport.stats.last_protocol_result = result;
        return false;
    }

    header.Identifier = transport.feedback_id;
    header.IdType = FDCAN_EXTENDED_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_48;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_ON;
    header.FDFormat = FDCAN_FD_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0U;
    if (!bsp_fdcan_tx_try_lock()) {
        ++transport.stats.tx_busy;
        return false;
    }
    tx_status = HAL_FDCAN_AddMessageToTxFifoQ(transport.fdcan,
                                              &header,
                                              frame);
    bsp_fdcan_tx_unlock();
    if (tx_status != HAL_OK) {
        ++transport.stats.tx_busy;
        return false;
    }
    ++transport.stats.tx_feedback;
    return true;
}

canfd_transport_stats_t canfd_transport_stats(void)
{
    canfd_transport_stats_t snapshot;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    snapshot = transport.stats;
    if (primask == 0U) {
        __enable_irq();
    }
    return snapshot;
}

bool canfd_transport_clear_bus_off_latch(void)
{
    FDCAN_ProtocolStatusTypeDef protocol = {0};
    const uint32_t primask = __get_PRIMASK();

    __disable_irq();
    if (!transport.initialized ||
        (HAL_FDCAN_GetProtocolStatus(transport.fdcan, &protocol) != HAL_OK) ||
        (protocol.BusOff != 0U)) {
        if (primask == 0U) {
            __enable_irq();
        }
        return false;
    }
    transport.stats.bus_off = false;
    if (primask == 0U) {
        __enable_irq();
    }
    return true;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *fdcan,
                               uint32_t rx_fifo0_its)
{
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[64];

    if (!transport.initialized || (fdcan != transport.fdcan) ||
        ((rx_fifo0_its & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)) {
        return;
    }
    while (HAL_FDCAN_GetRxFifoFillLevel(fdcan, FDCAN_RX_FIFO0) > 0U) {
        if (HAL_FDCAN_GetRxMessage(fdcan,
                                   FDCAN_RX_FIFO0,
                                   &header,
                                   data) != HAL_OK) {
            ++transport.stats.rx_rejected;
            return;
        }
        if ((header.IdType == FDCAN_EXTENDED_ID) &&
            (header.Identifier == transport.command_id)) {
            motor_vendor_command_t decoded;
            motor_vendor_canfd_result_t result;

            if ((header.FDFormat != FDCAN_FD_CAN) ||
                (header.BitRateSwitch != FDCAN_BRS_ON) ||
                (header.DataLength != FDCAN_DLC_BYTES_32)) {
                transport.stats.last_protocol_result =
                    MOTOR_VENDOR_CANFD_LENGTH_ERROR;
                ++transport.stats.rx_rejected;
                continue;
            }
            result = motor_vendor_accept_command(&transport.guard,
                                                 data,
                                                 MOTOR_VENDOR_CANFD_COMMAND_LENGTH,
                                                 bsp_micros(),
                                                 &transport.limits,
                                                 &decoded);
            transport.stats.last_protocol_result = result;
            if ((result == MOTOR_VENDOR_CANFD_OK) ||
                (result == MOTOR_VENDOR_CANFD_SEQUENCE_GAP)) {
                transport.pending_command = decoded;
                transport.pending = true;
                ++transport.stats.rx_valid;
            } else {
                ++transport.stats.rx_rejected;
            }
        } else if ((header.IdType == FDCAN_STANDARD_ID) &&
                   (header.FDFormat == FDCAN_CLASSIC_CAN)) {
            const uint8_t length = dlc_to_bytes(header.DataLength);
            canopen_classic_receive_isr((uint16_t)header.Identifier,
                                        data,
                                        length > 8U ? 8U : length);
            ++transport.stats.rx_classic;
        } else {
            ++transport.stats.rx_rejected;
        }
    }
}

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *fdcan)
{
    if (transport.initialized && (fdcan == transport.fdcan)) {
        ++transport.stats.bus_errors;
    }
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *fdcan,
                                   uint32_t error_status_its)
{
    if (!transport.initialized || (fdcan != transport.fdcan)) {
        return;
    }
    ++transport.stats.bus_errors;
    if ((error_status_its & FDCAN_IT_BUS_OFF) != 0U) {
        transport.stats.bus_off = true;
        /* Bosch M_CAN enters INIT on bus-off. Clearing INIT starts the
         * ISO 11898-1 recovery sequence; the drive fault remains latched until
         * a later, explicit operator reset. */
        CLEAR_BIT(fdcan->Instance->CCCR, FDCAN_CCCR_INIT);
    }
}
