#ifndef BRUSHLESS_CANFD_TRANSPORT_STM32G4_H
#define BRUSHLESS_CANFD_TRANSPORT_STM32G4_H

#include <stdbool.h>
#include <stdint.h>

#include "bsp.h"
#include "motor_vendor_canfd.h"

typedef struct {
    uint32_t rx_valid;
    uint32_t rx_rejected;
    uint32_t rx_classic;
    uint32_t tx_feedback;
    uint32_t tx_busy;
    uint32_t bus_errors;
    motor_vendor_canfd_result_t last_protocol_result;
    bool bus_off;
} canfd_transport_stats_t;

bool canfd_transport_init(FDCAN_HandleTypeDef *fdcan,
                          uint32_t command_extended_id,
                          uint32_t feedback_extended_id,
                          const motor_vendor_command_limits_t *limits);
bool canfd_transport_take_command(motor_vendor_command_t *command);
bool canfd_transport_send_feedback(const motor_vendor_feedback_t *feedback,
                                   const motor_vendor_feedback_limits_t *limits);
canfd_transport_stats_t canfd_transport_stats(void);
/* Clear the software bus-off latch only after the M_CAN protocol engine has
 * completed recovery.  A fresh CiA 402 fault-reset edge is still required. */
bool canfd_transport_clear_bus_off_latch(void);

/*
 * The transport supplies a weak fallback; the CANopenNode port provides the
 * strong implementation. Frames here are classic 11-bit frames only.
 */
void canopen_classic_receive_isr(uint16_t identifier,
                                 const uint8_t *data,
                                 uint8_t data_length);

#endif
