/* Cooperative CANopenNode integration for the brushless drive. */
#ifndef BRUSHLESS_CANOPEN_CLASSIC_H
#define BRUSHLESS_CANOPEN_CLASSIC_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CANOPEN_CLASSIC_REQUIRED_BITRATE_KBIT UINT16_C(500)

typedef enum {
    CANOPEN_CLASSIC_STATE_UNINITIALIZED = 0,
    CANOPEN_CLASSIC_STATE_PRE_OPERATIONAL,
    CANOPEN_CLASSIC_STATE_OPERATIONAL,
    CANOPEN_CLASSIC_STATE_STOPPED,
    CANOPEN_CLASSIC_STATE_ERROR
} canopen_classic_state_t;

typedef struct {
    canopen_classic_state_t state;
    uint32_t initialization_error_info;
    int32_t stack_error;
    uint32_t communication_resets;
    uint32_t reported_drive_faults;
    uint32_t rpdo_transaction_rejections;
    bool heartbeat_configured;
    bool heartbeat_active;
    bool application_reset_requested;
} canopen_classic_status_t;

/* The BSP must initialize fdcan before this call and start it before the first
 * process call.  Only 500 kbit/s classic frames are supported because timing
 * is owned by the shared BSP. */
bool canopen_classic_init(FDCAN_HandleTypeDef *fdcan,
                          uint8_t node_id,
                          uint16_t bitrate_kbit);

/* Call from a deferred 1 kHz context (a task or low-priority PendSV bottom
 * half), never inline from the 20 kHz current-control ISR. */
void canopen_classic_process_1ms(void);

/* Call from the main loop with the elapsed time since the previous call. */
void canopen_classic_background(uint32_t elapsed_us);

/* Bridge the application's effective (latched OR live) fault mask into one
 * manufacturer-specific EMCY condition.  Call after publishing 0x603F.  This
 * function is short, interrupt-safe through CANopenNode's EMCY lock, and does
 * not access the FDCAN peripheral directly. */
void canopen_classic_report_drive_faults(uint32_t effective_faults,
                                         uint16_t cia402_error_code);

canopen_classic_status_t canopen_classic_status(void);
bool canopen_classic_is_operational(void);

#ifdef __cplusplus
}
#endif

#endif /* BRUSHLESS_CANOPEN_CLASSIC_H */
