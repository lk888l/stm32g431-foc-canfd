/*
 * CANopenNode v4 target definitions for STM32G431 FDCAN1.
 *
 * IMPORTANT: this port deliberately sends ISO 11898-1 classic CAN frames
 * (11-bit identifier, 0..8 data bytes) through the STM32 FDCAN peripheral.
 * It is a CiA 301 CANopen port.  It is not a CiA 1301 CANopen FD stack.
 */
#ifndef BRUSHLESS_CO_DRIVER_TARGET_H
#define BRUSHLESS_CO_DRIVER_TARGET_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Keep the embedded build deterministic: CANopenNode uses static objects. */
#define CO_USE_GLOBALS

/* Small, drive-oriented CiA 301 feature set. */
#define CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE (0)
#define CO_CONFIG_GLOBAL_RT_FLAG_CALLBACK_PRE (0)
#define CO_CONFIG_GLOBAL_FLAG_TIMERNEXT (0)
#define CO_CONFIG_GLOBAL_FLAG_OD_DYNAMIC (0)
#define CO_CONFIG_NMT (CO_CONFIG_NMT_CALLBACK_CHANGE)
#define CO_CONFIG_HB_CONS                                                                    \
    (CO_CONFIG_HB_CONS_ENABLE | CO_CONFIG_HB_CONS_CALLBACK_MULTI |                         \
     CO_CONFIG_HB_CONS_QUERY_FUNCT | CO_CONFIG_FLAG_OD_DYNAMIC)
#define CO_CONFIG_NODE_GUARDING (0)
#define CO_CONFIG_EM (CO_CONFIG_EM_PRODUCER | CO_CONFIG_EM_HISTORY)
#define CO_CONFIG_EM_ERR_STATUS_BITS_COUNT (80)
#define CO_CONFIG_SDO_SRV (CO_CONFIG_SDO_SRV_SEGMENTED)
#define CO_CONFIG_SDO_SRV_BUFFER_SIZE (32)
#define CO_CONFIG_SDO_CLI (0)
#define CO_CONFIG_TIME (0)
#define CO_CONFIG_SYNC (0)
#define CO_CONFIG_PDO                                                                            \
    (CO_CONFIG_RPDO_ENABLE | CO_CONFIG_TPDO_ENABLE | CO_CONFIG_RPDO_TIMERS_ENABLE |             \
     CO_CONFIG_TPDO_TIMERS_ENABLE | CO_CONFIG_PDO_OD_IO_ACCESS)
#define CO_CONFIG_STORAGE (0)
#define CO_CONFIG_LEDS (0)
#define CO_CONFIG_GFC (0)
#define CO_CONFIG_SRDO (0)
#define CO_CONFIG_LSS (0)
#define CO_CONFIG_GTW (0)
#define CO_CONFIG_CRC16 (0)
#define CO_CONFIG_FIFO (CO_CONFIG_FIFO_ENABLE)
#define CO_CONFIG_TRACE (0)
#define CO_CONFIG_DEBUG (0)

#define CO_LITTLE_ENDIAN
#define CO_SWAP_16(x) (x)
#define CO_SWAP_32(x) (x)
#define CO_SWAP_64(x) (x)

typedef uint_fast8_t bool_t;
typedef float float32_t;
typedef double float64_t;

typedef struct {
    uint16_t ident; /* bits 0..10 identifier, bit 11 RTR */
    uint8_t DLC;
    uint8_t data[8];
} CO_CANrxMsg_t;

#define CO_CANrxMsg_readIdent(msg) (((const CO_CANrxMsg_t *)(msg))->ident & UINT16_C(0x07FF))
#define CO_CANrxMsg_readDLC(msg) (((const CO_CANrxMsg_t *)(msg))->DLC)
#define CO_CANrxMsg_readData(msg) (((const CO_CANrxMsg_t *)(msg))->data)

typedef struct {
    uint16_t ident;
    uint16_t mask;
    void *object;
    void (*CANrx_callback)(void *object, void *message);
} CO_CANrx_t;

typedef struct {
    uint16_t ident;
    uint8_t DLC;
    uint8_t data[8];
    bool_t rtr;
    volatile bool_t bufferFull;
    volatile bool_t syncFlag;
} CO_CANtx_t;

typedef struct {
    void *CANptr;
    CO_CANrx_t *rxArray;
    uint16_t rxSize;
    CO_CANtx_t *txArray;
    uint16_t txSize;
    uint16_t CANerrorStatus;
    volatile bool_t CANnormal;
    volatile bool_t useCANrxFilters;
    volatile bool_t bufferInhibitFlag;
    volatile bool_t firstCANtxMessage;
    volatile uint16_t CANtxCount;
    uint32_t errOld;

    /* Target additions.  Separate saved interrupt states avoid cross-lock use. */
    uint32_t lockCanSend;
    uint32_t lockEmergency;
    uint32_t lockOd;
    volatile uint32_t rxDropped;
    volatile uint32_t txDriverErrors;
} CO_CANmodule_t;

typedef struct {
    void *addr;
    size_t len;
    uint8_t subIndexOD;
    uint8_t attr;
    void *storageModule;
    uint16_t crc;
    size_t eepromAddrSignature;
    size_t eepromAddr;
    size_t offset;
    void *additionalParameters;
} CO_storage_entry_t;

uint32_t canopen_classic_irq_lock(void);
void canopen_classic_irq_unlock(uint32_t previousPrimask);

#define CO_LOCK_CAN_SEND(CAN_MODULE)                                                            \
    do {                                                                                         \
        (CAN_MODULE)->lockCanSend = canopen_classic_irq_lock();                                 \
    } while (0)
#define CO_UNLOCK_CAN_SEND(CAN_MODULE)                                                          \
    do {                                                                                         \
        canopen_classic_irq_unlock((CAN_MODULE)->lockCanSend);                                  \
    } while (0)
#define CO_LOCK_EMCY(CAN_MODULE)                                                                \
    do {                                                                                         \
        (CAN_MODULE)->lockEmergency = canopen_classic_irq_lock();                               \
    } while (0)
#define CO_UNLOCK_EMCY(CAN_MODULE)                                                              \
    do {                                                                                         \
        canopen_classic_irq_unlock((CAN_MODULE)->lockEmergency);                                \
    } while (0)
#define CO_LOCK_OD(CAN_MODULE)                                                                  \
    do {                                                                                         \
        (CAN_MODULE)->lockOd = canopen_classic_irq_lock();                                      \
    } while (0)
#define CO_UNLOCK_OD(CAN_MODULE)                                                                \
    do {                                                                                         \
        canopen_classic_irq_unlock((CAN_MODULE)->lockOd);                                       \
    } while (0)

#define CO_MemoryBarrier() __atomic_thread_fence(__ATOMIC_SEQ_CST)
#define CO_FLAG_READ(rxNew) ((rxNew) != NULL)
#define CO_FLAG_SET(rxNew)                                                                      \
    do {                                                                                         \
        CO_MemoryBarrier();                                                                      \
        (rxNew) = (void *)1L;                                                                    \
    } while (0)
#define CO_FLAG_CLEAR(rxNew)                                                                    \
    do {                                                                                         \
        CO_MemoryBarrier();                                                                      \
        (rxNew) = NULL;                                                                          \
    } while (0)

#ifdef __cplusplus
}
#endif

#endif /* BRUSHLESS_CO_DRIVER_TARGET_H */
