#ifndef ROLLER_CO_DRIVER_TARGET_H
#define ROLLER_CO_DRIVER_TARGET_H
#include "co_transport.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define CO_LITTLE_ENDIAN
#define CO_SWAP_16(x) (x)
#define CO_SWAP_32(x) (x)
#define CO_SWAP_64(x) (x)
typedef uint_fast8_t bool_t;
typedef float float32_t;
typedef double float64_t;
#define CO_USE_GLOBALS
#define CO_CONFIG_NMT 0
#define CO_CONFIG_HB_CONS (CO_CONFIG_HB_CONS_ENABLE | CO_CONFIG_FLAG_OD_DYNAMIC)
#define CO_CONFIG_NODE_GUARDING 0
#define CO_CONFIG_EM (CO_CONFIG_EM_PRODUCER | CO_CONFIG_EM_HISTORY | CO_CONFIG_FLAG_OD_DYNAMIC)
#define CO_CONFIG_SDO_SRV (CO_CONFIG_SDO_SRV_SEGMENTED)
#define CO_CONFIG_SDO_SRV_BUFFER_SIZE 128
#define CO_CONFIG_SDO_CLI 0
#define CO_CONFIG_SYNC (CO_CONFIG_SYNC_ENABLE | CO_CONFIG_FLAG_OD_DYNAMIC)
#define CO_CONFIG_PDO                                                                                        \
    (CO_CONFIG_RPDO_ENABLE | CO_CONFIG_TPDO_ENABLE | CO_CONFIG_RPDO_TIMERS_ENABLE |                          \
     CO_CONFIG_TPDO_TIMERS_ENABLE | CO_CONFIG_PDO_SYNC_ENABLE | CO_CONFIG_PDO_OD_IO_ACCESS |                 \
     CO_CONFIG_FLAG_OD_DYNAMIC)
// Manufacturer extension: classic CANopen services with up to 64-byte FD PDOs.
// CANopenNode already sizes mapping and payload buffers through these macros.
#define CO_PDO_MAX_SIZE 64U
#define CO_PDO_MAX_MAPPED_ENTRIES 16U
#define CO_CONFIG_TIME 0
#define CO_CONFIG_LEDS 0
#define CO_CONFIG_LSS 0
#define CO_CONFIG_GFC 0
#define CO_CONFIG_SRDO 0
#define CO_CONFIG_TRACE 0
#define CO_CONFIG_STORAGE 0
#define CO_CONFIG_CRC16 0
#define CO_CONFIG_FIFO 0
#define CO_CONFIG_GTWA 0

#define CO_CANrxMsg_readIdent(msg) (((const RollerCanFrame*)(msg))->ident)
#define CO_CANrxMsg_readDLC(msg) (((const RollerCanFrame*)(msg))->size)
#define CO_CANrxMsg_readData(msg) (((const RollerCanFrame*)(msg))->data)
typedef struct {
    uint16_t ident, mask;
    void* object;
    void (*CANrx_callback)(void*, void*);
} CO_CANrx_t;
typedef struct {
    uint32_t ident;
    uint8_t DLC, data[64];
    bool_t bufferFull, syncFlag;
} CO_CANtx_t;
typedef struct {
    void* CANptr;
    CO_CANrx_t* rxArray;
    CO_CANtx_t* txArray;
    uint16_t rxSize, txSize, CANerrorStatus, CANtxCount;
    bool_t CANnormal, useCANrxFilters, bufferInhibitFlag, firstCANtxMessage;
    uint32_t errOld;
} CO_CANmodule_t;
typedef struct {
    void* addr;
    size_t len;
    uint8_t subIndexOD, attr;
} CO_storage_entry_t;
// CANopen is an actor: RX dispatch, OD, process and TX execute in ONE task.
// Hardware ISRs only enqueue complete frames, never enter CANopenNode.
#define CO_LOCK_CAN_SEND(module) ((void)(module))
#define CO_UNLOCK_CAN_SEND(module) ((void)(module))
#define CO_LOCK_EMCY(module) ((void)(module))
#define CO_UNLOCK_EMCY(module) ((void)(module))
#define CO_LOCK_OD(module) ((void)(module))
#define CO_UNLOCK_OD(module) ((void)(module))
#define CO_MemoryBarrier() __asm__ volatile("" ::: "memory")
#define CO_FLAG_READ(flag) ((flag) != NULL)
#define CO_FLAG_SET(flag)                                                                                    \
    do {                                                                                                     \
        CO_MemoryBarrier();                                                                                  \
        (flag) = (void*)1;                                                                                   \
    } while (0)
#define CO_FLAG_CLEAR(flag)                                                                                  \
    do {                                                                                                     \
        CO_MemoryBarrier();                                                                                  \
        (flag) = NULL;                                                                                       \
    } while (0)
#endif
