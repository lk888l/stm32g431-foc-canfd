/* Compact, fixed-map Object Dictionary for the brushless drive.
 * Compatible with CANopenNode v4.  Communication parameters are deliberately
 * fixed at build time; safety-critical deployments should regenerate this
 * file from the released EDS/XDD and rerun CANopen conformance tests. */
#ifndef BRUSHLESS_CANOPEN_OD_H
#define BRUSHLESS_CANOPEN_OD_H

#include <stdint.h>

#include "301/CO_ODinterface.h"

#ifdef __cplusplus
extern "C" {
#endif

#define OD_CNT_NMT 1
#define OD_CNT_EM 1
#define OD_CNT_EM_PROD 1
#define OD_CNT_SYNC 0
#define OD_CNT_SYNC_PROD 0
#define OD_CNT_STORAGE 0
#define OD_CNT_TIME 0
#define OD_CNT_HB_CONS 1
#define OD_CNT_HB_PROD 1
#define OD_CNT_SDO_SRV 1
#define OD_CNT_SDO_CLI 0
#define OD_CNT_RPDO 2
#define OD_CNT_TPDO 2

#define OD_CNT_ARR_1003 8
#define OD_CNT_ARR_1016 1

typedef struct {
    uint32_t x1000_deviceType;
    uint32_t x1014_cobIdEmcy;
    uint8_t x1016_consumerHeartbeatTime_sub0;
    uint32_t x1016_consumerHeartbeatTime[OD_CNT_ARR_1016];
    uint16_t x1017_producerHeartbeatTime;
    struct {
        uint8_t highestSubIndexSupported;
        uint32_t vendorId;
        uint32_t productCode;
        uint32_t revisionNumber;
        uint32_t serialNumber;
    } x1018_identity;
    struct {
        uint8_t highestSubIndexSupported;
        uint32_t cobId;
        uint8_t transmissionType;
        uint16_t eventTimer;
    } x1400_rpdo1Comm;
    struct {
        uint8_t highestSubIndexSupported;
        uint32_t cobId;
        uint8_t transmissionType;
        uint16_t eventTimer;
    } x1401_rpdo2Comm;
    struct {
        uint8_t mappedCount;
        uint32_t map[1];
    } x1600_rpdo1Map;
    struct {
        uint8_t mappedCount;
        uint32_t map[2];
    } x1601_rpdo2Map;
    struct {
        uint8_t highestSubIndexSupported;
        uint32_t cobId;
        uint8_t transmissionType;
        uint16_t inhibitTime;
        uint16_t eventTimer;
        uint8_t syncStartValue;
    } x1800_tpdo1Comm;
    struct {
        uint8_t highestSubIndexSupported;
        uint32_t cobId;
        uint8_t transmissionType;
        uint16_t inhibitTime;
        uint16_t eventTimer;
        uint8_t syncStartValue;
    } x1801_tpdo2Comm;
    struct {
        uint8_t mappedCount;
        uint32_t map[3];
    } x1A00_tpdo1Map;
    struct {
        uint8_t mappedCount;
        uint32_t map[2];
    } x1A01_tpdo2Map;
} OD_PERSIST_COMM_t;

typedef struct {
    uint8_t x1001_errorRegister;
    struct {
        uint8_t highestSubIndexSupported;
        uint32_t cobIdClientToServerRx;
        uint32_t cobIdServerToClientTx;
    } x1200_sdoServer;

    uint8_t x2000_mitHighestSubIndex;
    int32_t x2000_mitPosition;
    int32_t x2000_mitVelocity;
    int32_t x2000_mitTorque;
    uint16_t x2000_mitKp;
    uint16_t x2000_mitKd;
    uint16_t x2000_mitCurrentLimit;
    uint16_t x2000_mitSequence;

    uint16_t x603F_errorCode;
    uint16_t x6040_controlword;
    uint16_t x6041_statusword;
    int8_t x6060_modesOfOperation;
    int8_t x6061_modesDisplay;
    int32_t x6064_positionActual;
    int32_t x606C_velocityActual;
    int16_t x6071_targetTorque;
    int16_t x6077_torqueActual;
    int32_t x607A_targetPosition;
    uint32_t x6081_profileVelocity;
    uint32_t x6083_profileAcceleration;
    uint32_t x6084_profileDeceleration;
    int32_t x60FF_targetVelocity;
    uint32_t x6502_supportedModes;
} OD_RAM_t;

extern OD_PERSIST_COMM_t OD_PERSIST_COMM;
extern OD_RAM_t OD_RAM;
extern OD_t *OD;

/* Entry positions in the sorted OD list.  CANopenNode's static configuration
 * consumes these names while compiling CANopen.c. */
#define OD_ENTRY_H1000 (&OD->list[0])
#define OD_ENTRY_H1001 (&OD->list[1])
#define OD_ENTRY_H1003 (&OD->list[2])
#define OD_ENTRY_H1014 (&OD->list[3])
#define OD_ENTRY_H1016 (&OD->list[4])
#define OD_ENTRY_H1017 (&OD->list[5])
#define OD_ENTRY_H1018 (&OD->list[6])
#define OD_ENTRY_H1200 (&OD->list[7])
#define OD_ENTRY_H1400 (&OD->list[8])
#define OD_ENTRY_H1401 (&OD->list[9])
#define OD_ENTRY_H1600 (&OD->list[10])
#define OD_ENTRY_H1601 (&OD->list[11])
#define OD_ENTRY_H1800 (&OD->list[12])
#define OD_ENTRY_H1801 (&OD->list[13])
#define OD_ENTRY_H1A00 (&OD->list[14])
#define OD_ENTRY_H1A01 (&OD->list[15])
#define OD_ENTRY_H2000 (&OD->list[16])
#define OD_ENTRY_H603F (&OD->list[17])
#define OD_ENTRY_H6040 (&OD->list[18])
#define OD_ENTRY_H6041 (&OD->list[19])
#define OD_ENTRY_H6060 (&OD->list[20])
#define OD_ENTRY_H6061 (&OD->list[21])
#define OD_ENTRY_H6064 (&OD->list[22])
#define OD_ENTRY_H606C (&OD->list[23])
#define OD_ENTRY_H6071 (&OD->list[24])
#define OD_ENTRY_H6077 (&OD->list[25])
#define OD_ENTRY_H607A (&OD->list[26])
#define OD_ENTRY_H6081 (&OD->list[27])
#define OD_ENTRY_H6083 (&OD->list[28])
#define OD_ENTRY_H6084 (&OD->list[29])
#define OD_ENTRY_H60FF (&OD->list[30])
#define OD_ENTRY_H6502 (&OD->list[31])

#ifdef __cplusplus
}
#endif

#endif /* BRUSHLESS_CANOPEN_OD_H */
