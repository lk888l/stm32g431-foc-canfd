#define OD_DEFINITION
#include "301/CO_ODinterface.h"
#include "OD.h"

#if CO_VERSION_MAJOR < 4
#error "This Object Dictionary requires CANopenNode v4"
#endif

#define OD_MAP(index_, sub_, bits_)                                                            \
    ((((uint32_t)(index_)) << 16) | (((uint32_t)(sub_)) << 8) | (uint32_t)(bits_))

OD_PERSIST_COMM_t OD_PERSIST_COMM = {
    /* 0x0192 is the CiA 402 device-profile number. */
    .x1000_deviceType = UINT32_C(0x00000192),
    .x1014_cobIdEmcy = UINT32_C(0x00000080),
    .x1016_consumerHeartbeatTime_sub0 = 1U,
    .x1016_consumerHeartbeatTime = {UINT32_C(0)},
    .x1017_producerHeartbeatTime = UINT16_C(100),
    .x1018_identity = {
        .highestSubIndexSupported = 4U,
        .vendorId = UINT32_C(0),
        .productCode = UINT32_C(0x47343331), /* "G431", manufacturer specific */
        .revisionNumber = UINT32_C(0x00010000),
        .serialNumber = UINT32_C(0),
    },
    .x1400_rpdo1Comm = {
        .highestSubIndexSupported = 5U,
        .cobId = UINT32_C(0x00000200),
        .transmissionType = UINT8_C(254),
        .eventTimer = UINT16_C(0),
    },
    .x1401_rpdo2Comm = {
        .highestSubIndexSupported = 5U,
        .cobId = UINT32_C(0x00000300),
        .transmissionType = UINT8_C(254),
        .eventTimer = UINT16_C(0),
    },
    .x1600_rpdo1Map = {
        /* Keep the enabling controlword in its own PDO. CANopenNode applies
         * multi-object RPDO entries sequentially and cannot roll back an
         * earlier write if a later mapped object fails validation. */
        .mappedCount = 1U,
        .map = {
            OD_MAP(0x6040, 0, 16),
        },
    },
    .x1601_rpdo2Map = {
        .mappedCount = 2U,
        .map = {
            OD_MAP(0x607A, 0, 32),
            OD_MAP(0x60FF, 0, 32),
        },
    },
    .x1800_tpdo1Comm = {
        .highestSubIndexSupported = 6U,
        .cobId = UINT32_C(0x00000180),
        .transmissionType = UINT8_C(254),
        .inhibitTime = UINT16_C(0),
        .eventTimer = UINT16_C(10),
        .syncStartValue = UINT8_C(0),
    },
    .x1801_tpdo2Comm = {
        .highestSubIndexSupported = 6U,
        .cobId = UINT32_C(0x00000280),
        .transmissionType = UINT8_C(254),
        .inhibitTime = UINT16_C(0),
        .eventTimer = UINT16_C(20),
        .syncStartValue = UINT8_C(0),
    },
    .x1A00_tpdo1Map = {
        .mappedCount = 3U,
        .map = {
            OD_MAP(0x6041, 0, 16),
            OD_MAP(0x6061, 0, 8),
            OD_MAP(0x603F, 0, 16),
        },
    },
    .x1A01_tpdo2Map = {
        .mappedCount = 2U,
        .map = {
            OD_MAP(0x6064, 0, 32),
            OD_MAP(0x606C, 0, 32),
        },
    },
};

OD_RAM_t OD_RAM = {
    .x1001_errorRegister = UINT8_C(0),
    .x1200_sdoServer = {
        .highestSubIndexSupported = 2U,
        .cobIdClientToServerRx = UINT32_C(0x00000600),
        .cobIdServerToClientTx = UINT32_C(0x00000580),
    },
    .x2000_mitHighestSubIndex = 7U,
    .x6502_supportedModes = UINT32_C(0x80000005),
};

typedef struct {
    OD_obj_var_t o1000;
    OD_obj_var_t o1001;
    OD_obj_array_t o1003;
    OD_obj_var_t o1014;
    OD_obj_array_t o1016;
    OD_obj_var_t o1017;
    OD_obj_record_t o1018[5];
    OD_obj_record_t o1200[3];
    OD_obj_record_t o1400[4];
    OD_obj_record_t o1401[4];
    OD_obj_record_t o1600[2];
    OD_obj_record_t o1601[3];
    OD_obj_record_t o1800[6];
    OD_obj_record_t o1801[6];
    OD_obj_record_t o1A00[4];
    OD_obj_record_t o1A01[3];
    OD_obj_record_t o2000[8];
    OD_obj_var_t o603F;
    OD_obj_var_t o6040;
    OD_obj_var_t o6041;
    OD_obj_var_t o6060;
    OD_obj_var_t o6061;
    OD_obj_var_t o6064;
    OD_obj_var_t o606C;
    OD_obj_var_t o6071;
    OD_obj_var_t o6077;
    OD_obj_var_t o607A;
    OD_obj_var_t o6081;
    OD_obj_var_t o6083;
    OD_obj_var_t o6084;
    OD_obj_var_t o60FF;
    OD_obj_var_t o6502;
} brushless_od_objects_t;

#define OD_VAR(ptr_, attr_, len_) {.dataOrig = (ptr_), .attribute = (attr_), .dataLength = (len_)}
#define OD_REC(ptr_, sub_, attr_, len_)                                                         \
    {.dataOrig = (ptr_), .subIndex = (sub_), .attribute = (attr_), .dataLength = (len_)}

static CO_PROGMEM brushless_od_objects_t odObjects = {
    .o1000 = OD_VAR(&OD_PERSIST_COMM.x1000_deviceType, ODA_SDO_R | ODA_MB, 4),
    .o1001 = OD_VAR(&OD_RAM.x1001_errorRegister, ODA_SDO_R | ODA_TPDO, 1),
    .o1003 = {
        .dataOrig0 = NULL,
        .dataOrig = NULL,
        .attribute0 = ODA_SDO_RW,
        .attribute = ODA_SDO_R | ODA_MB,
        .dataElementLength = 4,
        .dataElementSizeof = sizeof(uint32_t),
    },
    .o1014 = OD_VAR(&OD_PERSIST_COMM.x1014_cobIdEmcy, ODA_SDO_R | ODA_MB, 4),
    .o1016 = {
        .dataOrig0 = &OD_PERSIST_COMM.x1016_consumerHeartbeatTime_sub0,
        .dataOrig = &OD_PERSIST_COMM.x1016_consumerHeartbeatTime[0],
        .attribute0 = ODA_SDO_R,
        .attribute = ODA_SDO_RW | ODA_MB,
        .dataElementLength = 4,
        .dataElementSizeof = sizeof(uint32_t),
    },
    .o1017 = OD_VAR(&OD_PERSIST_COMM.x1017_producerHeartbeatTime, ODA_SDO_RW | ODA_MB, 2),
    .o1018 = {
        OD_REC(&OD_PERSIST_COMM.x1018_identity.highestSubIndexSupported, 0, ODA_SDO_R, 1),
        OD_REC(&OD_PERSIST_COMM.x1018_identity.vendorId, 1, ODA_SDO_R | ODA_MB, 4),
        OD_REC(&OD_PERSIST_COMM.x1018_identity.productCode, 2, ODA_SDO_R | ODA_MB, 4),
        OD_REC(&OD_PERSIST_COMM.x1018_identity.revisionNumber, 3, ODA_SDO_R | ODA_MB, 4),
        OD_REC(&OD_PERSIST_COMM.x1018_identity.serialNumber, 4, ODA_SDO_R | ODA_MB, 4),
    },
    .o1200 = {
        OD_REC(&OD_RAM.x1200_sdoServer.highestSubIndexSupported, 0, ODA_SDO_R, 1),
        OD_REC(&OD_RAM.x1200_sdoServer.cobIdClientToServerRx, 1, ODA_SDO_R | ODA_MB, 4),
        OD_REC(&OD_RAM.x1200_sdoServer.cobIdServerToClientTx, 2, ODA_SDO_R | ODA_MB, 4),
    },
    .o1400 = {
        OD_REC(&OD_PERSIST_COMM.x1400_rpdo1Comm.highestSubIndexSupported, 0, ODA_SDO_R, 1),
        OD_REC(&OD_PERSIST_COMM.x1400_rpdo1Comm.cobId, 1, ODA_SDO_R | ODA_MB, 4),
        OD_REC(&OD_PERSIST_COMM.x1400_rpdo1Comm.transmissionType, 2, ODA_SDO_R, 1),
        OD_REC(&OD_PERSIST_COMM.x1400_rpdo1Comm.eventTimer, 5, ODA_SDO_R | ODA_MB, 2),
    },
    .o1401 = {
        OD_REC(&OD_PERSIST_COMM.x1401_rpdo2Comm.highestSubIndexSupported, 0, ODA_SDO_R, 1),
        OD_REC(&OD_PERSIST_COMM.x1401_rpdo2Comm.cobId, 1, ODA_SDO_R | ODA_MB, 4),
        OD_REC(&OD_PERSIST_COMM.x1401_rpdo2Comm.transmissionType, 2, ODA_SDO_R, 1),
        OD_REC(&OD_PERSIST_COMM.x1401_rpdo2Comm.eventTimer, 5, ODA_SDO_R | ODA_MB, 2),
    },
    .o1600 = {
        OD_REC(&OD_PERSIST_COMM.x1600_rpdo1Map.mappedCount, 0, ODA_SDO_R, 1),
        OD_REC(&OD_PERSIST_COMM.x1600_rpdo1Map.map[0], 1, ODA_SDO_R | ODA_MB, 4),
    },
    .o1601 = {
        OD_REC(&OD_PERSIST_COMM.x1601_rpdo2Map.mappedCount, 0, ODA_SDO_R, 1),
        OD_REC(&OD_PERSIST_COMM.x1601_rpdo2Map.map[0], 1, ODA_SDO_R | ODA_MB, 4),
        OD_REC(&OD_PERSIST_COMM.x1601_rpdo2Map.map[1], 2, ODA_SDO_R | ODA_MB, 4),
    },
    .o1800 = {
        OD_REC(&OD_PERSIST_COMM.x1800_tpdo1Comm.highestSubIndexSupported, 0, ODA_SDO_R, 1),
        OD_REC(&OD_PERSIST_COMM.x1800_tpdo1Comm.cobId, 1, ODA_SDO_R | ODA_MB, 4),
        OD_REC(&OD_PERSIST_COMM.x1800_tpdo1Comm.transmissionType, 2, ODA_SDO_R, 1),
        OD_REC(&OD_PERSIST_COMM.x1800_tpdo1Comm.inhibitTime, 3, ODA_SDO_R | ODA_MB, 2),
        OD_REC(&OD_PERSIST_COMM.x1800_tpdo1Comm.eventTimer, 5, ODA_SDO_R | ODA_MB, 2),
        OD_REC(&OD_PERSIST_COMM.x1800_tpdo1Comm.syncStartValue, 6, ODA_SDO_R, 1),
    },
    .o1801 = {
        OD_REC(&OD_PERSIST_COMM.x1801_tpdo2Comm.highestSubIndexSupported, 0, ODA_SDO_R, 1),
        OD_REC(&OD_PERSIST_COMM.x1801_tpdo2Comm.cobId, 1, ODA_SDO_R | ODA_MB, 4),
        OD_REC(&OD_PERSIST_COMM.x1801_tpdo2Comm.transmissionType, 2, ODA_SDO_R, 1),
        OD_REC(&OD_PERSIST_COMM.x1801_tpdo2Comm.inhibitTime, 3, ODA_SDO_R | ODA_MB, 2),
        OD_REC(&OD_PERSIST_COMM.x1801_tpdo2Comm.eventTimer, 5, ODA_SDO_R | ODA_MB, 2),
        OD_REC(&OD_PERSIST_COMM.x1801_tpdo2Comm.syncStartValue, 6, ODA_SDO_R, 1),
    },
    .o1A00 = {
        OD_REC(&OD_PERSIST_COMM.x1A00_tpdo1Map.mappedCount, 0, ODA_SDO_R, 1),
        OD_REC(&OD_PERSIST_COMM.x1A00_tpdo1Map.map[0], 1, ODA_SDO_R | ODA_MB, 4),
        OD_REC(&OD_PERSIST_COMM.x1A00_tpdo1Map.map[1], 2, ODA_SDO_R | ODA_MB, 4),
        OD_REC(&OD_PERSIST_COMM.x1A00_tpdo1Map.map[2], 3, ODA_SDO_R | ODA_MB, 4),
    },
    .o1A01 = {
        OD_REC(&OD_PERSIST_COMM.x1A01_tpdo2Map.mappedCount, 0, ODA_SDO_R, 1),
        OD_REC(&OD_PERSIST_COMM.x1A01_tpdo2Map.map[0], 1, ODA_SDO_R | ODA_MB, 4),
        OD_REC(&OD_PERSIST_COMM.x1A01_tpdo2Map.map[1], 2, ODA_SDO_R | ODA_MB, 4),
    },
    .o2000 = {
        OD_REC(&OD_RAM.x2000_mitHighestSubIndex, 0, ODA_SDO_R, 1),
        OD_REC(&OD_RAM.x2000_mitPosition, 1, ODA_SDO_RW | ODA_MB, 4),
        OD_REC(&OD_RAM.x2000_mitVelocity, 2, ODA_SDO_RW | ODA_MB, 4),
        OD_REC(&OD_RAM.x2000_mitTorque, 3, ODA_SDO_RW | ODA_MB, 4),
        OD_REC(&OD_RAM.x2000_mitKp, 4, ODA_SDO_RW | ODA_MB, 2),
        OD_REC(&OD_RAM.x2000_mitKd, 5, ODA_SDO_RW | ODA_MB, 2),
        OD_REC(&OD_RAM.x2000_mitCurrentLimit, 6, ODA_SDO_RW | ODA_MB, 2),
        OD_REC(&OD_RAM.x2000_mitSequence, 7, ODA_SDO_RW | ODA_MB, 2),
    },
    .o603F = OD_VAR(&OD_RAM.x603F_errorCode, ODA_SDO_R | ODA_TPDO | ODA_MB, 2),
    .o6040 = OD_VAR(&OD_RAM.x6040_controlword, ODA_SDO_RW | ODA_RPDO | ODA_MB, 2),
    .o6041 = OD_VAR(&OD_RAM.x6041_statusword, ODA_SDO_R | ODA_TPDO | ODA_MB, 2),
    .o6060 = OD_VAR(&OD_RAM.x6060_modesOfOperation, ODA_SDO_RW, 1),
    .o6061 = OD_VAR(&OD_RAM.x6061_modesDisplay, ODA_SDO_R | ODA_TPDO, 1),
    .o6064 = OD_VAR(&OD_RAM.x6064_positionActual, ODA_SDO_R | ODA_TPDO | ODA_MB, 4),
    .o606C = OD_VAR(&OD_RAM.x606C_velocityActual, ODA_SDO_R | ODA_TPDO | ODA_MB, 4),
    .o6071 = OD_VAR(&OD_RAM.x6071_targetTorque, ODA_SDO_RW | ODA_MB, 2),
    .o6077 = OD_VAR(&OD_RAM.x6077_torqueActual, ODA_SDO_R | ODA_TPDO | ODA_MB, 2),
    .o607A = OD_VAR(&OD_RAM.x607A_targetPosition, ODA_SDO_RW | ODA_RPDO | ODA_MB, 4),
    .o6081 = OD_VAR(&OD_RAM.x6081_profileVelocity, ODA_SDO_RW | ODA_MB, 4),
    .o6083 = OD_VAR(&OD_RAM.x6083_profileAcceleration, ODA_SDO_RW | ODA_MB, 4),
    .o6084 = OD_VAR(&OD_RAM.x6084_profileDeceleration, ODA_SDO_RW | ODA_MB, 4),
    .o60FF = OD_VAR(&OD_RAM.x60FF_targetVelocity, ODA_SDO_RW | ODA_RPDO | ODA_MB, 4),
    .o6502 = OD_VAR(&OD_RAM.x6502_supportedModes, ODA_SDO_R | ODA_MB, 4),
};

static OD_entry_t odList[] = {
    {0x1000, 0x01, ODT_VAR, &odObjects.o1000, NULL},
    {0x1001, 0x01, ODT_VAR, &odObjects.o1001, NULL},
    {0x1003, 0x09, ODT_ARR, &odObjects.o1003, NULL},
    {0x1014, 0x01, ODT_VAR, &odObjects.o1014, NULL},
    {0x1016, 0x02, ODT_ARR, &odObjects.o1016, NULL},
    {0x1017, 0x01, ODT_VAR, &odObjects.o1017, NULL},
    {0x1018, 0x05, ODT_REC, &odObjects.o1018, NULL},
    {0x1200, 0x03, ODT_REC, &odObjects.o1200, NULL},
    {0x1400, 0x04, ODT_REC, &odObjects.o1400, NULL},
    {0x1401, 0x04, ODT_REC, &odObjects.o1401, NULL},
    {0x1600, 0x02, ODT_REC, &odObjects.o1600, NULL},
    {0x1601, 0x03, ODT_REC, &odObjects.o1601, NULL},
    {0x1800, 0x06, ODT_REC, &odObjects.o1800, NULL},
    {0x1801, 0x06, ODT_REC, &odObjects.o1801, NULL},
    {0x1A00, 0x04, ODT_REC, &odObjects.o1A00, NULL},
    {0x1A01, 0x03, ODT_REC, &odObjects.o1A01, NULL},
    {0x2000, 0x08, ODT_REC, &odObjects.o2000, NULL},
    {0x603F, 0x01, ODT_VAR, &odObjects.o603F, NULL},
    {0x6040, 0x01, ODT_VAR, &odObjects.o6040, NULL},
    {0x6041, 0x01, ODT_VAR, &odObjects.o6041, NULL},
    {0x6060, 0x01, ODT_VAR, &odObjects.o6060, NULL},
    {0x6061, 0x01, ODT_VAR, &odObjects.o6061, NULL},
    {0x6064, 0x01, ODT_VAR, &odObjects.o6064, NULL},
    {0x606C, 0x01, ODT_VAR, &odObjects.o606C, NULL},
    {0x6071, 0x01, ODT_VAR, &odObjects.o6071, NULL},
    {0x6077, 0x01, ODT_VAR, &odObjects.o6077, NULL},
    {0x607A, 0x01, ODT_VAR, &odObjects.o607A, NULL},
    {0x6081, 0x01, ODT_VAR, &odObjects.o6081, NULL},
    {0x6083, 0x01, ODT_VAR, &odObjects.o6083, NULL},
    {0x6084, 0x01, ODT_VAR, &odObjects.o6084, NULL},
    {0x60FF, 0x01, ODT_VAR, &odObjects.o60FF, NULL},
    {0x6502, 0x01, ODT_VAR, &odObjects.o6502, NULL},
    {0x0000, 0x00, 0, NULL, NULL},
};

static OD_t od = {
    .size = (sizeof(odList) / sizeof(odList[0])) - 1U,
    .list = &odList[0],
};

OD_t *OD = &od;
