#ifndef ROLLER_CO_TRANSPORT_H
#define ROLLER_CO_TRANSPORT_H
#include <stdbool.h>
#include <stdint.h>
#ifdef __cplusplus
#define ROLLER_CAN_DEFAULT(value) = value
extern "C" {
#else
#define ROLLER_CAN_DEFAULT(value)
#endif
typedef struct {
    uint16_t ident;
    uint8_t size;
    uint8_t data[64];
    bool synchronous;
    bool fd ROLLER_CAN_DEFAULT(false);
    bool brs ROLLER_CAN_DEFAULT(false);
} RollerCanFrame;

/* Lengths are byte counts, never encoded DLC values. Invalid inputs return 255. */
static inline uint8_t roller_can_dlc_to_length(uint8_t dlc) {
    const uint8_t lengths[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
    return dlc < 16 ? lengths[dlc] : 255;
}
static inline uint8_t roller_can_length_to_dlc(uint8_t length) {
    if (length > 64) {
        return 255;
    }
    for (uint8_t dlc = 0; dlc < 16; ++dlc) {
        if (length <= roller_can_dlc_to_length(dlc)) {
            return dlc;
        }
    }
    return 255;
}
static inline uint8_t roller_can_wire_size(uint8_t length) {
    return roller_can_dlc_to_length(roller_can_length_to_dlc(length));
}
static inline bool roller_can_valid_nominal_bitrate(uint32_t bitrate) {
    return bitrate == 125000 || bitrate == 250000 || bitrate == 500000 || bitrate == 1000000;
}
static inline bool roller_can_valid_data_bitrate(uint32_t bitrate) {
    return bitrate == 1000000 || bitrate == 2000000 || bitrate == 4000000 ||
           bitrate == 5000000 || bitrate == 8000000;
}
typedef struct {
    void* context;
    bool (*start)(void*);
    void (*stop)(void*);
    bool (*send)(void*, const RollerCanFrame*);
    uint16_t (*errors)(void*);
    void (*cancel_sync)(void*);
    /* Called while stopped, before start; settings take effect only at boot/reset. */
    bool (*configure)(void*, uint32_t nominal_bitrate, uint32_t data_bitrate, bool brs)
        ROLLER_CAN_DEFAULT(nullptr);
} RollerCanTransport;
#ifdef __cplusplus
}
#endif
#undef ROLLER_CAN_DEFAULT
#endif
