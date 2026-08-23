#ifndef BRUSHLESS_ENCODER_H
#define BRUSHLESS_ENCODER_H

#include <stdbool.h>
#include <stdint.h>

enum {
    ENCODER_HEALTH_OK = 0U,
    ENCODER_HEALTH_NO_MAGNET = 1U << 0,
    ENCODER_HEALTH_MAGNET_WEAK = 1U << 1,
    ENCODER_HEALTH_MAGNET_STRONG = 1U << 2,
    ENCODER_HEALTH_OVERSPEED = 1U << 3,
    ENCODER_HEALTH_CRC = 1U << 4,
    ENCODER_HEALTH_BUS = 1U << 5,
    ENCODER_HEALTH_STALE = 1U << 6,
    ENCODER_HEALTH_STATUS_RESERVED = 1U << 7
};

typedef struct {
    float mechanical_angle_rad;
    uint16_t raw_count;
    uint16_t counts_per_turn;
    uint32_t timestamp_us;
    uint32_t health_flags;
    bool valid;
} encoder_sample_t;

#endif
