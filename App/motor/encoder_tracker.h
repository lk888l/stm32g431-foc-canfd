#ifndef BRUSHLESS_ENCODER_TRACKER_H
#define BRUSHLESS_ENCODER_TRACKER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int64_t accumulated_counts;
    int64_t zero_offset_counts;
    uint16_t previous_raw;
    uint16_t counts_per_turn;
    bool initialized;
} encoder_tracker_t;

bool encoder_tracker_update(encoder_tracker_t *tracker,
                            uint16_t raw_count,
                            uint16_t counts_per_turn);
void encoder_tracker_zero_at_current(encoder_tracker_t *tracker);
float encoder_tracker_position_rad(const encoder_tracker_t *tracker);
int32_t encoder_tracker_position_q16_16(const encoder_tracker_t *tracker,
                                        bool *saturated);

#endif
