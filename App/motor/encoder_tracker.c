#include "encoder_tracker.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>

#define TWO_PI_F 6.2831853071795864769f
#define Q16_SCALE 65536.0

bool encoder_tracker_update(encoder_tracker_t *tracker,
                            uint16_t raw_count,
                            uint16_t counts_per_turn)
{
    int32_t delta;
    int32_t half_turn;

    if ((tracker == NULL) || (counts_per_turn < 4U) ||
        (raw_count >= counts_per_turn)) {
        return false;
    }
    if (!tracker->initialized ||
        (tracker->counts_per_turn != counts_per_turn)) {
        tracker->accumulated_counts = raw_count;
        tracker->zero_offset_counts = raw_count;
        tracker->previous_raw = raw_count;
        tracker->counts_per_turn = counts_per_turn;
        tracker->initialized = true;
        return true;
    }

    delta = (int32_t)raw_count - (int32_t)tracker->previous_raw;
    half_turn = (int32_t)counts_per_turn / 2;
    if (delta > half_turn) {
        delta -= (int32_t)counts_per_turn;
    } else if (delta < -half_turn) {
        delta += (int32_t)counts_per_turn;
    }
    tracker->accumulated_counts += delta;
    tracker->previous_raw = raw_count;
    return true;
}

void encoder_tracker_zero_at_current(encoder_tracker_t *tracker)
{
    if ((tracker != NULL) && tracker->initialized) {
        tracker->zero_offset_counts = tracker->accumulated_counts;
    }
}

float encoder_tracker_position_rad(const encoder_tracker_t *tracker)
{
    if ((tracker == NULL) || !tracker->initialized) {
        return 0.0f;
    }
    return (float)(tracker->accumulated_counts - tracker->zero_offset_counts) *
           TWO_PI_F / (float)tracker->counts_per_turn;
}

int32_t encoder_tracker_position_q16_16(const encoder_tracker_t *tracker,
                                        bool *saturated)
{
    double scaled;

    if (saturated != NULL) {
        *saturated = false;
    }
    if ((tracker == NULL) || !tracker->initialized) {
        return 0;
    }
    scaled = (double)(tracker->accumulated_counts - tracker->zero_offset_counts) *
             (6.2831853071795864769 * Q16_SCALE) /
             (double)tracker->counts_per_turn;
    if (scaled >= (double)INT32_MAX) {
        if (saturated != NULL) {
            *saturated = true;
        }
        return INT32_MAX;
    }
    if (scaled <= (double)INT32_MIN) {
        if (saturated != NULL) {
            *saturated = true;
        }
        return INT32_MIN;
    }
    return (int32_t)llround(scaled);
}
