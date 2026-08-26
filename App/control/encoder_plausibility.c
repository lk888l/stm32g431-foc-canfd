#include "encoder_plausibility.h"

#include <math.h>

#define ENCODER_PLAUSIBILITY_TWO_PI_F 6.2831853071795864769f
#define ENCODER_PLAUSIBILITY_US_PER_S 1000000.0f

uint16_t encoder_plausibility_distance_counts(uint16_t first_count,
                                              uint16_t second_count,
                                              uint16_t counts_per_turn)
{
    uint32_t distance;
    uint32_t wrapped_distance;

    if ((counts_per_turn < 2U) || (first_count >= counts_per_turn) ||
        (second_count >= counts_per_turn)) {
        return UINT16_MAX;
    }

    if (first_count >= second_count) {
        distance = (uint32_t)first_count - (uint32_t)second_count;
    } else {
        distance = (uint32_t)second_count - (uint32_t)first_count;
    }
    wrapped_distance = (uint32_t)counts_per_turn - distance;
    if (wrapped_distance < distance) {
        distance = wrapped_distance;
    }
    return (uint16_t)distance;
}

bool encoder_plausibility_step(uint16_t previous_count,
                               uint16_t candidate_count,
                               uint16_t counts_per_turn,
                               uint32_t elapsed_us,
                               float maximum_speed_rad_s,
                               uint16_t margin_counts)
{
    const uint16_t distance = encoder_plausibility_distance_counts(
        previous_count, candidate_count, counts_per_turn);
    const uint32_t half_turn_counts = (uint32_t)counts_per_turn / 2U;
    float allowed_counts_f;
    uint32_t allowed_counts;

    if ((distance == UINT16_MAX) || !isfinite(maximum_speed_rad_s) ||
        (maximum_speed_rad_s < 0.0f)) {
        return false;
    }
    if ((uint32_t)margin_counts >= half_turn_counts) {
        return true;
    }

    allowed_counts_f =
        (maximum_speed_rad_s * (float)elapsed_us *
         (float)counts_per_turn) /
        (ENCODER_PLAUSIBILITY_TWO_PI_F *
         ENCODER_PLAUSIBILITY_US_PER_S);
    allowed_counts_f += (float)margin_counts;
    if (!isfinite(allowed_counts_f) ||
        (allowed_counts_f >= (float)half_turn_counts)) {
        return true;
    }

    /* Round the physical envelope upward: this check must reject gross bus
     * corruption, not turn a floating-point rounding edge into a false trip. */
    allowed_counts = (uint32_t)allowed_counts_f;
    if ((float)allowed_counts < allowed_counts_f) {
        ++allowed_counts;
    }
    return (uint32_t)distance <= allowed_counts;
}
