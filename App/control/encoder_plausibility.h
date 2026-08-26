#ifndef BRUSHLESS_ENCODER_PLAUSIBILITY_H
#define BRUSHLESS_ENCODER_PLAUSIBILITY_H

#include <stdbool.h>
#include <stdint.h>

/* Return the shortest circular distance between two absolute encoder counts.
 * Invalid inputs return UINT16_MAX so callers fail closed. */
uint16_t encoder_plausibility_distance_counts(uint16_t first_count,
                                              uint16_t second_count,
                                              uint16_t counts_per_turn);

/* Check a new absolute count against a speed envelope plus a fixed allowance
 * for quantization, filter latency and sampling jitter. elapsed_us is intended
 * to be obtained by unsigned timestamp subtraction, so timer wrap is safe. */
bool encoder_plausibility_step(uint16_t previous_count,
                               uint16_t candidate_count,
                               uint16_t counts_per_turn,
                               uint32_t elapsed_us,
                               float maximum_speed_rad_s,
                               uint16_t margin_counts);

#endif
