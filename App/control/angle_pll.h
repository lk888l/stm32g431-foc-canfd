#ifndef APP_CONTROL_ANGLE_PLL_H
#define APP_CONTROL_ANGLE_PLL_H

/**
 * @file angle_pll.h
 * @brief Wrapped-angle type-II PLL for continuous angle and speed estimation.
 */

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /** Proportional phase correction, (rad/s)/rad. */
    float proportional_gain_per_s;
    /** Integral frequency correction, (rad/s^2)/rad. */
    float integral_gain_per_s2;
    float maximum_speed_rad_s;
    float max_timestep_s;
} control_angle_pll_config_t;

typedef struct {
    control_angle_pll_config_t config;
    /** Public continuous-angle estimate; precision is naturally float-limited. */
    float continuous_angle_rad;
    /**
     * Small-range phase state used by the observer.  Keeping phase detection
     * independent of a growing continuous float avoids loss of PLL resolution
     * after long operation.
     */
    float wrapped_angle_rad;
    /** High-precision accumulator retained for long-running position users. */
    double continuous_angle_precise_rad;
    /** Integral state of the type-II PLL. */
    float integral_speed_rad_s;
    /** Integral speed plus proportional phase correction. */
    float speed_rad_s;
    float phase_error_rad;
    bool initialized;
} control_angle_pll_t;

bool control_angle_pll_config_is_valid(
    const control_angle_pll_config_t *config);

/** Initialize the PLL from one wrapped mechanical or electrical angle sample. */
bool control_angle_pll_init(control_angle_pll_t *pll,
                            const control_angle_pll_config_t *config,
                            float initial_wrapped_angle_rad);

/**
 * Incorporate one wrapped angle sample.
 *
 * Correct operation requires the true angular advance between samples to be
 * less than pi radians; this is the normal sampling theorem constraint for an
 * incremental wrapped-angle observer. Invalid input leaves state unchanged.
 */
bool control_angle_pll_update(control_angle_pll_t *pll,
                              float measured_wrapped_angle_rad,
                              float timestep_s);

#ifdef __cplusplus
}
#endif

#endif /* APP_CONTROL_ANGLE_PLL_H */
