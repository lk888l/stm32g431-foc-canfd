#ifndef APP_CONTROL_CONTROL_PI_H
#define APP_CONTROL_CONTROL_PI_H

/**
 * @file control_pi.h
 * @brief Deterministic PI controller with back-calculation anti-windup.
 *
 * Gains and limits use the SI units of the controlled signal. No allocation,
 * clock access, or hardware access occurs in this module.
 */

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float proportional_gain;
    float integral_gain_per_s;
    /** Back-calculation gain in 1/s. Zero disables back-calculation. */
    float anti_windup_gain_per_s;
    float output_min;
    float output_max;
    float integrator_min;
    float integrator_max;
    /** Reject larger time steps so a stalled scheduler cannot cause a jump. */
    float max_timestep_s;
} control_pi_config_t;

typedef struct {
    control_pi_config_t config;
    float integrator;
    float output;
    bool initialized;
} control_pi_t;

typedef struct {
    float proportional;
    float integrator;
    float unsaturated_output;
    float output;
    bool saturated;
} control_pi_result_t;

bool control_pi_config_is_valid(const control_pi_config_t *config);

/** Initialize and reset a controller. */
bool control_pi_init(control_pi_t *controller,
                     const control_pi_config_t *config);

/**
 * Reset controller memory for a bumpless transition.
 * integrator and output are clamped to their configured limits.
 */
bool control_pi_reset(control_pi_t *controller,
                      float integrator,
                      float output);

/**
 * Execute one PI step.
 *
 * u = Kp*error + integrator + feed_forward
 * d(integrator)/dt = Ki*error + Kaw*(u_limited - u_unlimited)
 *
 * Invalid data or a time step outside (0, max_timestep_s] is rejected without
 * modifying controller state.
 */
bool control_pi_step(control_pi_t *controller,
                     float error,
                     float feed_forward,
                     float timestep_s,
                     control_pi_result_t *result);

/**
 * Track the outputs actually applied by a coupled two-axis actuator.
 *
 * A per-axis PI cannot account for a later vector-magnitude limiter by
 * itself.  After both PI steps and the coupled limiter, this function applies
 * the additional back-calculation term to each integrator:
 *
 *   delta_integrator = Kaw * (u_applied - u_axis) * dt
 *
 * u_axis is the controller's most recent per-axis limited output and
 * u_applied is the value delivered by the coupled limiter.  Kaw is in 1/s;
 * therefore, for a voltage-output PI, the tracking error and integrator
 * correction are both in volts.
 *
 * Both candidate integrators are validated and clamped before either
 * controller is modified.  Invalid/non-finite data, aliased controllers, an
 * applied output outside its axis limits, or a time step outside
 * (0, max_timestep_s] leaves both controllers unchanged.  On success each
 * controller's output records u_applied for truthful actuator diagnostics.
 */
bool control_pi_track_applied_pair(control_pi_t *first_controller,
                                   float first_applied_output,
                                   control_pi_t *second_controller,
                                   float second_applied_output,
                                   float timestep_s);

#ifdef __cplusplus
}
#endif

#endif /* APP_CONTROL_CONTROL_PI_H */
