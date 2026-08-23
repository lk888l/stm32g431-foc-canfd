#ifndef APP_CONTROL_CONTROL_TRAJECTORY_H
#define APP_CONTROL_CONTROL_TRAJECTORY_H

/**
 * @file control_trajectory.h
 * @brief Acceleration-limited PV and trapezoidal PP command generators.
 *
 * Position, velocity and acceleration use rad, rad/s and rad/s^2. The same
 * implementation can be used for linear motion if the caller consistently
 * substitutes metres, m/s and m/s^2.
 */

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float maximum_velocity_rad_s;
    float maximum_acceleration_rad_s2;
    float maximum_deceleration_rad_s2;
    float position_tolerance_rad;
    float velocity_tolerance_rad_s;
    /** Scheduler-stall guard; larger update intervals are rejected. */
    float max_timestep_s;
} control_trajectory_limits_t;

typedef struct {
    float position_rad;
    float velocity_rad_s;
    bool target_reached;
    bool initialized;
} control_trajectory_state_t;

bool control_trajectory_limits_are_valid(
    const control_trajectory_limits_t *limits);

bool control_trajectory_init(control_trajectory_state_t *state,
                             float initial_position_rad,
                             float initial_velocity_rad_s);

/**
 * Profile Velocity (PV): slew velocity using separate acceleration and
 * deceleration limits and integrate a continuous position command.
 */
bool control_trajectory_step_pv(control_trajectory_state_t *state,
                                const control_trajectory_limits_t *limits,
                                float target_velocity_rad_s,
                                float timestep_s);

/**
 * Profile Position (PP): generate a rest-to-rest trapezoidal/triangular
 * profile. The braking velocity sqrt(2*a*distance) is recomputed each step,
 * which also permits a target to be changed while a move is active.
 */
bool control_trajectory_step_pp(control_trajectory_state_t *state,
                                const control_trajectory_limits_t *limits,
                                float target_position_rad,
                                float timestep_s);

#ifdef __cplusplus
}
#endif

#endif /* APP_CONTROL_CONTROL_TRAJECTORY_H */
