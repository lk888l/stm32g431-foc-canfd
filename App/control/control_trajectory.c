#include "control_trajectory.h"

#include "foc_math.h"

#include <math.h>
#include <stddef.h>

static bool control_trajectory_float_is_finite(float value)
{
    return isfinite(value) != 0;
}

static bool control_integrate_ramp(float value,
                                   float target,
                                   float rate,
                                   float timestep_s,
                                   float *next_value,
                                   float *integral)
{
    const float delta = target - value;
    float next;
    float area;

    if (delta == 0.0f) {
        next = target;
        area = target * timestep_s;
    } else {
        const float time_to_target = fabsf(delta) / rate;

        if (time_to_target < timestep_s) {
            next = target;
            area = (((0.5f * value) + (0.5f * target)) *
                    time_to_target) +
                   (target * (timestep_s - time_to_target));
        } else {
            const float signed_rate = (delta > 0.0f) ? rate : -rate;
            next = value + (signed_rate * timestep_s);
            area = ((0.5f * value) + (0.5f * next)) * timestep_s;
        }
    }

    if (!control_trajectory_float_is_finite(next) ||
        !control_trajectory_float_is_finite(area)) {
        return false;
    }
    *next_value = next;
    *integral = area;
    return true;
}

static bool control_advance_velocity(float old_velocity,
                                     float target_velocity,
                                     float acceleration,
                                     float deceleration,
                                     float timestep_s,
                                     float *next_velocity,
                                     float *position_delta)
{
    float next;
    float delta;
    const bool reversing =
        ((old_velocity > 0.0f) && (target_velocity < 0.0f)) ||
        ((old_velocity < 0.0f) && (target_velocity > 0.0f));

    if (reversing) {
        const float time_to_zero = fabsf(old_velocity) / deceleration;

        if (time_to_zero < timestep_s) {
            const float remaining_s = timestep_s - time_to_zero;
            const float first_delta =
                (0.5f * old_velocity) * time_to_zero;
            float second_delta;

            if (!control_integrate_ramp(0.0f, target_velocity,
                                        acceleration, remaining_s,
                                        &next, &second_delta)) {
                return false;
            }
            delta = first_delta + second_delta;
        } else {
            if (!control_integrate_ramp(old_velocity, 0.0f,
                                        deceleration, timestep_s,
                                        &next, &delta)) {
                return false;
            }
        }
    } else {
        const bool speeding_up =
            fabsf(target_velocity) > fabsf(old_velocity);
        const float rate = speeding_up ? acceleration : deceleration;

        if (!control_integrate_ramp(old_velocity, target_velocity,
                                    rate, timestep_s, &next, &delta)) {
            return false;
        }
    }

    if (!control_trajectory_float_is_finite(next) ||
        !control_trajectory_float_is_finite(delta)) {
        return false;
    }
    *next_velocity = next;
    *position_delta = delta;
    return true;
}

static bool control_trajectory_step_inputs_are_valid(
    const control_trajectory_state_t *state,
    const control_trajectory_limits_t *limits,
    float target,
    float timestep_s)
{
    return (state != NULL) && state->initialized &&
           control_trajectory_float_is_finite(state->position_rad) &&
           control_trajectory_float_is_finite(state->velocity_rad_s) &&
           control_trajectory_limits_are_valid(limits) &&
           control_trajectory_float_is_finite(target) &&
           control_trajectory_float_is_finite(timestep_s) &&
           (timestep_s > 0.0f) && (timestep_s <= limits->max_timestep_s);
}

bool control_trajectory_limits_are_valid(
    const control_trajectory_limits_t *limits)
{
    if (limits == NULL) {
        return false;
    }

    return control_trajectory_float_is_finite(limits->maximum_velocity_rad_s) &&
           control_trajectory_float_is_finite(
               limits->maximum_acceleration_rad_s2) &&
           control_trajectory_float_is_finite(
               limits->maximum_deceleration_rad_s2) &&
           control_trajectory_float_is_finite(limits->position_tolerance_rad) &&
           control_trajectory_float_is_finite(
               limits->velocity_tolerance_rad_s) &&
           control_trajectory_float_is_finite(limits->max_timestep_s) &&
           (limits->maximum_velocity_rad_s > 0.0f) &&
           (limits->maximum_acceleration_rad_s2 > 0.0f) &&
           (limits->maximum_deceleration_rad_s2 > 0.0f) &&
           (limits->position_tolerance_rad >= 0.0f) &&
           (limits->velocity_tolerance_rad_s >= 0.0f) &&
           (limits->max_timestep_s > 0.0f);
}

bool control_trajectory_init(control_trajectory_state_t *state,
                             float initial_position_rad,
                             float initial_velocity_rad_s)
{
    if ((state == NULL) ||
        !control_trajectory_float_is_finite(initial_position_rad) ||
        !control_trajectory_float_is_finite(initial_velocity_rad_s)) {
        return false;
    }

    state->position_rad = initial_position_rad;
    state->velocity_rad_s = initial_velocity_rad_s;
    state->target_reached = false;
    state->initialized = true;
    return true;
}

bool control_trajectory_step_pv(control_trajectory_state_t *state,
                                const control_trajectory_limits_t *limits,
                                float target_velocity_rad_s,
                                float timestep_s)
{
    float target_velocity;
    float next_velocity;
    float next_position;
    float position_delta;
    const float old_velocity = (state != NULL) ? state->velocity_rad_s : 0.0f;

    if (!control_trajectory_step_inputs_are_valid(state, limits,
                                                   target_velocity_rad_s,
                                                   timestep_s)) {
        return false;
    }

    target_velocity = control_clampf(target_velocity_rad_s,
                                      -limits->maximum_velocity_rad_s,
                                      limits->maximum_velocity_rad_s);

    /* A reversal is integrated in two pieces: decelerate exactly to zero,
     * then use only the remaining time to accelerate in the new direction. */
    if (!control_advance_velocity(old_velocity, target_velocity,
                                  limits->maximum_acceleration_rad_s2,
                                  limits->maximum_deceleration_rad_s2,
                                  timestep_s, &next_velocity,
                                  &position_delta)) {
        return false;
    }
    next_velocity = control_clampf(next_velocity,
                                    -limits->maximum_velocity_rad_s,
                                    limits->maximum_velocity_rad_s);
    next_position = state->position_rad + position_delta;

    if (!control_trajectory_float_is_finite(next_position)) {
        return false;
    }

    state->velocity_rad_s = next_velocity;
    state->position_rad = next_position;
    state->target_reached =
        fabsf(next_velocity - target_velocity) <=
        limits->velocity_tolerance_rad_s;
    return true;
}

bool control_trajectory_step_pp(control_trajectory_state_t *state,
                                const control_trajectory_limits_t *limits,
                                float target_position_rad,
                                float timestep_s)
{
    float error;
    float direction;
    float braking_distance_error;
    float braking_energy;
    float braking_velocity;
    float desired_velocity;
    float next_velocity;
    float next_position;
    float next_error;
    float position_delta;
    bool crossed_target;
    const float old_velocity = (state != NULL) ? state->velocity_rad_s : 0.0f;

    if (!control_trajectory_step_inputs_are_valid(state, limits,
                                                   target_position_rad,
                                                   timestep_s)) {
        return false;
    }

    error = target_position_rad - state->position_rad;
    if (!control_trajectory_float_is_finite(error)) {
        return false;
    }
    if ((fabsf(error) <= limits->position_tolerance_rad) &&
        (fabsf(old_velocity) <= limits->velocity_tolerance_rad_s)) {
        state->position_rad = target_position_rad;
        state->velocity_rad_s = 0.0f;
        state->target_reached = true;
        return true;
    }

    direction = (error >= 0.0f) ? 1.0f : -1.0f;
    braking_distance_error = fabsf(error) - limits->position_tolerance_rad;
    if (braking_distance_error < 0.0f) {
        braking_distance_error = 0.0f;
    }
    braking_energy = 2.0f * limits->maximum_deceleration_rad_s2 *
                     braking_distance_error;
    braking_velocity = control_trajectory_float_is_finite(braking_energy)
                           ? sqrtf(braking_energy)
                           : limits->maximum_velocity_rad_s;
    desired_velocity = direction * fminf(limits->maximum_velocity_rad_s,
                                          braking_velocity);

    if (!control_advance_velocity(old_velocity, desired_velocity,
                                  limits->maximum_acceleration_rad_s2,
                                  limits->maximum_deceleration_rad_s2,
                                  timestep_s, &next_velocity,
                                  &position_delta)) {
        return false;
    }
    next_velocity = control_clampf(next_velocity,
                                    -limits->maximum_velocity_rad_s,
                                    limits->maximum_velocity_rad_s);
    next_position = state->position_rad + position_delta;

    if (!control_trajectory_float_is_finite(next_position)) {
        return false;
    }
    next_error = target_position_rad - next_position;
    if (!control_trajectory_float_is_finite(next_error)) {
        return false;
    }
    crossed_target = ((error >= 0.0f) && (next_error <= 0.0f)) ||
                     ((error <= 0.0f) && (next_error >= 0.0f));

    /* Quantization/discrete-time crossing near zero speed is snapped exactly
     * to the target, preventing a one-count limit cycle at standstill. */
    if (crossed_target &&
        (fabsf(next_velocity) <=
         ((limits->maximum_deceleration_rad_s2 * timestep_s) +
          limits->velocity_tolerance_rad_s))) {
        next_position = target_position_rad;
        next_velocity = 0.0f;
    }

    state->position_rad = next_position;
    state->velocity_rad_s = next_velocity;
    state->target_reached =
        (fabsf(target_position_rad - next_position) <=
         limits->position_tolerance_rad) &&
        (fabsf(next_velocity) <= limits->velocity_tolerance_rad_s);

    if (state->target_reached) {
        state->position_rad = target_position_rad;
        state->velocity_rad_s = 0.0f;
    }
    return true;
}
