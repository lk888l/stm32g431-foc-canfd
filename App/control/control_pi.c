#include "control_pi.h"

#include "foc_math.h"

#include <math.h>
#include <stddef.h>

static bool control_pi_float_is_finite(float value)
{
    return isfinite(value) != 0;
}

bool control_pi_config_is_valid(const control_pi_config_t *config)
{
    if (config == NULL) {
        return false;
    }

    return control_pi_float_is_finite(config->proportional_gain) &&
           control_pi_float_is_finite(config->integral_gain_per_s) &&
           control_pi_float_is_finite(config->anti_windup_gain_per_s) &&
           control_pi_float_is_finite(config->output_min) &&
           control_pi_float_is_finite(config->output_max) &&
           control_pi_float_is_finite(config->integrator_min) &&
           control_pi_float_is_finite(config->integrator_max) &&
           control_pi_float_is_finite(config->max_timestep_s) &&
           (config->proportional_gain >= 0.0f) &&
           (config->integral_gain_per_s >= 0.0f) &&
           (config->anti_windup_gain_per_s >= 0.0f) &&
           (config->output_min <= config->output_max) &&
           (config->integrator_min <= config->integrator_max) &&
           (config->max_timestep_s > 0.0f);
}

bool control_pi_init(control_pi_t *controller,
                     const control_pi_config_t *config)
{
    if ((controller == NULL) || !control_pi_config_is_valid(config)) {
        return false;
    }

    controller->config = *config;
    controller->integrator = 0.0f;
    controller->output = control_clampf(0.0f,
                                        config->output_min,
                                        config->output_max);
    controller->initialized = true;
    return true;
}

bool control_pi_reset(control_pi_t *controller,
                      float integrator,
                      float output)
{
    if ((controller == NULL) || !controller->initialized ||
        !control_pi_float_is_finite(integrator) ||
        !control_pi_float_is_finite(output)) {
        return false;
    }

    controller->integrator = control_clampf(integrator,
                                             controller->config.integrator_min,
                                             controller->config.integrator_max);
    controller->output = control_clampf(output,
                                         controller->config.output_min,
                                         controller->config.output_max);
    return true;
}

bool control_pi_step(control_pi_t *controller,
                     float error,
                     float feed_forward,
                     float timestep_s,
                     control_pi_result_t *result)
{
    float proportional;
    float integrated;
    float preliminary_unsaturated;
    float preliminary_limited;
    float corrected_integrator;
    float unsaturated;
    float limited;

    if ((controller == NULL) || (result == NULL) ||
        !controller->initialized ||
        !control_pi_float_is_finite(error) ||
        !control_pi_float_is_finite(feed_forward) ||
        !control_pi_float_is_finite(timestep_s) ||
        (timestep_s <= 0.0f) ||
        (timestep_s > controller->config.max_timestep_s)) {
        return false;
    }

    proportional = controller->config.proportional_gain * error;
    integrated = controller->integrator +
                 (controller->config.integral_gain_per_s * error * timestep_s);
    preliminary_unsaturated = proportional + integrated + feed_forward;
    preliminary_limited = control_clampf(preliminary_unsaturated,
                                          controller->config.output_min,
                                          controller->config.output_max);

    corrected_integrator = integrated +
        (controller->config.anti_windup_gain_per_s *
         (preliminary_limited - preliminary_unsaturated) * timestep_s);
    corrected_integrator = control_clampf(corrected_integrator,
                                           controller->config.integrator_min,
                                           controller->config.integrator_max);

    unsaturated = proportional + corrected_integrator + feed_forward;
    limited = control_clampf(unsaturated,
                              controller->config.output_min,
                              controller->config.output_max);

    /* Reject arithmetic overflow instead of committing corrupt state. */
    if (!control_pi_float_is_finite(proportional) ||
        !control_pi_float_is_finite(corrected_integrator) ||
        !control_pi_float_is_finite(unsaturated) ||
        !control_pi_float_is_finite(limited)) {
        return false;
    }

    controller->integrator = corrected_integrator;
    controller->output = limited;

    result->proportional = proportional;
    result->integrator = corrected_integrator;
    result->unsaturated_output = unsaturated;
    result->output = limited;
    result->saturated = (unsaturated < controller->config.output_min) ||
                        (unsaturated > controller->config.output_max);
    return true;
}

static bool control_pi_tracking_candidate(const control_pi_t *controller,
                                          float applied_output,
                                          float timestep_s,
                                          float *corrected_integrator)
{
    float tracking_error;
    float tracking_correction;
    float candidate;

    if ((controller == NULL) || (corrected_integrator == NULL) ||
        !controller->initialized ||
        !control_pi_config_is_valid(&controller->config) ||
        !control_pi_float_is_finite(controller->integrator) ||
        !control_pi_float_is_finite(controller->output) ||
        !control_pi_float_is_finite(applied_output) ||
        !control_pi_float_is_finite(timestep_s) ||
        (applied_output < controller->config.output_min) ||
        (applied_output > controller->config.output_max) ||
        (timestep_s <= 0.0f) ||
        (timestep_s > controller->config.max_timestep_s)) {
        return false;
    }

    tracking_error = applied_output - controller->output;
    tracking_correction = controller->config.anti_windup_gain_per_s *
                          tracking_error * timestep_s;
    candidate = controller->integrator + tracking_correction;
    if (!control_pi_float_is_finite(tracking_error) ||
        !control_pi_float_is_finite(tracking_correction) ||
        !control_pi_float_is_finite(candidate)) {
        return false;
    }

    candidate = control_clampf(candidate,
                               controller->config.integrator_min,
                               controller->config.integrator_max);
    if (!control_pi_float_is_finite(candidate)) {
        return false;
    }
    *corrected_integrator = candidate;
    return true;
}

bool control_pi_track_applied(control_pi_t *controller,
                              float applied_output,
                              float timestep_s)
{
    float corrected_integrator;

    if (!control_pi_tracking_candidate(controller,
                                       applied_output,
                                       timestep_s,
                                       &corrected_integrator)) {
        return false;
    }
    controller->integrator = corrected_integrator;
    controller->output = applied_output;
    return true;
}

bool control_pi_track_applied_pair(control_pi_t *first_controller,
                                   float first_applied_output,
                                   control_pi_t *second_controller,
                                   float second_applied_output,
                                   float timestep_s)
{
    float first_integrator;
    float second_integrator;

    if ((first_controller == second_controller) ||
        !control_pi_tracking_candidate(first_controller,
                                       first_applied_output,
                                       timestep_s,
                                       &first_integrator) ||
        !control_pi_tracking_candidate(second_controller,
                                       second_applied_output,
                                       timestep_s,
                                       &second_integrator)) {
        return false;
    }

    /* Commit only after both axes have a valid candidate. */
    first_controller->integrator = first_integrator;
    second_controller->integrator = second_integrator;
    first_controller->output = first_applied_output;
    second_controller->output = second_applied_output;
    return true;
}
