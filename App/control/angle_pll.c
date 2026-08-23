#include "angle_pll.h"

#include "foc_math.h"

#include <math.h>
#include <stddef.h>

static bool control_angle_pll_float_is_finite(float value)
{
    return isfinite(value) != 0;
}

bool control_angle_pll_config_is_valid(
    const control_angle_pll_config_t *config)
{
    if (config == NULL) {
        return false;
    }

    return control_angle_pll_float_is_finite(config->proportional_gain_per_s) &&
           control_angle_pll_float_is_finite(config->integral_gain_per_s2) &&
           control_angle_pll_float_is_finite(config->maximum_speed_rad_s) &&
           control_angle_pll_float_is_finite(config->max_timestep_s) &&
           (config->proportional_gain_per_s >= 0.0f) &&
           (config->integral_gain_per_s2 >= 0.0f) &&
           (config->maximum_speed_rad_s > 0.0f) &&
           (config->max_timestep_s > 0.0f);
}

bool control_angle_pll_init(control_angle_pll_t *pll,
                            const control_angle_pll_config_t *config,
                            float initial_wrapped_angle_rad)
{
    if ((pll == NULL) || !control_angle_pll_config_is_valid(config) ||
        !control_angle_pll_float_is_finite(initial_wrapped_angle_rad)) {
        return false;
    }

    pll->config = *config;
    pll->continuous_angle_rad =
        control_angle_wrap_pm_pi(initial_wrapped_angle_rad);
    pll->wrapped_angle_rad = pll->continuous_angle_rad;
    pll->continuous_angle_precise_rad =
        (double)pll->continuous_angle_rad;
    pll->integral_speed_rad_s = 0.0f;
    pll->speed_rad_s = 0.0f;
    pll->phase_error_rad = 0.0f;
    pll->initialized = true;
    return true;
}

bool control_angle_pll_update(control_angle_pll_t *pll,
                              float measured_wrapped_angle_rad,
                              float timestep_s)
{
    float phase_error;
    float integral_speed;
    float corrected_speed;
    float integral_increment;
    float proportional_correction;
    float angle_increment;
    float next_wrapped_angle;
    float next_angle;
    double next_precise_angle;

    if ((pll == NULL) || !pll->initialized ||
        !control_angle_pll_float_is_finite(measured_wrapped_angle_rad) ||
        !control_angle_pll_float_is_finite(timestep_s) ||
        (timestep_s <= 0.0f) ||
        (timestep_s > pll->config.max_timestep_s)) {
        return false;
    }

    phase_error = control_angle_wrap_pm_pi(
        measured_wrapped_angle_rad -
        pll->wrapped_angle_rad);

    integral_increment =
        pll->config.integral_gain_per_s2 * phase_error * timestep_s;
    proportional_correction =
        pll->config.proportional_gain_per_s * phase_error;
    if (!control_angle_pll_float_is_finite(integral_increment) ||
        !control_angle_pll_float_is_finite(proportional_correction)) {
        return false;
    }

    integral_speed = pll->integral_speed_rad_s + integral_increment;
    if (!control_angle_pll_float_is_finite(integral_speed)) {
        return false;
    }
    integral_speed = control_clampf(integral_speed,
                                     -pll->config.maximum_speed_rad_s,
                                     pll->config.maximum_speed_rad_s);

    corrected_speed = integral_speed + proportional_correction;
    if (!control_angle_pll_float_is_finite(corrected_speed)) {
        return false;
    }
    corrected_speed = control_clampf(corrected_speed,
                                      -pll->config.maximum_speed_rad_s,
                                      pll->config.maximum_speed_rad_s);
    angle_increment = corrected_speed * timestep_s;
    if (!control_angle_pll_float_is_finite(angle_increment)) {
        return false;
    }
    next_wrapped_angle = control_angle_wrap_pm_pi(
        pll->wrapped_angle_rad + angle_increment);
    next_precise_angle = pll->continuous_angle_precise_rad +
                         (double)angle_increment;
    next_angle = (float)next_precise_angle;

    if (!control_angle_pll_float_is_finite(next_wrapped_angle) ||
        !isfinite(next_precise_angle) ||
        !control_angle_pll_float_is_finite(next_angle)) {
        return false;
    }

    pll->phase_error_rad = phase_error;
    pll->integral_speed_rad_s = integral_speed;
    pll->speed_rad_s = corrected_speed;
    pll->wrapped_angle_rad = next_wrapped_angle;
    pll->continuous_angle_precise_rad = next_precise_angle;
    pll->continuous_angle_rad = next_angle;
    return true;
}
