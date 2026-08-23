#include "foc_math.h"

#include <math.h>
#include <stddef.h>

static bool control_float_is_finite(float value)
{
    return isfinite(value) != 0;
}

static void control_svpwm_safe_output(control_svpwm_output_t *output)
{
    if (output != NULL) {
        output->duty_a = 0.5f;
        output->duty_b = 0.5f;
        output->duty_c = 0.5f;
        output->applied_voltage_v.alpha = 0.0f;
        output->applied_voltage_v.beta = 0.0f;
        output->voltage_scale = 0.0f;
        output->saturated = true;
    }
}

float control_clampf(float value, float lower_bound, float upper_bound)
{
    if (!control_float_is_finite(lower_bound) ||
        !control_float_is_finite(upper_bound) ||
        (lower_bound > upper_bound) || !control_float_is_finite(value)) {
        return lower_bound;
    }

    if (value < lower_bound) {
        return lower_bound;
    }
    if (value > upper_bound) {
        return upper_bound;
    }
    return value;
}

float control_angle_wrap_pm_pi(float angle_rad)
{
    float wrapped;

    if (!control_float_is_finite(angle_rad)) {
        return 0.0f;
    }

    wrapped = fmodf(angle_rad + CONTROL_PI_F, CONTROL_TWO_PI_F);
    if (wrapped < 0.0f) {
        wrapped += CONTROL_TWO_PI_F;
    }
    return wrapped - CONTROL_PI_F;
}

float control_angle_wrap_0_2pi(float angle_rad)
{
    float wrapped;

    if (!control_float_is_finite(angle_rad)) {
        return 0.0f;
    }

    wrapped = fmodf(angle_rad, CONTROL_TWO_PI_F);
    if (wrapped < 0.0f) {
        wrapped += CONTROL_TWO_PI_F;
    }
    return wrapped;
}

bool control_clarke(const control_abc_t *abc, control_alphabeta_t *alphabeta)
{
    float alpha;
    float beta;

    if ((abc == NULL) || (alphabeta == NULL)) {
        return false;
    }
    if (!control_float_is_finite(abc->a) ||
        !control_float_is_finite(abc->b) ||
        !control_float_is_finite(abc->c)) {
        alphabeta->alpha = 0.0f;
        alphabeta->beta = 0.0f;
        return false;
    }

    /* 2/3 amplitude-invariant Clarke transform, including all three phases. */
    alpha = ((2.0f / 3.0f) * abc->a) - ((1.0f / 3.0f) * abc->b) -
            ((1.0f / 3.0f) * abc->c);
    beta = (CONTROL_INV_SQRT3_F * abc->b) -
           (CONTROL_INV_SQRT3_F * abc->c);
    if (!control_float_is_finite(alpha) || !control_float_is_finite(beta)) {
        alphabeta->alpha = 0.0f;
        alphabeta->beta = 0.0f;
        return false;
    }
    alphabeta->alpha = alpha;
    alphabeta->beta = beta;
    return true;
}

bool control_inverse_clarke(const control_alphabeta_t *alphabeta,
                            control_abc_t *abc)
{
    float a;
    float b;
    float c;

    if ((alphabeta == NULL) || (abc == NULL)) {
        return false;
    }
    if (!control_float_is_finite(alphabeta->alpha) ||
        !control_float_is_finite(alphabeta->beta)) {
        abc->a = 0.0f;
        abc->b = 0.0f;
        abc->c = 0.0f;
        return false;
    }

    a = alphabeta->alpha;
    b = (-0.5f * alphabeta->alpha) +
        (CONTROL_SQRT3_BY_2_F * alphabeta->beta);
    c = (-0.5f * alphabeta->alpha) -
        (CONTROL_SQRT3_BY_2_F * alphabeta->beta);
    if (!control_float_is_finite(a) || !control_float_is_finite(b) ||
        !control_float_is_finite(c)) {
        abc->a = 0.0f;
        abc->b = 0.0f;
        abc->c = 0.0f;
        return false;
    }
    abc->a = a;
    abc->b = b;
    abc->c = c;
    return true;
}

bool control_park(const control_alphabeta_t *alphabeta,
                  float electrical_angle_rad,
                  control_dq_t *dq)
{
    float sine;
    float cosine;
    float d;
    float q;

    if ((alphabeta == NULL) || (dq == NULL)) {
        return false;
    }
    if (!control_float_is_finite(alphabeta->alpha) ||
        !control_float_is_finite(alphabeta->beta) ||
        !control_float_is_finite(electrical_angle_rad)) {
        dq->d = 0.0f;
        dq->q = 0.0f;
        return false;
    }

    sine = sinf(electrical_angle_rad);
    cosine = cosf(electrical_angle_rad);
    d = (alphabeta->alpha * cosine) + (alphabeta->beta * sine);
    q = (-alphabeta->alpha * sine) + (alphabeta->beta * cosine);
    if (!control_float_is_finite(d) || !control_float_is_finite(q)) {
        dq->d = 0.0f;
        dq->q = 0.0f;
        return false;
    }
    dq->d = d;
    dq->q = q;
    return true;
}

bool control_inverse_park(const control_dq_t *dq,
                          float electrical_angle_rad,
                          control_alphabeta_t *alphabeta)
{
    float sine;
    float cosine;
    float alpha;
    float beta;

    if ((dq == NULL) || (alphabeta == NULL)) {
        return false;
    }
    if (!control_float_is_finite(dq->d) ||
        !control_float_is_finite(dq->q) ||
        !control_float_is_finite(electrical_angle_rad)) {
        alphabeta->alpha = 0.0f;
        alphabeta->beta = 0.0f;
        return false;
    }

    sine = sinf(electrical_angle_rad);
    cosine = cosf(electrical_angle_rad);
    alpha = (dq->d * cosine) - (dq->q * sine);
    beta = (dq->d * sine) + (dq->q * cosine);
    if (!control_float_is_finite(alpha) || !control_float_is_finite(beta)) {
        alphabeta->alpha = 0.0f;
        alphabeta->beta = 0.0f;
        return false;
    }
    alphabeta->alpha = alpha;
    alphabeta->beta = beta;
    return true;
}

bool control_limit_dq_voltage(const control_dq_t *requested,
                              float max_magnitude_v,
                              control_dq_t *limited,
                              float *applied_scale)
{
    float max_component;
    float normalized_magnitude;
    float scale = 1.0f;
    float limited_d;
    float limited_q;

    if ((requested == NULL) || (limited == NULL)) {
        return false;
    }
    if (!control_float_is_finite(requested->d) ||
        !control_float_is_finite(requested->q) ||
        !control_float_is_finite(max_magnitude_v) ||
        (max_magnitude_v < 0.0f)) {
        limited->d = 0.0f;
        limited->q = 0.0f;
        if (applied_scale != NULL) {
            *applied_scale = 0.0f;
        }
        return false;
    }

    max_component = fmaxf(fabsf(requested->d), fabsf(requested->q));
    if (max_component > 0.0f) {
        normalized_magnitude = hypotf(requested->d / max_component,
                                      requested->q / max_component);
        scale = (max_magnitude_v / max_component) /
                normalized_magnitude;
        if (scale > 1.0f) {
            scale = 1.0f;
        }
    }

    limited_d = requested->d * scale;
    limited_q = requested->q * scale;
    if (!control_float_is_finite(scale) ||
        !control_float_is_finite(limited_d) ||
        !control_float_is_finite(limited_q)) {
        limited->d = 0.0f;
        limited->q = 0.0f;
        if (applied_scale != NULL) {
            *applied_scale = 0.0f;
        }
        return false;
    }
    limited->d = limited_d;
    limited->q = limited_q;
    if (applied_scale != NULL) {
        *applied_scale = scale;
    }
    return true;
}

bool control_svpwm(const control_alphabeta_t *requested_voltage_v,
                    const control_svpwm_config_t *config,
                    control_svpwm_output_t *output)
{
    float normalized_a;
    float normalized_b;
    float normalized_c;
    float normalized_max;
    float normalized_min;
    float normalized_span;
    float input_scale;
    float scaled_alpha;
    float scaled_beta;
    float phase_a;
    float phase_b;
    float phase_c;
    float phase_max;
    float phase_min;
    float available_span;
    float common_mode;
    float min_duty;
    float scale = 1.0f;

    if (output == NULL) {
        return false;
    }
    control_svpwm_safe_output(output);

    if ((requested_voltage_v == NULL) || (config == NULL)) {
        return false;
    }
    if (!control_float_is_finite(requested_voltage_v->alpha) ||
        !control_float_is_finite(requested_voltage_v->beta) ||
        !control_float_is_finite(config->dc_bus_voltage_v) ||
        !control_float_is_finite(config->pwm_period_s) ||
        !control_float_is_finite(config->minimum_pulse_s) ||
        (config->dc_bus_voltage_v <= 0.0f) ||
        (config->pwm_period_s <= 0.0f) ||
        (config->minimum_pulse_s < 0.0f) ||
        (config->minimum_pulse_s >= (0.5f * config->pwm_period_s))) {
        return false;
    }

    min_duty = config->minimum_pulse_s / config->pwm_period_s;

    available_span = config->dc_bus_voltage_v * (1.0f - (2.0f * min_duty));

    /* Normalize first, so neither the phase transform nor span calculation
     * can overflow for extreme-but-finite command values. */
    input_scale = fmaxf(fabsf(requested_voltage_v->alpha),
                        fabsf(requested_voltage_v->beta));
    if (input_scale > 0.0f) {
        normalized_a = requested_voltage_v->alpha / input_scale;
        normalized_b = (-0.5f * normalized_a) +
            (CONTROL_SQRT3_BY_2_F *
             (requested_voltage_v->beta / input_scale));
        normalized_c = (-0.5f * normalized_a) -
            (CONTROL_SQRT3_BY_2_F *
             (requested_voltage_v->beta / input_scale));
        normalized_max = fmaxf(normalized_a,
                                fmaxf(normalized_b, normalized_c));
        normalized_min = fminf(normalized_a,
                                fminf(normalized_b, normalized_c));
        normalized_span = normalized_max - normalized_min;
        scale = (available_span / input_scale) / normalized_span;
        if (scale > 1.0f) {
            scale = 1.0f;
        }
    }

    scaled_alpha = requested_voltage_v->alpha * scale;
    scaled_beta = requested_voltage_v->beta * scale;
    phase_a = scaled_alpha;
    phase_b = (-0.5f * scaled_alpha) +
              (CONTROL_SQRT3_BY_2_F * scaled_beta);
    phase_c = (-0.5f * scaled_alpha) -
              (CONTROL_SQRT3_BY_2_F * scaled_beta);
    if (!control_float_is_finite(scale) ||
        !control_float_is_finite(phase_a) ||
        !control_float_is_finite(phase_b) ||
        !control_float_is_finite(phase_c)) {
        return false;
    }

    phase_max = fmaxf(phase_a, fmaxf(phase_b, phase_c));
    phase_min = fminf(phase_a, fminf(phase_b, phase_c));

    /* Center the active vector in the DC-link window (zero-sequence injection). */
    common_mode = (-0.5f * phase_max) + (-0.5f * phase_min);
    output->duty_a = 0.5f + ((phase_a + common_mode) /
                             config->dc_bus_voltage_v);
    output->duty_b = 0.5f + ((phase_b + common_mode) /
                             config->dc_bus_voltage_v);
    output->duty_c = 0.5f + ((phase_c + common_mode) /
                             config->dc_bus_voltage_v);

    /* Clamp only round-off excursions; vector scaling above preserves angle. */
    output->duty_a = control_clampf(output->duty_a,
                                    min_duty, 1.0f - min_duty);
    output->duty_b = control_clampf(output->duty_b,
                                    min_duty, 1.0f - min_duty);
    output->duty_c = control_clampf(output->duty_c,
                                    min_duty, 1.0f - min_duty);
    output->applied_voltage_v.alpha = scaled_alpha;
    output->applied_voltage_v.beta = scaled_beta;
    output->voltage_scale = scale;
    output->saturated = scale < 0.999999f;
    return true;
}
