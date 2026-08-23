#ifndef APP_CONTROL_FOC_MATH_H
#define APP_CONTROL_FOC_MATH_H

/**
 * @file foc_math.h
 * @brief Hardware-independent single precision FOC transforms and modulation.
 *
 * All angular quantities are in radians, voltages are in volts and times are
 * in seconds.  The module has no hidden state and allocates no memory.
 */

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CONTROL_PI_F        (3.14159265358979323846f)
#define CONTROL_TWO_PI_F    (6.28318530717958647692f)
#define CONTROL_INV_SQRT3_F (0.57735026918962576451f)
#define CONTROL_SQRT3_BY_2_F (0.86602540378443864676f)

typedef struct {
    float a;
    float b;
    float c;
} control_abc_t;

typedef struct {
    float alpha;
    float beta;
} control_alphabeta_t;

typedef struct {
    float d;
    float q;
} control_dq_t;

/** Configuration for centered, linear-region SVPWM. */
typedef struct {
    float dc_bus_voltage_v;
    float pwm_period_s;
    /**
     * Minimum allowed high or low pulse duration.  A value of zero disables
     * pulse suppression. It must be less than half pwm_period_s.
     */
    float minimum_pulse_s;
} control_svpwm_config_t;

typedef struct {
    float duty_a;
    float duty_b;
    float duty_c;
    control_alphabeta_t applied_voltage_v;
    /** 1.0 when the requested vector fitted; otherwise the applied scale. */
    float voltage_scale;
    bool saturated;
} control_svpwm_output_t;

/** Clamp a finite value. Invalid bounds or a NaN value return lower_bound. */
float control_clampf(float value, float lower_bound, float upper_bound);

/** Wrap an angle into [-pi, pi). Non-finite input produces 0 rad. */
float control_angle_wrap_pm_pi(float angle_rad);

/** Wrap an angle into [0, 2*pi). Non-finite input produces 0 rad. */
float control_angle_wrap_0_2pi(float angle_rad);

/** Amplitude-invariant, three-input Clarke transform. */
bool control_clarke(const control_abc_t *abc, control_alphabeta_t *alphabeta);

/** Inverse of control_clarke for a zero-sequence-free three-phase system. */
bool control_inverse_clarke(const control_alphabeta_t *alphabeta,
                            control_abc_t *abc);

/** Orthogonal alpha/beta rotation using the supplied electrical angle. */
bool control_park(const control_alphabeta_t *alphabeta,
                  float electrical_angle_rad,
                  control_dq_t *dq);

/** Inverse of control_park. */
bool control_inverse_park(const control_dq_t *dq,
                          float electrical_angle_rad,
                          control_alphabeta_t *alphabeta);

/**
 * Limit a d/q vector to a voltage circle without changing its direction.
 * max_magnitude_v may be zero. On invalid input, limited is set to zero and
 * the function returns false.
 */
bool control_limit_dq_voltage(const control_dq_t *requested,
                              float max_magnitude_v,
                              control_dq_t *limited,
                              float *applied_scale);

/**
 * Generate centered SVPWM duties and enforce minimum on/off pulse duration.
 *
 * The input voltage is the requested stator alpha/beta vector. If the vector
 * does not fit in the available hexagon after reserving the minimum pulses,
 * it is scaled radially. Duties are always constrained to
 * [minimum_pulse_s / pwm_period_s, 1 - that value]. On an invalid request the
 * output is placed in the safe neutral state (all duties 0.5) and false is
 * returned.
 */
bool control_svpwm(const control_alphabeta_t *requested_voltage_v,
                    const control_svpwm_config_t *config,
                    control_svpwm_output_t *output);

#ifdef __cplusplus
}
#endif

#endif /* APP_CONTROL_FOC_MATH_H */
