#ifndef BRUSHLESS_ENCODER_AS5600_H
#define BRUSHLESS_ENCODER_AS5600_H

#include <stdbool.h>

#include "bsp.h"
#include "encoder.h"

typedef struct {
    uint32_t status_raw;
    uint32_t agc;
    uint32_t magnitude;
    uint32_t minimum_agc;
    uint32_t maximum_agc;
    uint32_t minimum_magnitude;
    uint32_t maximum_magnitude;
    uint32_t angle_read_count;
    uint32_t diagnostic_read_count;
    uint32_t bus_error_count;
    uint32_t bus_recovery_count;
    uint32_t bus_failure_count;
    uint32_t diagnostic_failure_count;
    uint32_t angle_glitch_count;
    uint32_t angle_glitch_recovery_count;
    bool valid;
} encoder_as5600_diagnostics_t;

typedef struct {
    I2C_HandleTypeDef *i2c;
    uint32_t timeout_ms;
    GPIO_TypeDef *scl_port;
    GPIO_TypeDef *sda_port;
    uint16_t scl_pin;
    uint16_t sda_pin;
    uint32_t gpio_half_period_cycles;
    bool gpio_backend;
    uint16_t last_accepted_count;
    uint32_t last_accepted_timestamp_us;
    bool angle_history_valid;
    encoder_as5600_diagnostics_t diagnostics;
} encoder_as5600_t;

bool encoder_as5600_init(encoder_as5600_t *encoder,
                         I2C_HandleTypeDef *i2c,
                         bool configure_fast_filter);
bool encoder_as5600_init_gpio(encoder_as5600_t *encoder,
                              I2C_HandleTypeDef *i2c_to_disable,
                              GPIO_TypeDef *scl_port,
                              uint16_t scl_pin,
                              GPIO_TypeDef *sda_port,
                              uint16_t sda_pin,
                              bool configure_fast_filter);
bool encoder_as5600_read(encoder_as5600_t *encoder,
                         uint32_t now_us,
                         encoder_sample_t *sample);
bool encoder_as5600_read_diagnostics(encoder_as5600_t *encoder);
void encoder_as5600_get_diagnostics(
    const encoder_as5600_t *encoder,
    encoder_as5600_diagnostics_t *diagnostics);
void encoder_as5600_release_gpio_bus(encoder_as5600_t *encoder);
uint32_t encoder_as5600_gpio_levels(const encoder_as5600_t *encoder);

enum {
    ENCODER_AS5600_GPIO_LEVEL_SCL_HIGH = UINT32_C(1) << 0,
    ENCODER_AS5600_GPIO_LEVEL_SDA_HIGH = UINT32_C(1) << 1
};

#endif
