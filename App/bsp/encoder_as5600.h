#ifndef BRUSHLESS_ENCODER_AS5600_H
#define BRUSHLESS_ENCODER_AS5600_H

#include <stdbool.h>

#include "bsp.h"
#include "encoder.h"

typedef struct {
    I2C_HandleTypeDef *i2c;
    uint32_t timeout_ms;
} encoder_as5600_t;

bool encoder_as5600_init(encoder_as5600_t *encoder,
                         I2C_HandleTypeDef *i2c,
                         bool configure_fast_filter);
bool encoder_as5600_read(encoder_as5600_t *encoder,
                         uint32_t now_us,
                         encoder_sample_t *sample);

#endif
