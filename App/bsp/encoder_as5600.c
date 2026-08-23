#include "encoder_as5600.h"

#include <stddef.h>

#define AS5600_ADDRESS (0x36U << 1)
#define AS5600_REG_STATUS 0x0BU
#define AS5600_REG_RAW_ANGLE 0x0CU
#define AS5600_REG_CONF_H 0x07U

#define AS5600_STATUS_MAGNET_HIGH (1U << 3)
#define AS5600_STATUS_MAGNET_LOW  (1U << 4)
#define AS5600_STATUS_MAGNET_OK   (1U << 5)

#define TWO_PI_F 6.2831853071795864769f

static bool read_registers(encoder_as5600_t *encoder,
                           uint8_t reg,
                           uint8_t *data,
                           uint16_t length)
{
    return HAL_I2C_Mem_Read(encoder->i2c,
                            AS5600_ADDRESS,
                            reg,
                            I2C_MEMADD_SIZE_8BIT,
                            data,
                            length,
                            encoder->timeout_ms) == HAL_OK;
}

bool encoder_as5600_init(encoder_as5600_t *encoder,
                         I2C_HandleTypeDef *i2c,
                         bool configure_fast_filter)
{
    uint8_t configuration[2];
    uint16_t conf;

    if ((encoder == NULL) || (i2c == NULL)) {
        return false;
    }
    encoder->i2c = i2c;
    encoder->timeout_ms = 2U;

    if (!configure_fast_filter) {
        return true;
    }
    if (!read_registers(encoder, AS5600_REG_CONF_H, configuration, 2U)) {
        return false;
    }

    /* Volatile CONF only: SF=11 (fastest slow filter), FTH=001. No OTP burn. */
    conf = (uint16_t)(((uint16_t)configuration[0] << 8) | configuration[1]);
    conf &= (uint16_t)~((3U << 8) | (7U << 5));
    conf |= (uint16_t)((3U << 8) | (1U << 5));
    configuration[0] = (uint8_t)(conf >> 8);
    configuration[1] = (uint8_t)conf;
    return HAL_I2C_Mem_Write(encoder->i2c,
                             AS5600_ADDRESS,
                             AS5600_REG_CONF_H,
                             I2C_MEMADD_SIZE_8BIT,
                             configuration,
                             2U,
                             encoder->timeout_ms) == HAL_OK;
}

bool encoder_as5600_read(encoder_as5600_t *encoder,
                         uint32_t now_us,
                         encoder_sample_t *sample)
{
    uint8_t raw[2];
    uint8_t status;
    uint16_t count;
    uint32_t health = ENCODER_HEALTH_OK;

    if ((encoder == NULL) || (encoder->i2c == NULL) || (sample == NULL)) {
        return false;
    }
    if (!read_registers(encoder, AS5600_REG_RAW_ANGLE, raw, 2U) ||
        !read_registers(encoder, AS5600_REG_STATUS, &status, 1U)) {
        sample->valid = false;
        sample->health_flags = ENCODER_HEALTH_BUS;
        sample->timestamp_us = now_us;
        return false;
    }

    count = (uint16_t)((((uint16_t)raw[0] & 0x0FU) << 8) | raw[1]);
    if ((status & AS5600_STATUS_MAGNET_OK) == 0U) {
        health |= ENCODER_HEALTH_NO_MAGNET;
    }
    if ((status & AS5600_STATUS_MAGNET_LOW) != 0U) {
        health |= ENCODER_HEALTH_MAGNET_WEAK;
    }
    if ((status & AS5600_STATUS_MAGNET_HIGH) != 0U) {
        health |= ENCODER_HEALTH_MAGNET_STRONG;
    }

    sample->raw_count = count;
    sample->counts_per_turn = 4096U;
    sample->mechanical_angle_rad = TWO_PI_F * (float)count / 4096.0f;
    sample->timestamp_us = now_us;
    sample->health_flags = health;
    sample->valid = health == ENCODER_HEALTH_OK;
    return sample->valid;
}
