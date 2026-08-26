#include "encoder_as5600.h"

#include <stddef.h>

#include "board_config.h"
#include "encoder_plausibility.h"

#define AS5600_ADDRESS_7BIT 0x36U
#define AS5600_ADDRESS (AS5600_ADDRESS_7BIT << 1)
#define AS5600_REG_STATUS 0x0BU
#define AS5600_REG_CONF_H 0x07U
#define AS5600_REG_AGC 0x1AU

#define AS5600_STATUS_MAGNET_HIGH (1U << 3)
#define AS5600_STATUS_MAGNET_LOW  (1U << 4)
#define AS5600_STATUS_MAGNET_OK   (1U << 5)
#define AS5600_CONF_SLOW_FILTER_MASK (3U << 8)
#define AS5600_CONF_FAST_THRESHOLD_MASK (7U << 10)
#define AS5600_CONF_SLOW_FILTER_FASTEST (3U << 8)
#define AS5600_CONF_FAST_THRESHOLD_6LSB (1U << 10)

#define AS5600_GPIO_CLOCK_HIGH_TIMEOUT_CYCLES \
    (BOARD_SYSCLK_HZ / 10000UL)
#define AS5600_GPIO_READ_ATTEMPTS 3U

#if (BOARD_LAB_AS5600_GPIO_I2C_HZ == 0UL) || \
    (BOARD_LAB_AS5600_GPIO_I2C_HZ > 1000000UL)
#error "BOARD_LAB_AS5600_GPIO_I2C_HZ must be in the AS5600 supported range"
#endif

#define TWO_PI_F 6.2831853071795864769f
#define AS5600_COUNTS_PER_TURN 4096U

#if (BOARD_LAB_AS5600_STEP_MARGIN_COUNTS >= 2048U) || \
    (BOARD_LAB_AS5600_STARTUP_MATCH_COUNTS >= 2048U)
#error "AS5600 plausibility margins must be less than half a turn"
#endif

typedef struct {
    uint16_t count;
    uint32_t health;
} as5600_angle_frame_t;

static void gpio_delay(const encoder_as5600_t *encoder)
{
    const uint32_t start = DWT->CYCCNT;

    while ((uint32_t)(DWT->CYCCNT - start) <
           encoder->gpio_half_period_cycles) {
        __NOP();
    }
}

static void gpio_scl_low(const encoder_as5600_t *encoder)
{
    encoder->scl_port->BSRR = (uint32_t)encoder->scl_pin << 16;
}

static void gpio_scl_release(const encoder_as5600_t *encoder)
{
    encoder->scl_port->BSRR = encoder->scl_pin;
}

static void gpio_sda_low(const encoder_as5600_t *encoder)
{
    encoder->sda_port->BSRR = (uint32_t)encoder->sda_pin << 16;
}

static void gpio_sda_release(const encoder_as5600_t *encoder)
{
    encoder->sda_port->BSRR = encoder->sda_pin;
}

static bool gpio_scl_is_high(const encoder_as5600_t *encoder)
{
    return (encoder->scl_port->IDR & encoder->scl_pin) != 0U;
}

static bool gpio_sda_is_high(const encoder_as5600_t *encoder)
{
    return (encoder->sda_port->IDR & encoder->sda_pin) != 0U;
}

static bool gpio_sample_sda_majority(const encoder_as5600_t *encoder)
{
    const uint32_t start = DWT->CYCCNT;
    uint32_t spacing = encoder->gpio_half_period_cycles / 4U;
    uint32_t high_samples = 0U;
    uint32_t sample_index;

    if (spacing == 0U) {
        spacing = 1U;
    }
    for (sample_index = 1U; sample_index <= 3U; sample_index++) {
        const uint32_t target_cycles = spacing * sample_index;

        while ((uint32_t)(DWT->CYCCNT - start) < target_cycles) {
            __NOP();
        }
        if (gpio_sda_is_high(encoder)) {
            ++high_samples;
        }
    }
    while ((uint32_t)(DWT->CYCCNT - start) <
           encoder->gpio_half_period_cycles) {
        __NOP();
    }
    return high_samples >= 2U;
}

static bool gpio_wait_scl_high(const encoder_as5600_t *encoder)
{
    const uint32_t start = DWT->CYCCNT;

    gpio_scl_release(encoder);
    while (!gpio_scl_is_high(encoder)) {
        if ((uint32_t)(DWT->CYCCNT - start) >=
            AS5600_GPIO_CLOCK_HIGH_TIMEOUT_CYCLES) {
            return false;
        }
    }
    return true;
}

static bool gpio_stop(encoder_as5600_t *encoder)
{
    bool clock_released;
    bool data_released;

    gpio_sda_low(encoder);
    gpio_delay(encoder);
    clock_released = gpio_wait_scl_high(encoder);
    gpio_delay(encoder);
    gpio_sda_release(encoder);
    data_released = gpio_sample_sda_majority(encoder);
    return clock_released && data_released;
}

static bool gpio_bus_recover(encoder_as5600_t *encoder)
{
    uint32_t pulse;

    gpio_sda_release(encoder);
    /* The debugger can reset the MCU at any bit position. Nine clocks are
     * sufficient only when the slave is known to be byte-aligned; use up to
     * four bytes so a retained AS5600 transmit state can reach a released
     * ACK slot from an unknown phase. */
    for (pulse = 0U; pulse < 32U; pulse++) {
        gpio_scl_low(encoder);
        gpio_delay(encoder);
        if (!gpio_wait_scl_high(encoder)) {
            gpio_sda_release(encoder);
            return false;
        }
        if (gpio_sample_sda_majority(encoder)) {
            return gpio_stop(encoder) &&
                   gpio_scl_is_high(encoder) &&
                   gpio_sda_is_high(encoder);
        }
    }
    if (!gpio_stop(encoder)) {
        return false;
    }
    return gpio_scl_is_high(encoder) && gpio_sda_is_high(encoder);
}

static bool gpio_start(encoder_as5600_t *encoder)
{
    gpio_sda_release(encoder);
    if (!gpio_wait_scl_high(encoder) ||
        !gpio_sample_sda_majority(encoder)) {
        return false;
    }
    gpio_sda_low(encoder);
    gpio_delay(encoder);
    gpio_scl_low(encoder);
    gpio_delay(encoder);
    return true;
}

static bool gpio_write_byte(encoder_as5600_t *encoder, uint8_t value)
{
    uint8_t mask;
    bool acknowledged;

    for (mask = 0x80U; mask != 0U; mask >>= 1U) {
        if ((value & mask) != 0U) {
            gpio_sda_release(encoder);
        } else {
            gpio_sda_low(encoder);
        }
        gpio_delay(encoder);
        if (!gpio_wait_scl_high(encoder)) {
            return false;
        }
        gpio_delay(encoder);
        gpio_scl_low(encoder);
        gpio_delay(encoder);
    }

    gpio_sda_release(encoder);
    gpio_delay(encoder);
    if (!gpio_wait_scl_high(encoder)) {
        return false;
    }
    acknowledged = !gpio_sample_sda_majority(encoder);
    gpio_scl_low(encoder);
    gpio_delay(encoder);
    return acknowledged;
}

static bool gpio_read_byte(encoder_as5600_t *encoder,
                           uint8_t *value,
                           bool acknowledge)
{
    uint32_t bit;
    uint8_t received = 0U;

    gpio_sda_release(encoder);
    for (bit = 0U; bit < 8U; bit++) {
        if (!gpio_wait_scl_high(encoder)) {
            return false;
        }
        received = (uint8_t)(received << 1U);
        if (gpio_sample_sda_majority(encoder)) {
            received |= 1U;
        }
        gpio_scl_low(encoder);
        gpio_delay(encoder);
    }

    if (acknowledge) {
        gpio_sda_low(encoder);
    } else {
        gpio_sda_release(encoder);
    }
    gpio_delay(encoder);
    if (!gpio_wait_scl_high(encoder)) {
        gpio_sda_release(encoder);
        return false;
    }
    (void)gpio_sample_sda_majority(encoder);
    gpio_scl_low(encoder);
    gpio_sda_release(encoder);
    gpio_delay(encoder);
    *value = received;
    return true;
}

static bool gpio_device_ready(encoder_as5600_t *encoder)
{
    bool ready;

    if (!gpio_bus_recover(encoder) || !gpio_start(encoder)) {
        return false;
    }
    ready = gpio_write_byte(encoder, (uint8_t)AS5600_ADDRESS);
    (void)gpio_stop(encoder);
    return ready;
}

static bool gpio_read_registers(encoder_as5600_t *encoder,
                                uint8_t reg,
                                uint8_t *data,
                                uint16_t length)
{
    uint16_t index;
    bool success = (data != NULL) && (length > 0U);

    if (success) {
        success = gpio_start(encoder) &&
                  gpio_write_byte(encoder, (uint8_t)AS5600_ADDRESS) &&
                  gpio_write_byte(encoder, reg) &&
                  gpio_start(encoder) &&
                  gpio_write_byte(encoder,
                                  (uint8_t)(AS5600_ADDRESS | 1U));
    }
    for (index = 0U; success && (index < length); index++) {
        success = gpio_read_byte(encoder,
                                 &data[index],
                                 index + 1U < length);
    }
    if (!gpio_stop(encoder)) {
        success = false;
    }
    return success;
}

static bool gpio_write_registers(encoder_as5600_t *encoder,
                                 uint8_t reg,
                                 const uint8_t *data,
                                 uint16_t length)
{
    uint16_t index;
    bool success = (data != NULL) && (length > 0U);

    if (success) {
        success = gpio_start(encoder) &&
                  gpio_write_byte(encoder, (uint8_t)AS5600_ADDRESS) &&
                  gpio_write_byte(encoder, reg);
    }
    for (index = 0U; success && (index < length); index++) {
        success = gpio_write_byte(encoder, data[index]);
    }
    if (!gpio_stop(encoder)) {
        success = false;
    }
    return success;
}

static bool read_registers(encoder_as5600_t *encoder,
                           uint8_t reg,
                           uint8_t *data,
                           uint16_t length)
{
    if (encoder->gpio_backend) {
        return gpio_read_registers(encoder, reg, data, length);
    }
    return HAL_I2C_Mem_Read(encoder->i2c,
                            AS5600_ADDRESS,
                            reg,
                            I2C_MEMADD_SIZE_8BIT,
                            data,
                            length,
                            encoder->timeout_ms) == HAL_OK;
}

static bool read_registers_recoverable(encoder_as5600_t *encoder,
                                       uint8_t reg,
                                       uint8_t *data,
                                       uint16_t length)
{
    uint32_t attempt;

    if (read_registers(encoder, reg, data, length)) {
        return true;
    }

    ++encoder->diagnostics.bus_error_count;
    if (encoder->gpio_backend) {
        /* At 200 kHz the bounded third attempt still fits comfortably under
         * the independent 8 ms stale watchdog. It handles a short PWM-noise
         * burst without weakening the gate interlock or publishing old data
         * as fresh. */
        for (attempt = 1U; attempt < AS5600_GPIO_READ_ATTEMPTS; attempt++) {
            if (gpio_bus_recover(encoder) &&
                read_registers(encoder, reg, data, length)) {
                ++encoder->diagnostics.bus_recovery_count;
                return true;
            }
        }
    }

    ++encoder->diagnostics.bus_failure_count;
    return false;
}

static bool read_angle_frame(encoder_as5600_t *encoder,
                             as5600_angle_frame_t *frame)
{
    uint8_t bytes[3];
    uint32_t health = ENCODER_HEALTH_OK;

    if ((encoder == NULL) || (frame == NULL) ||
        !read_registers_recoverable(encoder,
                                    AS5600_REG_STATUS,
                                    bytes,
                                    3U)) {
        return false;
    }

    encoder->diagnostics.status_raw = bytes[0];
    ++encoder->diagnostics.angle_read_count;
    frame->count = (uint16_t)((((uint16_t)bytes[1] & 0x0FU) << 8) |
                              bytes[2]);
    if ((bytes[0] & AS5600_STATUS_MAGNET_OK) == 0U) {
        health |= ENCODER_HEALTH_NO_MAGNET;
    }
    if ((bytes[0] & AS5600_STATUS_MAGNET_LOW) != 0U) {
        health |= ENCODER_HEALTH_MAGNET_WEAK;
    }
    if ((bytes[0] & AS5600_STATUS_MAGNET_HIGH) != 0U) {
        health |= ENCODER_HEALTH_MAGNET_STRONG;
    }
    frame->health = health;
    return true;
}

static bool angle_frame_trackable(const as5600_angle_frame_t *frame)
{
    return (frame != NULL) &&
           ((frame->health & ~ENCODER_HEALTH_MAGNET_WEAK) == 0U);
}

static bool publish_angle_sample(encoder_as5600_t *encoder,
                                 const as5600_angle_frame_t *frame,
                                 uint32_t now_us,
                                 bool update_history,
                                 encoder_sample_t *sample)
{
    sample->raw_count = frame->count;
    sample->counts_per_turn = AS5600_COUNTS_PER_TURN;
    sample->mechanical_angle_rad =
        TWO_PI_F * (float)frame->count / (float)AS5600_COUNTS_PER_TURN;
    sample->timestamp_us = now_us;
    sample->health_flags = frame->health;
    sample->valid = frame->health == ENCODER_HEALTH_OK;

    if (update_history) {
        encoder->last_accepted_count = frame->count;
        encoder->last_accepted_timestamp_us = now_us;
        encoder->angle_history_valid = true;
    }
    return sample->valid;
}

static void publish_bus_failure(uint32_t now_us, encoder_sample_t *sample)
{
    sample->mechanical_angle_rad = 0.0f;
    sample->raw_count = 0U;
    sample->counts_per_turn = AS5600_COUNTS_PER_TURN;
    sample->timestamp_us = now_us;
    sample->health_flags = ENCODER_HEALTH_BUS;
    sample->valid = false;
}

static bool write_registers(encoder_as5600_t *encoder,
                            uint8_t reg,
                            const uint8_t *data,
                            uint16_t length)
{
    if (encoder->gpio_backend) {
        return gpio_write_registers(encoder, reg, data, length);
    }
    return HAL_I2C_Mem_Write(encoder->i2c,
                             AS5600_ADDRESS,
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             (uint8_t *)data,
                             length,
                             encoder->timeout_ms) == HAL_OK;
}

static bool configure_encoder(encoder_as5600_t *encoder,
                              bool configure_fast_filter)
{
    uint8_t configuration[2];
    uint16_t conf;
    uint16_t verified_conf;

    if (!configure_fast_filter) {
        return true;
    }
    if (!read_registers(encoder, AS5600_REG_CONF_H, configuration, 2U)) {
        return false;
    }

    /* Volatile CONF only: SF=11 (fastest slow filter), FTH=001 (6 LSB).
     * FTH is CONF[12:10], not the PWM/OUTS field at CONF[7:5]. No OTP burn. */
    conf = (uint16_t)(((uint16_t)configuration[0] << 8) | configuration[1]);
    conf &= (uint16_t)~(AS5600_CONF_SLOW_FILTER_MASK |
                        AS5600_CONF_FAST_THRESHOLD_MASK);
    conf |= (uint16_t)(AS5600_CONF_SLOW_FILTER_FASTEST |
                       AS5600_CONF_FAST_THRESHOLD_6LSB);
    configuration[0] = (uint8_t)(conf >> 8);
    configuration[1] = (uint8_t)conf;
    if (!write_registers(encoder,
                         AS5600_REG_CONF_H,
                         configuration,
                         2U) ||
        !read_registers(encoder,
                        AS5600_REG_CONF_H,
                        configuration,
                        2U)) {
        return false;
    }
    verified_conf = (uint16_t)(((uint16_t)configuration[0] << 8) |
                               configuration[1]);
    return (verified_conf &
            (AS5600_CONF_SLOW_FILTER_MASK |
             AS5600_CONF_FAST_THRESHOLD_MASK)) ==
           (AS5600_CONF_SLOW_FILTER_FASTEST |
            AS5600_CONF_FAST_THRESHOLD_6LSB);
}

bool encoder_as5600_init(encoder_as5600_t *encoder,
                         I2C_HandleTypeDef *i2c,
                         bool configure_fast_filter)
{
    if ((encoder == NULL) || (i2c == NULL)) {
        return false;
    }
    encoder->i2c = i2c;
    encoder->timeout_ms = 5U;
    encoder->gpio_backend = false;
    encoder->last_accepted_count = 0U;
    encoder->last_accepted_timestamp_us = 0U;
    encoder->angle_history_valid = false;
    encoder->diagnostics = (encoder_as5600_diagnostics_t){0};

    if (HAL_I2C_IsDeviceReady(encoder->i2c,
                              AS5600_ADDRESS,
                              2U,
                              encoder->timeout_ms) != HAL_OK) {
        return false;
    }
    if (!configure_encoder(encoder, configure_fast_filter)) {
        return false;
    }
    (void)encoder_as5600_read_diagnostics(encoder);
    return true;
}

bool encoder_as5600_init_gpio(encoder_as5600_t *encoder,
                              I2C_HandleTypeDef *i2c_to_disable,
                              GPIO_TypeDef *scl_port,
                              uint16_t scl_pin,
                              GPIO_TypeDef *sda_port,
                              uint16_t sda_pin,
                              bool configure_fast_filter)
{
    GPIO_InitTypeDef gpio = {0};

    if ((encoder == NULL) || (scl_port == NULL) || (sda_port == NULL) ||
        (scl_pin == 0U) || (sda_pin == 0U)) {
        return false;
    }
    if ((i2c_to_disable != NULL) && (i2c_to_disable->Instance != NULL)) {
        __HAL_I2C_DISABLE(i2c_to_disable);
    }

    encoder->i2c = i2c_to_disable;
    encoder->timeout_ms = 5U;
    encoder->scl_port = scl_port;
    encoder->sda_port = sda_port;
    encoder->scl_pin = scl_pin;
    encoder->sda_pin = sda_pin;
    encoder->gpio_half_period_cycles =
        BOARD_SYSCLK_HZ / (2UL * BOARD_LAB_AS5600_GPIO_I2C_HZ);
    encoder->gpio_backend = true;
    encoder->last_accepted_count = 0U;
    encoder->last_accepted_timestamp_us = 0U;
    encoder->angle_history_valid = false;
    encoder->diagnostics = (encoder_as5600_diagnostics_t){0};

    gpio_scl_release(encoder);
    gpio_sda_release(encoder);
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    if (scl_port == sda_port) {
        gpio.Pin = scl_pin | sda_pin;
        HAL_GPIO_Init(scl_port, &gpio);
    } else {
        gpio.Pin = scl_pin;
        HAL_GPIO_Init(scl_port, &gpio);
        gpio.Pin = sda_pin;
        HAL_GPIO_Init(sda_port, &gpio);
    }

    if (!gpio_device_ready(encoder)) {
        gpio_scl_release(encoder);
        gpio_sda_release(encoder);
        return false;
    }
    if (!configure_encoder(encoder, configure_fast_filter)) {
        return false;
    }
    (void)encoder_as5600_read_diagnostics(encoder);
    return true;
}

bool encoder_as5600_read(encoder_as5600_t *encoder,
                         uint32_t now_us,
                         encoder_sample_t *sample)
{
    as5600_angle_frame_t first_frame = {0};
    as5600_angle_frame_t second_frame = {0};
    as5600_angle_frame_t third_frame = {0};
    uint32_t elapsed_us;

    if ((encoder == NULL) || (sample == NULL) ||
        (!encoder->gpio_backend && (encoder->i2c == NULL))) {
        return false;
    }
    /* STATUS, RAW_ANGLE_H and RAW_ANGLE_L are contiguous. One burst avoids a
     * second address phase and halves the opportunity for a blocking error. */
    if (!read_angle_frame(encoder, &first_frame)) {
        publish_bus_failure(now_us, sample);
        return false;
    }
    if (!angle_frame_trackable(&first_frame)) {
        return publish_angle_sample(encoder,
                                    &first_frame,
                                    now_us,
                                    false,
                                    sample);
    }

    if (!encoder->angle_history_valid) {
        /* At acquisition start there is no trusted predecessor. Require two
         * nearby frames; if they disagree, a third frame selects the pair
         * that agrees. This prevents a single startup 0xFFF frame from
         * becoming the plausibility reference for the whole run. */
        if (!read_angle_frame(encoder, &second_frame)) {
            publish_bus_failure(now_us, sample);
            return false;
        }
        if (!angle_frame_trackable(&second_frame)) {
            return publish_angle_sample(encoder,
                                        &second_frame,
                                        now_us,
                                        false,
                                        sample);
        }
        if (encoder_plausibility_step(
                first_frame.count,
                second_frame.count,
                AS5600_COUNTS_PER_TURN,
                0U,
                0.0f,
                BOARD_LAB_AS5600_STARTUP_MATCH_COUNTS)) {
            return publish_angle_sample(encoder,
                                        &second_frame,
                                        now_us,
                                        true,
                                        sample);
        }

        ++encoder->diagnostics.angle_glitch_count;
        if (!read_angle_frame(encoder, &third_frame)) {
            publish_bus_failure(now_us, sample);
            return false;
        }
        if (!angle_frame_trackable(&third_frame)) {
            third_frame.health |= ENCODER_HEALTH_OVERSPEED;
            return publish_angle_sample(encoder,
                                        &third_frame,
                                        now_us,
                                        false,
                                        sample);
        }
        if (encoder_plausibility_step(
                first_frame.count,
                third_frame.count,
                AS5600_COUNTS_PER_TURN,
                0U,
                0.0f,
                BOARD_LAB_AS5600_STARTUP_MATCH_COUNTS) ||
            encoder_plausibility_step(
                second_frame.count,
                third_frame.count,
                AS5600_COUNTS_PER_TURN,
                0U,
                0.0f,
                BOARD_LAB_AS5600_STARTUP_MATCH_COUNTS)) {
            ++encoder->diagnostics.angle_glitch_recovery_count;
            return publish_angle_sample(encoder,
                                        &third_frame,
                                        now_us,
                                        true,
                                        sample);
        }
        third_frame.health |= ENCODER_HEALTH_OVERSPEED;
        return publish_angle_sample(encoder,
                                    &third_frame,
                                    now_us,
                                    false,
                                    sample);
    }

    elapsed_us = now_us - encoder->last_accepted_timestamp_us;
    if (encoder_plausibility_step(
            encoder->last_accepted_count,
            first_frame.count,
            AS5600_COUNTS_PER_TURN,
            elapsed_us,
            BOARD_LAB_AS5600_MAX_STEP_SPEED_RAD_S,
            BOARD_LAB_AS5600_STEP_MARGIN_COUNTS)) {
        return publish_angle_sample(encoder,
                                    &first_frame,
                                    now_us,
                                    true,
                                    sample);
    }

    /* A gross count jump is not immediately trusted even if STATUS says the
     * magnet is present. Re-read in the same poll and accept only a frame that
     * agrees with the last trusted count. One corrupted frame is therefore
     * invisible to the tracker/FOC; persistent corruption fails closed. */
    ++encoder->diagnostics.angle_glitch_count;
    if (!read_angle_frame(encoder, &second_frame)) {
        publish_bus_failure(now_us, sample);
        return false;
    }
    if (angle_frame_trackable(&second_frame) &&
        encoder_plausibility_step(
            encoder->last_accepted_count,
            second_frame.count,
            AS5600_COUNTS_PER_TURN,
            elapsed_us,
            BOARD_LAB_AS5600_MAX_STEP_SPEED_RAD_S,
            BOARD_LAB_AS5600_STEP_MARGIN_COUNTS)) {
        ++encoder->diagnostics.angle_glitch_recovery_count;
        return publish_angle_sample(encoder,
                                    &second_frame,
                                    now_us,
                                    true,
                                    sample);
    }

    second_frame.health |= ENCODER_HEALTH_OVERSPEED;
    return publish_angle_sample(encoder,
                                &second_frame,
                                now_us,
                                false,
                                sample);
}

bool encoder_as5600_read_diagnostics(encoder_as5600_t *encoder)
{
    uint8_t frame[3];
    uint32_t agc;
    uint32_t magnitude;

    if ((encoder == NULL) ||
        (!encoder->gpio_backend && (encoder->i2c == NULL))) {
        return false;
    }
    /* AGC (0x1A), MAGNITUDE_H (0x1B) and MAGNITUDE_L (0x1C) are
     * contiguous. They are diagnostic-only: failure must not discard an
     * otherwise fresh angle sample used by the control interlock. */
    if (!read_registers_recoverable(encoder, AS5600_REG_AGC, frame, 3U)) {
        ++encoder->diagnostics.diagnostic_failure_count;
        return false;
    }

    agc = frame[0];
    magnitude = (((uint32_t)frame[1] & 0x0FU) << 8) | frame[2];
    encoder->diagnostics.agc = agc;
    encoder->diagnostics.magnitude = magnitude;
    if (!encoder->diagnostics.valid) {
        encoder->diagnostics.minimum_agc = agc;
        encoder->diagnostics.maximum_agc = agc;
        encoder->diagnostics.minimum_magnitude = magnitude;
        encoder->diagnostics.maximum_magnitude = magnitude;
        encoder->diagnostics.valid = true;
    } else {
        if (agc < encoder->diagnostics.minimum_agc) {
            encoder->diagnostics.minimum_agc = agc;
        }
        if (agc > encoder->diagnostics.maximum_agc) {
            encoder->diagnostics.maximum_agc = agc;
        }
        if (magnitude < encoder->diagnostics.minimum_magnitude) {
            encoder->diagnostics.minimum_magnitude = magnitude;
        }
        if (magnitude > encoder->diagnostics.maximum_magnitude) {
            encoder->diagnostics.maximum_magnitude = magnitude;
        }
    }
    ++encoder->diagnostics.diagnostic_read_count;
    return true;
}

void encoder_as5600_get_diagnostics(
    const encoder_as5600_t *encoder,
    encoder_as5600_diagnostics_t *diagnostics)
{
    if (diagnostics == NULL) {
        return;
    }
    if (encoder == NULL) {
        *diagnostics = (encoder_as5600_diagnostics_t){0};
        return;
    }
    *diagnostics = encoder->diagnostics;
}

void encoder_as5600_release_gpio_bus(encoder_as5600_t *encoder)
{
    if ((encoder == NULL) || !encoder->gpio_backend) {
        return;
    }
    gpio_scl_release(encoder);
    gpio_sda_release(encoder);
    encoder->angle_history_valid = false;
}

uint32_t encoder_as5600_gpio_levels(const encoder_as5600_t *encoder)
{
    uint32_t levels = 0U;

    if ((encoder == NULL) || !encoder->gpio_backend) {
        return levels;
    }
    if (gpio_scl_is_high(encoder)) {
        levels |= ENCODER_AS5600_GPIO_LEVEL_SCL_HIGH;
    }
    if (gpio_sda_is_high(encoder)) {
        levels |= ENCODER_AS5600_GPIO_LEVEL_SDA_HIGH;
    }
    return levels;
}
