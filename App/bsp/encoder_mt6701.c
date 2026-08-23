#include "encoder_mt6701.h"

#include <stddef.h>

#define TWO_PI_F 6.2831853071795864769f

static encoder_mt6701_t *volatile active_dma_encoder;

/* CRC-6, polynomial x^6 + x + 1, over angle[13:0] followed by status[3:0]. */
static uint8_t crc6_mt6701(uint32_t payload18)
{
    uint8_t crc = 0U;
    int bit;

    for (bit = 17; bit >= 0; --bit) {
        const uint8_t feedback =
            (uint8_t)(((payload18 >> (uint32_t)bit) & 1U) ^ ((crc >> 5) & 1U));
        crc = (uint8_t)((crc << 1) & 0x3FU);
        if (feedback != 0U) {
            crc ^= 0x03U;
        }
    }
    return crc;
}

bool encoder_mt6701_init(encoder_mt6701_t *encoder,
                         SPI_HandleTypeDef *spi,
                         GPIO_TypeDef *chip_select_port,
                         uint16_t chip_select_pin)
{
    if ((encoder == NULL) || (spi == NULL) || (chip_select_port == NULL)) {
        return false;
    }
    encoder->spi = spi;
    encoder->chip_select_port = chip_select_port;
    encoder->chip_select_pin = chip_select_pin;
    encoder->timeout_ms = 1U;
    encoder->dma_start_cycles = 0U;
    encoder->dma_busy = false;
    encoder->dma_sample_ready = false;
    encoder->dma_tx[0] = 0U;
    encoder->dma_tx[1] = 0U;
    encoder->dma_tx[2] = 0U;
    HAL_GPIO_WritePin(chip_select_port, chip_select_pin, GPIO_PIN_SET);
    return true;
}

bool encoder_mt6701_parse_ssi(const uint8_t frame[3],
                              uint32_t now_us,
                              encoder_sample_t *sample)
{
    uint32_t packed;
    uint32_t payload;
    uint16_t angle;
    uint8_t status;
    uint8_t received_crc;
    uint32_t health = ENCODER_HEALTH_OK;

    if ((frame == NULL) || (sample == NULL)) {
        return false;
    }
    packed = ((uint32_t)frame[0] << 16) |
             ((uint32_t)frame[1] << 8) |
             (uint32_t)frame[2];
    payload = packed >> 6;
    angle = (uint16_t)(payload >> 4);
    status = (uint8_t)(payload & 0x0FU);
    received_crc = (uint8_t)(packed & 0x3FU);

    if (crc6_mt6701(payload) != received_crc) {
        health |= ENCODER_HEALTH_CRC;
    }
    switch (status & 0x03U) {
    case 1U:
        health |= ENCODER_HEALTH_MAGNET_STRONG;
        break;
    case 2U:
        health |= ENCODER_HEALTH_MAGNET_WEAK;
        break;
    case 3U:
        health |= ENCODER_HEALTH_STATUS_RESERVED;
        break;
    default:
        break;
    }
    if ((status & 0x08U) != 0U) {
        health |= ENCODER_HEALTH_OVERSPEED;
    }

    sample->raw_count = angle;
    sample->counts_per_turn = 16384U;
    sample->mechanical_angle_rad = TWO_PI_F * (float)angle / 16384.0f;
    sample->timestamp_us = now_us;
    sample->health_flags = health;
    sample->valid = health == ENCODER_HEALTH_OK;
    return sample->valid;
}

bool encoder_mt6701_read(encoder_mt6701_t *encoder,
                         uint32_t now_us,
                         encoder_sample_t *sample)
{
    uint8_t frame[3] = {0U, 0U, 0U};
    HAL_StatusTypeDef status;

    if ((encoder == NULL) || (encoder->spi == NULL) || (sample == NULL)) {
        return false;
    }
    HAL_GPIO_WritePin(encoder->chip_select_port,
                      encoder->chip_select_pin,
                      GPIO_PIN_RESET);
    status = HAL_SPI_Receive(encoder->spi, frame, 3U, encoder->timeout_ms);
    HAL_GPIO_WritePin(encoder->chip_select_port,
                      encoder->chip_select_pin,
                      GPIO_PIN_SET);
    if (status != HAL_OK) {
        sample->valid = false;
        sample->health_flags = ENCODER_HEALTH_BUS;
        sample->timestamp_us = now_us;
        return false;
    }
    return encoder_mt6701_parse_ssi(frame, now_us, sample);
}

bool encoder_mt6701_start_dma(encoder_mt6701_t *encoder)
{
    if ((encoder == NULL) || (encoder->spi == NULL) || encoder->dma_busy ||
        (active_dma_encoder != NULL)) {
        return false;
    }
    active_dma_encoder = encoder;
    encoder->dma_busy = true;
    encoder->dma_start_cycles = bsp_cycle_counter();
    HAL_GPIO_WritePin(encoder->chip_select_port,
                      encoder->chip_select_pin,
                      GPIO_PIN_RESET);
    if (HAL_SPI_TransmitReceive_DMA(encoder->spi,
                                    encoder->dma_tx,
                                    encoder->dma_rx,
                                    3U) != HAL_OK) {
        HAL_GPIO_WritePin(encoder->chip_select_port,
                          encoder->chip_select_pin,
                          GPIO_PIN_SET);
        encoder->dma_busy = false;
        active_dma_encoder = NULL;
        return false;
    }
    return true;
}

bool encoder_mt6701_take_dma_sample(encoder_mt6701_t *encoder,
                                    encoder_sample_t *sample)
{
    if ((encoder == NULL) || (sample == NULL) ||
        !encoder->dma_sample_ready) {
        return false;
    }
    *sample = encoder->dma_sample;
    encoder->dma_sample_ready = false;
    return true;
}

bool encoder_mt6701_dma_timed_out(const encoder_mt6701_t *encoder,
                                  uint32_t now_cycles,
                                  uint32_t timeout_cycles)
{
    return (encoder != NULL) && (timeout_cycles != 0U) &&
           encoder->dma_busy &&
           ((uint32_t)(now_cycles - encoder->dma_start_cycles) >
            timeout_cycles);
}

bool encoder_mt6701_recover_dma_timeout(encoder_mt6701_t *encoder,
                                        uint32_t now_cycles,
                                        uint32_t timeout_cycles)
{
    uint32_t primask;
    bool claimed = false;

    if ((encoder == NULL) || (encoder->spi == NULL) ||
        (timeout_cycles == 0U)) {
        return false;
    }

    /* Claim the timed-out transfer atomically.  dma_busy intentionally stays
     * set until HAL_SPI_Abort has made the peripheral and DMA channel inert,
     * preventing the 20 kHz ISR from starting a replacement underneath us. */
    primask = __get_PRIMASK();
    __disable_irq();
    if (encoder->dma_busy && (active_dma_encoder == encoder) &&
        ((uint32_t)(now_cycles - encoder->dma_start_cycles) >
         timeout_cycles)) {
        active_dma_encoder = NULL;
        claimed = true;
    }
    __set_PRIMASK(primask);
    if (!claimed) {
        return false;
    }

    (void)HAL_SPI_Abort(encoder->spi);
    HAL_GPIO_WritePin(encoder->chip_select_port,
                      encoder->chip_select_pin,
                      GPIO_PIN_SET);
    encoder->dma_sample.raw_count = 0U;
    encoder->dma_sample.counts_per_turn = 16384U;
    encoder->dma_sample.mechanical_angle_rad = 0.0f;
    encoder->dma_sample.timestamp_us = bsp_micros();
    encoder->dma_sample.health_flags = ENCODER_HEALTH_BUS;
    encoder->dma_sample.valid = false;
    __DMB();
    encoder->dma_sample_ready = true;
    __DMB();
    encoder->dma_busy = false;
    return true;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *spi)
{
    encoder_mt6701_t *encoder = active_dma_encoder;

    if ((encoder == NULL) || (spi != encoder->spi)) {
        return;
    }
    HAL_GPIO_WritePin(encoder->chip_select_port,
                      encoder->chip_select_pin,
                      GPIO_PIN_SET);
    (void)encoder_mt6701_parse_ssi(encoder->dma_rx,
                                  bsp_micros(),
                                  &encoder->dma_sample);
    /* Publish the completed sample only after every field is globally
     * visible to the higher-priority ADC/current-loop interrupt. */
    __DMB();
    encoder->dma_sample_ready = true;
    active_dma_encoder = NULL;
    encoder->dma_busy = false;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *spi)
{
    encoder_mt6701_t *encoder = active_dma_encoder;

    if ((encoder == NULL) || (spi != encoder->spi)) {
        return;
    }
    HAL_GPIO_WritePin(encoder->chip_select_port,
                      encoder->chip_select_pin,
                      GPIO_PIN_SET);
    encoder->dma_sample.valid = false;
    encoder->dma_sample.health_flags = ENCODER_HEALTH_BUS;
    encoder->dma_sample.timestamp_us = bsp_micros();
    __DMB();
    encoder->dma_sample_ready = true;
    active_dma_encoder = NULL;
    encoder->dma_busy = false;
}
