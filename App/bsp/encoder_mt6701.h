#ifndef BRUSHLESS_ENCODER_MT6701_H
#define BRUSHLESS_ENCODER_MT6701_H

#include <stdbool.h>
#include <stdint.h>

#include "bsp.h"
#include "encoder.h"

typedef struct {
    SPI_HandleTypeDef *spi;
    GPIO_TypeDef *chip_select_port;
    uint16_t chip_select_pin;
    uint32_t timeout_ms;
    uint8_t dma_tx[3];
    uint8_t dma_rx[3];
    encoder_sample_t dma_sample;
    volatile uint32_t dma_start_cycles;
    volatile bool dma_busy;
    volatile bool dma_sample_ready;
} encoder_mt6701_t;

bool encoder_mt6701_init(encoder_mt6701_t *encoder,
                         SPI_HandleTypeDef *spi,
                         GPIO_TypeDef *chip_select_port,
                         uint16_t chip_select_pin);
bool encoder_mt6701_parse_ssi(const uint8_t frame[3],
                              uint32_t now_us,
                              encoder_sample_t *sample);
bool encoder_mt6701_read(encoder_mt6701_t *encoder,
                         uint32_t now_us,
                         encoder_sample_t *sample);
bool encoder_mt6701_start_dma(encoder_mt6701_t *encoder);
bool encoder_mt6701_take_dma_sample(encoder_mt6701_t *encoder,
                                    encoder_sample_t *sample);
bool encoder_mt6701_dma_timed_out(const encoder_mt6701_t *encoder,
                                  uint32_t now_cycles,
                                  uint32_t timeout_cycles);
/* Recover a transfer that exceeded timeout_cycles.  The caller must first
 * establish a latched drive fault and disarm the power stage; recovery
 * intentionally publishes an invalid BUS sample. */
bool encoder_mt6701_recover_dma_timeout(encoder_mt6701_t *encoder,
                                        uint32_t now_cycles,
                                        uint32_t timeout_cycles);

#endif
