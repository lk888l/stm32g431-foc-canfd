#include "main.h"

#include "board_config.h"
#include "bsp.h"

void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    /* Deferred 1 kHz drive/safety supervisor: below all realtime I/O IRQs. */
    HAL_NVIC_SetPriority(PendSV_IRQn, 5U, 0U);
#if defined(PWR_CR3_UCPD_DBDIS)
    HAL_PWREx_DisableUCPDDeadBattery();
#endif
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *adc)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_ADC12_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    if (adc->Instance == ADC1) {
        /* PA1 = ADC1_IN2 phase-A current, PA3 = ADC1_IN4 DC bus.
         * PB0/ADC1_IN15 is reserved for an optional third shunt but is not
         * sampled by this two-shunt build. */
        gpio.Pin = GPIO_PIN_1 | GPIO_PIN_3;
        HAL_GPIO_Init(GPIOA, &gpio);
    } else if (adc->Instance == ADC2) {
        gpio.Pin = GPIO_PIN_7;
        HAL_GPIO_Init(GPIOA, &gpio);
    }

    HAL_NVIC_SetPriority(ADC1_2_IRQn, 1U, 0U);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *fdcan)
{
    GPIO_InitTypeDef gpio = {0};
    if (fdcan->Instance != FDCAN1) {
        return;
    }

    __HAL_RCC_FDCAN_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOA, &gpio);

    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 3U, 0U);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
    HAL_NVIC_SetPriority(FDCAN1_IT1_IRQn, 3U, 1U);
    HAL_NVIC_EnableIRQ(FDCAN1_IT1_IRQn);
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *i2c)
{
    GPIO_InitTypeDef gpio = {0};
    if (i2c->Instance != I2C1) {
        return;
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_AF_OD;
#if BOARD_PROFILE == BOARD_PROFILE_LAB_SIMPLEFOC_AS5600
    /* The AS5600-S board already has 4.7 kOhm pull-ups.  Weak MCU pull-ups
     * keep both lines released during connector/power sequencing and are
     * harmless in parallel with the external resistors. */
    gpio.Pull = GPIO_PULLUP;
#else
    gpio.Pull = GPIO_NOPULL;
#endif
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &gpio);

    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 4U, 0U);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 4U, 1U);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
}

void HAL_SPI_MspInit(SPI_HandleTypeDef *spi)
{
    GPIO_InitTypeDef gpio = {0};
    if (spi->Instance != SPI3) {
        return;
    }

    __HAL_RCC_SPI3_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &gpio);
    gpio.Pin = GPIO_PIN_5;
    HAL_GPIO_Init(GPIOB, &gpio);

    hdma_spi3_rx.Instance = DMA1_Channel1;
    hdma_spi3_rx.Init.Request = DMA_REQUEST_SPI3_RX;
    hdma_spi3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spi3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi3_rx.Init.Mode = DMA_NORMAL;
    hdma_spi3_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    if (HAL_DMA_Init(&hdma_spi3_rx) != HAL_OK) {
        Error_Handler();
    }
    __HAL_LINKDMA(spi, hdmarx, hdma_spi3_rx);

    hdma_spi3_tx.Instance = DMA1_Channel2;
    hdma_spi3_tx.Init.Request = DMA_REQUEST_SPI3_TX;
    hdma_spi3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi3_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi3_tx.Init.Mode = DMA_NORMAL;
    hdma_spi3_tx.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_spi3_tx) != HAL_OK) {
        Error_Handler();
    }
    __HAL_LINKDMA(spi, hdmatx, hdma_spi3_tx);

    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 2U, 0U);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 2U, 1U);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

    HAL_NVIC_SetPriority(SPI3_IRQn, 2U, 0U);
    HAL_NVIC_EnableIRQ(SPI3_IRQn);
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *timer)
{
    if (timer->Instance == TIM2) {
        __HAL_RCC_TIM2_CLK_ENABLE();
    } else if (timer->Instance == TIM1) {
        __HAL_RCC_TIM1_CLK_ENABLE();
        HAL_NVIC_SetPriority(TIM1_BRK_TIM15_IRQn, 0U, 0U);
        HAL_NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
    }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *timer)
{
    GPIO_InitTypeDef gpio = {0};
    if (timer->Instance != TIM1) {
        return;
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio.Pin = PHASE_A_H_Pin | PHASE_B_H_Pin | PHASE_C_H_Pin;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF6_TIM1;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* On STM32G431Cx UFQFPN48 PA15/TIM1_BKIN is AF9, not the AF6 used by
     * TIM1_CH1..3 on PA8..PA10. */
    gpio.Pin = GATE_FAULT_Pin;
    gpio.Alternate = GPIO_AF9_TIM1;
    HAL_GPIO_Init(GATE_FAULT_GPIO_Port, &gpio);

#if BOARD_PROFILE == BOARD_PROFILE_PRODUCTION_3SHUNT_MT6701
    gpio.Pin = PHASE_A_L_Pin | PHASE_B_L_Pin | PHASE_C_L_Pin;
    gpio.Alternate = GPIO_AF6_TIM1;
    HAL_GPIO_Init(GPIOB, &gpio);
#endif
}

void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    HAL_GPIO_WritePin(GATE_ENABLE_GPIO_Port, GATE_ENABLE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DRIVER_RESET_GPIO_Port, DRIVER_RESET_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DRIVER_SLEEP_GPIO_Port, DRIVER_SLEEP_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CAN_STANDBY_GPIO_Port, CAN_STANDBY_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ENCODER_CS_GPIO_Port, ENCODER_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);

    gpio.Pin = GATE_ENABLE_Pin | DRIVER_RESET_Pin | DRIVER_SLEEP_Pin |
               CAN_STANDBY_Pin | ENCODER_CS_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin = STATUS_LED_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &gpio);
}
