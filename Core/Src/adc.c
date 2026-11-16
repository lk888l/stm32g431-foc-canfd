/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "adc.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

ADC_HandleTypeDef hadc1, hadc2;
DMA_HandleTypeDef hdma_adc1;
static void configure(ADC_HandleTypeDef *adc, ADC_TypeDef *instance)
{
  adc->Instance = instance;
  adc->Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  adc->Init.Resolution = ADC_RESOLUTION_12B;
  adc->Init.DataAlign = ADC_DATAALIGN_RIGHT;
  adc->Init.ScanConvMode = ADC_SCAN_ENABLE;
  adc->Init.EOCSelection = ADC_EOC_SEQ_CONV;
  adc->Init.NbrOfConversion = 3;
  adc->Init.ExternalTrigConv = instance == ADC1 ? ADC_EXTERNALTRIG_T1_TRGO : ADC_SOFTWARE_START;
  adc->Init.ExternalTrigConvEdge = instance == ADC1 ? ADC_EXTERNALTRIGCONVEDGE_RISING : ADC_EXTERNALTRIGCONVEDGE_NONE;
  adc->Init.DMAContinuousRequests = ENABLE;
  adc->Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(adc) != HAL_OK) Error_Handler();
}
static void channel(ADC_HandleTypeDef *adc, uint32_t input, uint32_t rank)
{
  ADC_ChannelConfTypeDef cfg = {0};
  cfg.Channel = input;
  cfg.Rank = rank;
  cfg.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  cfg.SingleDiff = ADC_SINGLE_ENDED;
  cfg.OffsetNumber = ADC_OFFSET_NONE;
  if (HAL_ADC_ConfigChannel(adc, &cfg) != HAL_OK) Error_Handler();
}
void MX_ADC1_Init(void)
{
  configure(&hadc1, ADC1);
  ADC_MultiModeTypeDef mode = {0};
  mode.Mode = ADC_DUALMODE_REGSIMULT;
  mode.DMAAccessMode = ADC_DMAACCESSMODE_12_10_BITS;
  mode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_1CYCLE;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &mode) != HAL_OK) Error_Handler();
  channel(&hadc1, ADC_CHANNEL_1, ADC_REGULAR_RANK_1);
  channel(&hadc1, ADC_CHANNEL_15, ADC_REGULAR_RANK_2);
  channel(&hadc1, ADC_CHANNEL_12, ADC_REGULAR_RANK_3);
  ADC_InjectionConfTypeDef slow = {0};
  slow.InjectedChannel = ADC_CHANNEL_TEMPSENSOR_ADC1;
  slow.InjectedRank = ADC_INJECTED_RANK_1;
  slow.InjectedSamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  slow.InjectedSingleDiff = ADC_SINGLE_ENDED;
  slow.InjectedOffsetNumber = ADC_OFFSET_NONE;
  slow.InjectedNbrOfConversion = 2;
  slow.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  slow.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_NONE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &slow) != HAL_OK) Error_Handler();
  slow.InjectedChannel = ADC_CHANNEL_VREFINT;
  slow.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &slow) != HAL_OK) Error_Handler();
}
void MX_ADC2_Init(void)
{
  configure(&hadc2, ADC2);
  channel(&hadc2, ADC_CHANNEL_2, ADC_REGULAR_RANK_1);
  channel(&hadc2, ADC_CHANNEL_5, ADC_REGULAR_RANK_2);
  channel(&hadc2, ADC_CHANNEL_5, ADC_REGULAR_RANK_3);
}
void HAL_ADC_MspInit(ADC_HandleTypeDef *adcHandle)
{
  GPIO_InitTypeDef gpio = {0};
  RCC_PeriphCLKInitTypeDef clk = {0};
  clk.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
  clk.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&clk) != HAL_OK) Error_Handler();
  __HAL_RCC_ADC12_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  gpio.Mode = GPIO_MODE_ANALOG;
  gpio.Pull = GPIO_NOPULL;
  if (adcHandle->Instance == ADC1) {
    gpio.Pin = CURRENT_A_Pin;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = CURRENT_C_Pin | NTC_Pin;
    HAL_GPIO_Init(GPIOB, &gpio);
    hdma_adc1.Instance = DMA1_Channel1;
    hdma_adc1.Init.Request = DMA_REQUEST_ADC1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) Error_Handler();
    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc1);
  } else {
    gpio.Pin = CURRENT_B_Pin;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = VBUS_Pin;
    HAL_GPIO_Init(GPIOC, &gpio);
  }
  HAL_NVIC_SetPriority(ADC1_2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
}
void HAL_ADC_MspDeInit(ADC_HandleTypeDef *adcHandle)
{
  if (adcHandle->Instance == ADC1) HAL_DMA_DeInit(adcHandle->DMA_Handle);
}
