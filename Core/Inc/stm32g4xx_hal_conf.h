#ifndef STM32G4XX_HAL_CONF_H
#define STM32G4XX_HAL_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

#define HAL_MODULE_ENABLED
#define HAL_ADC_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_EXTI_MODULE_ENABLED
#define HAL_FDCAN_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_I2C_MODULE_ENABLED
#define HAL_IWDG_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_SPI_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED

#define HSE_VALUE              8000000UL
#define HSE_STARTUP_TIMEOUT    100UL
#define HSI_VALUE             16000000UL
#define HSI48_VALUE           48000000UL
#define LSI_VALUE                32000UL
#define LSE_VALUE                32768UL
#define LSE_STARTUP_TIMEOUT   5000UL
#define EXTERNAL_CLOCK_VALUE   12288000UL

#define VDD_VALUE               3300UL
#define TICK_INT_PRIORITY       15UL
#define USE_RTOS                   0U
#define PREFETCH_ENABLE            0U
#define INSTRUCTION_CACHE_ENABLE   1U
#define DATA_CACHE_ENABLE          1U

#define USE_HAL_ADC_REGISTER_CALLBACKS    0U
#define USE_HAL_FDCAN_REGISTER_CALLBACKS  0U
#define USE_HAL_I2C_REGISTER_CALLBACKS    0U
#define USE_HAL_SPI_REGISTER_CALLBACKS    0U
#define USE_HAL_TIM_REGISTER_CALLBACKS    0U

#define USE_FULL_ASSERT 1U

#include "stm32g4xx_hal_rcc.h"
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_hal_exti.h"
#include "stm32g4xx_hal_dma.h"
#include "stm32g4xx_hal_cortex.h"
#include "stm32g4xx_hal_adc.h"
#include "stm32g4xx_hal_adc_ex.h"
#include "stm32g4xx_hal_fdcan.h"
#include "stm32g4xx_hal_flash.h"
#include "stm32g4xx_hal_flash_ex.h"
#include "stm32g4xx_hal_i2c.h"
#include "stm32g4xx_hal_i2c_ex.h"
#include "stm32g4xx_hal_iwdg.h"
#include "stm32g4xx_hal_pwr.h"
#include "stm32g4xx_hal_pwr_ex.h"
#include "stm32g4xx_hal_spi.h"
#include "stm32g4xx_hal_spi_ex.h"
#include "stm32g4xx_hal_tim.h"
#include "stm32g4xx_hal_tim_ex.h"

void assert_failed(uint8_t *file, uint32_t line);

#ifdef USE_FULL_ASSERT
#define assert_param(expr)                                                     \
    ((expr) ? (void)0U                                                        \
            : assert_failed((uint8_t *)__FILE__, (uint32_t)__LINE__))
#else
#define assert_param(expr) ((void)0U)
#endif

#ifdef __cplusplus
}
#endif

#endif
