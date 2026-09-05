/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define VBUS_Pin GPIO_PIN_0
#define VBUS_GPIO_Port GPIOA
#define CURRENT_A_Pin GPIO_PIN_1
#define CURRENT_A_GPIO_Port GPIOA
#define CURRENT_B_Pin GPIO_PIN_2
#define CURRENT_B_GPIO_Port GPIOA
#define CURRENT_C_Pin GPIO_PIN_3
#define CURRENT_C_GPIO_Port GPIOA
#define ENC_CS_Pin GPIO_PIN_4
#define ENC_CS_GPIO_Port GPIOA
#define DRV_FAULT_Pin GPIO_PIN_0
#define DRV_FAULT_GPIO_Port GPIOB
#define DRV_FAULT_EXTI_IRQn EXTI0_IRQn
#define DRV_ENABLE_Pin GPIO_PIN_1
#define DRV_ENABLE_GPIO_Port GPIOB
#define PWM_ENABLE_Pin GPIO_PIN_2
#define PWM_ENABLE_GPIO_Port GPIOB
#define USER_BUTTON_Pin GPIO_PIN_6
#define USER_BUTTON_GPIO_Port GPIOC
#define PWM_C_Pin GPIO_PIN_8
#define PWM_C_GPIO_Port GPIOA
#define PWM_B_Pin GPIO_PIN_9
#define PWM_B_GPIO_Port GPIOA
#define PWM_A_Pin GPIO_PIN_10
#define PWM_A_GPIO_Port GPIOA
#define CAN_STB_Pin GPIO_PIN_4
#define CAN_STB_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
