#ifndef BRUSHLESS_MAIN_H
#define BRUSHLESS_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif
#include "stm32g4xx_hal.h"

void Error_Handler(void);

/* Production reference pin contract. Change only with the board schematic. */
#define PHASE_A_H_GPIO_Port GPIOA
#define PHASE_A_H_Pin       GPIO_PIN_8
#define PHASE_B_H_GPIO_Port GPIOA
#define PHASE_B_H_Pin       GPIO_PIN_9
#define PHASE_C_H_GPIO_Port GPIOA
#define PHASE_C_H_Pin       GPIO_PIN_10

#define PHASE_A_L_GPIO_Port GPIOB
#define PHASE_A_L_Pin       GPIO_PIN_13
#define PHASE_B_L_GPIO_Port GPIOB
#define PHASE_B_L_Pin       GPIO_PIN_14
#define PHASE_C_L_GPIO_Port GPIOB
#define PHASE_C_L_Pin       GPIO_PIN_15

#define GATE_ENABLE_GPIO_Port GPIOB
#define GATE_ENABLE_Pin       GPIO_PIN_10
#define GATE_FAULT_GPIO_Port  GPIOA
#define GATE_FAULT_Pin        GPIO_PIN_15
#define DRIVER_RESET_GPIO_Port GPIOB
#define DRIVER_RESET_Pin       GPIO_PIN_11
#define DRIVER_SLEEP_GPIO_Port GPIOB
#define DRIVER_SLEEP_Pin       GPIO_PIN_12

#define CAN_STANDBY_GPIO_Port GPIOB
#define CAN_STANDBY_Pin       GPIO_PIN_3

#define ENCODER_CS_GPIO_Port GPIOB
#define ENCODER_CS_Pin       GPIO_PIN_4

#define STATUS_LED_GPIO_Port GPIOC
#define STATUS_LED_Pin       GPIO_PIN_13

#ifdef __cplusplus
}
#endif

#endif
