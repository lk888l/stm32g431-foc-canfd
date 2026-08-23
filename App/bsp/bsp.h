#ifndef BRUSHLESS_BSP_H
#define BRUSHLESS_BSP_H

#include <stdbool.h>
#include <stdint.h>

#include "main.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern DMA_HandleTypeDef hdma_spi3_rx;
extern DMA_HandleTypeDef hdma_spi3_tx;
extern FDCAN_HandleTypeDef hfdcan1;
extern I2C_HandleTypeDef hi2c1;
extern IWDG_HandleTypeDef hiwdg;
extern SPI_HandleTypeDef hspi3;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

typedef struct {
    uint16_t phase_a_raw;
    uint16_t phase_b_raw;
    uint32_t timestamp_cycles;
    bool sampling_window_valid;
} bsp_current_sample_t;

void bsp_init(void);
void bsp_start_realtime(void);
void bsp_gate_arm(void);
void bsp_gate_disarm(void);
bool bsp_gate_fault_active(void);
void bsp_gate_latch_break(void);
bool bsp_gate_break_latched(void);
bool bsp_gate_clear_break_latch(void);
bool bsp_gate_is_armed(void);
void bsp_pwm_write(float duty_a, float duty_b, float duty_c);
void bsp_pwm_set_sample_point(uint16_t counter_ticks);
bsp_current_sample_t bsp_current_sample_read(void);
/* Called from ADC1_2 IRQ immediately after both injected sequences complete.
 * It services the low-rate regular VBUS conversion only in the collision-free
 * gap mandated by STM32G431 erratum ES0431 section 2.5.10. */
void bsp_adc_injected_sequence_complete(void);
/* Captures ADC1 regular rank 1 and discards rank 2.  The duplicated VBUS
 * channel is the Rev-Z ES0431 section 2.5.8 workaround. */
void bsp_adc_regular_conversion_irq(void);
bool bsp_bus_voltage_read(float *voltage_v);
/* Serialize shared FDCAN Tx message-RAM access without holding PRIMASK across
 * the HAL payload copy.  Callers must release after every successful lock. */
bool bsp_fdcan_tx_try_lock(void);
void bsp_fdcan_tx_unlock(void);
uint32_t bsp_cycle_counter(void);
uint32_t bsp_micros(void);
void bsp_watchdog_refresh(void);
void bsp_fatal_shutdown(void);

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_ADC1_Init(void);
void MX_ADC2_Init(void);
void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
void MX_FDCAN1_Init(void);
void MX_I2C1_Init(void);
void MX_SPI3_Init(void);
void MX_IWDG_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *timer);

#endif
