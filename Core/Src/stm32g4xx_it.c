#include "stm32g4xx_it.h"

#include "bsp.h"

void app_current_sample_irq(uint16_t phase_a_raw,
                            uint16_t phase_b_raw,
                            uint32_t timestamp_us) __attribute__((weak));
void app_break_irq(void) __attribute__((weak));
void app_adc_sync_fault_irq(void) __attribute__((weak));
void app_supervisor_irq(uint32_t timestamp_us) __attribute__((weak));

void app_current_sample_irq(uint16_t phase_a_raw,
                            uint16_t phase_b_raw,
                            uint32_t timestamp_us)
{
    (void)phase_a_raw;
    (void)phase_b_raw;
    (void)timestamp_us;
}

void app_break_irq(void)
{
}

void app_adc_sync_fault_irq(void)
{
}

void app_supervisor_irq(uint32_t timestamp_us)
{
    (void)timestamp_us;
}

void NMI_Handler(void)
{
    /* Clock-security and other NMI causes are not recoverable while driving
     * PWM.  Put the power stage in its hardware-safe state before waiting for
     * the independent watchdog reset. */
    bsp_fatal_shutdown();
    for (;;) {
    }
}

void HardFault_Handler(void)
{
    bsp_fatal_shutdown();
    for (;;) {
    }
}

void MemManage_Handler(void)
{
    HardFault_Handler();
}

void BusFault_Handler(void)
{
    HardFault_Handler();
}

void UsageFault_Handler(void)
{
    HardFault_Handler();
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
    app_supervisor_irq(bsp_micros());
}

void SysTick_Handler(void)
{
    HAL_IncTick();
}

void ADC1_2_IRQHandler(void)
{
    uint32_t adc1_jeos;
    uint32_t adc2_jeos;

    adc1_jeos = ADC1->ISR & ADC_ISR_JEOS;
    adc2_jeos = ADC2->ISR & ADC_ISR_JEOS;

    /* Dual injected-simultaneous completion is nominally same-cycle. Allow a
     * few core cycles for flag synchronization, but never consume only one
     * current sample or leave a level-sensitive mismatch interrupt spinning. */
    if ((adc1_jeos == 0U) != (adc2_jeos == 0U)) {
        uint32_t retries = 32U;
        while (retries-- > 0U) {
            __NOP();
            adc1_jeos = ADC1->ISR & ADC_ISR_JEOS;
            adc2_jeos = ADC2->ISR & ADC_ISR_JEOS;
            if ((adc1_jeos != 0U) && (adc2_jeos != 0U)) {
                break;
            }
        }
    }
    if ((adc1_jeos != 0U) && (adc2_jeos != 0U)) {
        const bsp_current_sample_t current = bsp_current_sample_read();
        ADC1->ISR = ADC_ISR_JEOS | ADC_ISR_JEOC;
        ADC2->ISR = ADC_ISR_JEOS | ADC_ISR_JEOC;
        bsp_adc_injected_sequence_complete();
        if (current.sampling_window_valid) {
            app_current_sample_irq(current.phase_a_raw,
                                   current.phase_b_raw,
                                   bsp_micros());
        }
    } else if ((adc1_jeos != 0U) || (adc2_jeos != 0U)) {
        ADC1->ISR = ADC_ISR_JEOS | ADC_ISR_JEOC;
        ADC2->ISR = ADC_ISR_JEOS | ADC_ISR_JEOC;
        bsp_gate_disarm();
        app_adc_sync_fault_irq();
    }

    /* Current feedback has priority if flags ever coincide.  Regular VBUS
     * uses OVR_DATA_PRESERVED, so delaying its service cannot overwrite the
     * Rev-Z rank-1 result with the deliberately discarded rank 2. */
    if (((ADC1->ISR & (ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR)) != 0U) &&
        ((ADC1->IER &
          (ADC_IER_EOCIE | ADC_IER_EOSIE | ADC_IER_OVRIE)) != 0U)) {
        bsp_adc_regular_conversion_irq();
    }
}

void TIM1_BRK_TIM15_IRQHandler(void)
{
    if ((__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_BREAK) != RESET) &&
        (__HAL_TIM_GET_IT_SOURCE(&htim1, TIM_IT_BREAK) != RESET)) {
        __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_BREAK);
        bsp_gate_latch_break();
        app_break_irq();
    }
}

void FDCAN1_IT0_IRQHandler(void)
{
    HAL_FDCAN_IRQHandler(&hfdcan1);
}

void FDCAN1_IT1_IRQHandler(void)
{
    HAL_FDCAN_IRQHandler(&hfdcan1);
}

void I2C1_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&hi2c1);
}

void I2C1_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&hi2c1);
}

void SPI3_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&hspi3);
}

void DMA1_Channel1_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_spi3_rx);
}

void DMA1_Channel2_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_spi3_tx);
}
