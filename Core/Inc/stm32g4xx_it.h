#ifndef STM32G4XX_IT_H
#define STM32G4XX_IT_H

#ifdef __cplusplus
extern "C" {
#endif

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void ADC1_2_IRQHandler(void);
void TIM1_BRK_TIM15_IRQHandler(void);
void FDCAN1_IT0_IRQHandler(void);
void FDCAN1_IT1_IRQHandler(void);
void I2C1_EV_IRQHandler(void);
void I2C1_ER_IRQHandler(void);
void SPI3_IRQHandler(void);
void DMA1_Channel1_IRQHandler(void);
void DMA1_Channel2_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif
