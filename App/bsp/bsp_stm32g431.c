#include "bsp.h"

#include "board_config.h"

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_spi3_rx;
DMA_HandleTypeDef hdma_spi3_tx;
FDCAN_HandleTypeDef hfdcan1;
I2C_HandleTypeDef hi2c1;
IWDG_HandleTypeDef hiwdg;
SPI_HandleTypeDef hspi3;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

static volatile bool gate_armed;
static volatile bool gate_break_latched;
static volatile bool fdcan_tx_locked;
static volatile bool bus_sample_requested;
static volatile bool bus_sample_in_progress;
static volatile bool bus_sample_valid;
static volatile uint16_t bus_sample_raw;
static volatile uint32_t bus_sample_timestamp_us;
static volatile bool bus_sample_rank1_captured;
static volatile uint16_t bus_sample_rank1_raw;
static bool current_sample_uses_rank1;
static bool current_sample_warmup_pending;
static bool lab_retention_allowed;

#define BSP_BUS_SAMPLE_STALE_US UINT32_C(30000)
#define BSP_STM32G431_REV_Z UINT16_C(0x2001)
#define BSP_STM32G431_REV_Y UINT16_C(0x2002)
#define BSP_STM32G431_REV_X UINT16_C(0x2003)
#define BSP_TIM1_MAIN_OUTPUT_MASK \
    (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E)
#if BOARD_PROFILE == BOARD_PROFILE_PRODUCTION_3SHUNT_MT6701
#define BSP_TIM1_DRIVE_OUTPUT_MASK \
    (BSP_TIM1_MAIN_OUTPUT_MASK | TIM_CCER_CC1NE | \
     TIM_CCER_CC2NE | TIM_CCER_CC3NE)
#else
#define BSP_TIM1_DRIVE_OUTPUT_MASK BSP_TIM1_MAIN_OUTPUT_MASK
#endif

static float clamp01(float value)
{
    if (value < 0.0f) {
        return 0.0f;
    }
    if (value > 1.0f) {
        return 1.0f;
    }
    return value;
}

void bsp_init(void)
{
    const uint32_t reset_flags = RCC->CSR;
    const uint16_t silicon_revision = (uint16_t)(
        (DBGMCU->IDCODE & DBGMCU_IDCODE_REV_ID_Msk) >>
        DBGMCU_IDCODE_REV_ID_Pos);

    /* Debug PIN/SW resets may retain a guarded calibration; command retention
     * is validated independently so an incomplete mailbox can never run.
     * Power, watchdog, low-power and option-byte resets force a fresh encoder
     * alignment. */
    lab_retention_allowed =
        ((reset_flags & (RCC_CSR_SFTRSTF | RCC_CSR_PINRSTF)) != 0U) &&
        ((reset_flags & (RCC_CSR_BORRSTF | RCC_CSR_IWDGRSTF |
                         RCC_CSR_WWDGRSTF | RCC_CSR_LPWRRSTF |
                         RCC_CSR_OBLRSTF)) == 0U);
    __HAL_RCC_CLEAR_RESET_FLAGS();

    gate_armed = false;
    gate_break_latched = false;
    fdcan_tx_locked = false;
    bus_sample_requested = false;
    bus_sample_in_progress = false;
    bus_sample_valid = false;
    bus_sample_raw = 0U;
    bus_sample_timestamp_us = 0U;
    bus_sample_rank1_captured = false;
    bus_sample_rank1_raw = 0U;
    current_sample_warmup_pending = true;

    /* Establish the power-stage safe state before a future or unidentified
     * die revision can take the fail-closed Error_Handler path. */
    MX_GPIO_Init();
    switch (silicon_revision) {
    case BSP_STM32G431_REV_Z:
        /* Rev Z: ES0431 2.5.8 applies, while 2.5.10 does not. Of two
         * consecutive same-channel conversions ST specifies keeping rank 1. */
        current_sample_uses_rank1 = true;
        break;
    case BSP_STM32G431_REV_Y:
        /* Rev Y: discard rank 1 as the ES0431 2.5.10 collision dummy. */
        current_sample_uses_rank1 = false;
        break;
    case BSP_STM32G431_REV_X:
        /* Rev X is unaffected by 2.5.8/2.5.10; rank 2 keeps one common path. */
        current_sample_uses_rank1 = false;
        break;
    default:
        /* A future or unidentified die revision needs a fresh errata review. */
        Error_Handler();
        break;
    }
    MX_ADC1_Init();
    MX_ADC2_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
#if BOARD_CAN_ENABLED
    MX_FDCAN1_Init();
#endif
#if BOARD_HARDWARE_I2C1_ENABLED
    MX_I2C1_Init();
#endif
    MX_SPI3_Init();

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

#if (BOARD_PROFILE == BOARD_PROFILE_LAB_SIMPLEFOC_AS5600) && \
    defined(BRUSHLESS_DEBUG_BUILD)
    /* ST-Link mailbox transactions briefly halt the core. Freeze the local
     * timeout/watchdog clocks so the guarded command can be committed before
     * the legacy tool resets the target. TIM1 deliberately keeps switching:
     * freezing an armed PWM timer could hold a phase output at a DC level. */
    __HAL_DBGMCU_FREEZE_TIM2();
    __HAL_DBGMCU_FREEZE_IWDG();
#endif

    if (HAL_TIM_Base_Start(&htim2) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) {
        Error_Handler();
    }

    /* Release RESET first, then SLEEP. The MS8313 needs about 1 ms to wake. */
    HAL_GPIO_WritePin(DRIVER_RESET_GPIO_Port, DRIVER_RESET_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DRIVER_SLEEP_GPIO_Port, DRIVER_SLEEP_Pin, GPIO_PIN_SET);
    HAL_Delay(2U);
}

void bsp_start_realtime(void)
{
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_BREAK);
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_BREAK);

    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK) {
        Error_Handler();
    }

    if ((HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) ||
        (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) ||
        (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK)) {
        Error_Handler();
    }

#if BOARD_PROFILE == BOARD_PROFILE_PRODUCTION_3SHUNT_MT6701
    if ((HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) ||
        (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) ||
        (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3) != HAL_OK)) {
        Error_Handler();
    }
#endif

    bsp_pwm_write(0.5f, 0.5f, 0.5f);
    /* HAL_TIM_PWM_Start() enables CCxE/MOE. Keep CH4/MOE alive for the ADC
     * trigger but remove every physical phase output before interrupts run. */
    bsp_gate_disarm();

    /* ES0431 2.5.9 applies to every known die revision: the first conversion
     * more than 1 ms after calibration or a preceding conversion can be
     * wrong.  bsp_current_sample_read() marks the first complete dual
     * sequence unusable; the ADC IRQ clears it without feeding the FOC loop. */
    current_sample_warmup_pending = true;
    if ((HAL_ADCEx_InjectedStart_IT(&hadc2) != HAL_OK) ||
        (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK)) {
        Error_Handler();
    }
#if BOARD_CAN_ENABLED
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_FDCAN_ActivateNotification(&hfdcan1,
                                       FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                                           FDCAN_IT_BUS_OFF |
                                           FDCAN_IT_ERROR_WARNING |
                                           FDCAN_IT_ERROR_PASSIVE,
                                       0U) != HAL_OK) {
        Error_Handler();
    }
    HAL_GPIO_WritePin(CAN_STANDBY_GPIO_Port, CAN_STANDBY_Pin, GPIO_PIN_RESET);
#endif

    MX_IWDG_Init();
}

void bsp_gate_arm(void)
{
    const uint32_t primask = __get_PRIMASK();

    /* Close the check/CCxE/MOE/enable race against TIM1 break IRQ.  BKIN
     * still acts asynchronously in hardware while IRQs are masked; the
     * second flag/pin check catches a fault that arrived inside this short
     * critical section. */
    __disable_irq();
    if (bsp_gate_fault_active() || gate_break_latched) {
        bsp_gate_disarm();
        __set_PRIMASK(primask);
        return;
    }

    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_BREAK);
    SET_BIT(htim1.Instance->CCER, BSP_TIM1_DRIVE_OUTPUT_MASK);
    __HAL_TIM_MOE_ENABLE(&htim1);
    HAL_GPIO_WritePin(GATE_ENABLE_GPIO_Port, GATE_ENABLE_Pin, GPIO_PIN_SET);
    __DMB();
    if (bsp_gate_fault_active() ||
        (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_BREAK) != RESET)) {
        gate_break_latched = true;
        bsp_gate_disarm();
    } else {
        gate_armed = true;
    }
    __set_PRIMASK(primask);
}

void bsp_gate_disarm(void)
{
    HAL_GPIO_WritePin(GATE_ENABLE_GPIO_Port, GATE_ENABLE_Pin, GPIO_PIN_RESET);
    if (htim1.Instance == TIM1) {
        /* CH4 must remain enabled under MOE because it triggers the ADC fast
         * loop. Isolate the bridge by removing only physical phase channels;
         * BKIN and fatal shutdown still clear MOE asynchronously/explicitly. */
        CLEAR_BIT(htim1.Instance->CCER, BSP_TIM1_DRIVE_OUTPUT_MASK);
        __DSB();
    }
    gate_armed = false;
}

bool bsp_gate_fault_active(void)
{
    return HAL_GPIO_ReadPin(GATE_FAULT_GPIO_Port, GATE_FAULT_Pin) == GPIO_PIN_RESET;
}

void bsp_gate_latch_break(void)
{
    gate_break_latched = true;
    bsp_gate_disarm();
}

bool bsp_gate_break_latched(void)
{
    return gate_break_latched;
}

bool bsp_gate_clear_break_latch(void)
{
    const uint32_t primask = __get_PRIMASK();
    bool cleared = false;

    __disable_irq();
    if (!bsp_gate_fault_active() && !bsp_gate_is_armed()) {
        __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_BREAK);
        __DMB();
        if (!bsp_gate_fault_active() &&
            (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_BREAK) == RESET)) {
            gate_break_latched = false;
            cleared = true;
        }
    }
    __set_PRIMASK(primask);
    if (!cleared) {
        return false;
    }
    return true;
}

bool bsp_gate_is_armed(void)
{
    return gate_armed && (htim1.Instance == TIM1) &&
           ((htim1.Instance->BDTR & TIM_BDTR_MOE) != 0U) &&
           ((htim1.Instance->CCER & BSP_TIM1_DRIVE_OUTPUT_MASK) ==
            BSP_TIM1_DRIVE_OUTPUT_MASK);
}

void bsp_pwm_write(float duty_a, float duty_b, float duty_c)
{
    const float period = (float)BOARD_PWM_PERIOD_TICKS;
    const float minimum = (float)BOARD_PWM_MIN_PULSE_TICKS / period;
    const float maximum = 1.0f - minimum;

    duty_a = clamp01(duty_a);
    duty_b = clamp01(duty_b);
    duty_c = clamp01(duty_c);

    if (duty_a < minimum) {
        duty_a = minimum;
    } else if (duty_a > maximum) {
        duty_a = maximum;
    }
    if (duty_b < minimum) {
        duty_b = minimum;
    } else if (duty_b > maximum) {
        duty_b = maximum;
    }
    if (duty_c < minimum) {
        duty_c = minimum;
    } else if (duty_c > maximum) {
        duty_c = maximum;
    }

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(duty_a * period));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(duty_b * period));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)(duty_c * period));
}

void bsp_pwm_set_sample_point(uint16_t counter_ticks)
{
    const uint16_t lower = (uint16_t)(BOARD_PWM_PERIOD_TICKS -
                                      BOARD_PWM_MIN_PULSE_TICKS +
                                      BOARD_PWM_DEADTIME_TICKS);
    const uint16_t upper = (uint16_t)(BOARD_PWM_PERIOD_TICKS - 1UL);

    if (counter_ticks < lower) {
        counter_ticks = lower;
    } else if (counter_ticks > upper) {
        counter_ticks = upper;
    }
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, counter_ticks);
}

bsp_current_sample_t bsp_current_sample_read(void)
{
    bsp_current_sample_t sample;
    if (current_sample_uses_rank1) {
        sample.phase_a_raw = (uint16_t)hadc1.Instance->JDR1;
        sample.phase_b_raw = (uint16_t)hadc2.Instance->JDR1;
    } else {
        sample.phase_a_raw = (uint16_t)hadc1.Instance->JDR2;
        sample.phase_b_raw = (uint16_t)hadc2.Instance->JDR2;
    }
    sample.timestamp_cycles = DWT->CYCCNT;
    sample.sampling_window_valid = !current_sample_warmup_pending;
    current_sample_warmup_pending = false;
    return sample;
}

void bsp_adc_injected_sequence_complete(void)
{
    const uint32_t regular_flags = ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR;

    /* ES0431 2.5.10 workaround: launch regular conversion only after JEOS,
     * never from arbitrary foreground timing.  The two consecutive VBUS ranks
     * also cover Rev-Z section 2.5.8; its first result is captured by the EOC
     * IRQ and its second result is deliberately discarded. */
    if (bus_sample_requested && !bus_sample_in_progress &&
        ((ADC1->CR & ADC_CR_ADSTART) == 0U) &&
        ((ADC1->CR & ADC_CR_ADEN) != 0U)) {
        ADC1->ISR = regular_flags;
        bus_sample_requested = false;
        bus_sample_in_progress = true;
        bus_sample_rank1_captured = false;
        __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_EOC | ADC_IT_EOS | ADC_IT_OVR);
        SET_BIT(ADC1->CR, ADC_CR_ADSTART);
    }
}

void bsp_adc_regular_conversion_irq(void)
{
    const uint32_t regular_flags = ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR;
    const uint32_t status = ADC1->ISR & regular_flags;
    uint16_t raw;

    if (status == 0U) {
        return;
    }
    if (!bus_sample_in_progress) {
        __HAL_ADC_DISABLE_IT(&hadc1, ADC_IT_EOC | ADC_IT_EOS | ADC_IT_OVR);
        ADC1->ISR = regular_flags;
        return;
    }

    /* OVR is configured as DATA_PRESERVED.  If the rank-1 interrupt is held
     * off until rank 2 completes (the normal case while the current ISR runs),
     * DR still contains rank 1 and EOS tells us that rank 2 was discarded in
     * hardware.  If rank 1 is serviced earlier, the later EOS entry reads and
     * discards rank 2 in software. */
    /* If the very first service already sees EOS, rank 2 completed before any
     * software read.  DATA_PRESERVED must therefore also have raised OVR and
     * kept rank 1 in DR.  Treat any other combination as a configuration or
     * DMA interference fault and invalidate VBUS rather than consuming rank 2. */
    if (!bus_sample_rank1_captured &&
        ((status & ADC_ISR_EOS) != 0U) &&
        ((status & ADC_ISR_OVR) == 0U)) {
        __HAL_ADC_DISABLE_IT(&hadc1, ADC_IT_EOC | ADC_IT_EOS | ADC_IT_OVR);
        ADC1->ISR = regular_flags;
        bus_sample_valid = false;
        bus_sample_in_progress = false;
        return;
    }

    raw = (uint16_t)ADC1->DR;
    if (!bus_sample_rank1_captured) {
        bus_sample_rank1_raw = raw;
        bus_sample_rank1_captured = true;
    }

    if ((status & ADC_ISR_EOS) != 0U) {
        __HAL_ADC_DISABLE_IT(&hadc1, ADC_IT_EOC | ADC_IT_EOS | ADC_IT_OVR);
        ADC1->ISR = regular_flags;
        bus_sample_raw = bus_sample_rank1_raw;
        bus_sample_timestamp_us = bsp_micros();
        __DMB();
        bus_sample_valid = true;
        bus_sample_in_progress = false;
    } else {
        ADC1->ISR = ADC_ISR_EOC | ADC_ISR_OVR;
    }
}

bool bsp_bus_voltage_read(float *voltage_v)
{
    uint16_t raw;
    uint32_t timestamp_us;
    bool valid;
    const uint32_t now_us = bsp_micros();
    const uint32_t primask = __get_PRIMASK();

    if ((voltage_v == NULL) || (BOARD_BUS_VOLTAGE_V_PER_COUNT <= 0.0f)) {
        return false;
    }

    __disable_irq();
    if (!bus_sample_requested && !bus_sample_in_progress) {
        bus_sample_requested = true;
    }
    raw = bus_sample_raw;
    timestamp_us = bus_sample_timestamp_us;
    valid = bus_sample_valid;
    __set_PRIMASK(primask);

    if (!valid ||
        ((uint32_t)(now_us - timestamp_us) > BSP_BUS_SAMPLE_STALE_US)) {
        return false;
    }
    if ((raw < BOARD_BUS_ADC_MIN_VALID_COUNT) ||
        (raw > BOARD_BUS_ADC_MAX_VALID_COUNT)) {
        return false;
    }
    *voltage_v = (float)raw * BOARD_BUS_VOLTAGE_V_PER_COUNT;
    return true;
}

bool bsp_fdcan_tx_try_lock(void)
{
    const uint32_t primask = __get_PRIMASK();
    bool acquired = false;

    __disable_irq();
    if (!fdcan_tx_locked) {
        fdcan_tx_locked = true;
        __DMB();
        acquired = true;
    }
    __set_PRIMASK(primask);
    return acquired;
}

void bsp_fdcan_tx_unlock(void)
{
    __DMB();
    fdcan_tx_locked = false;
}

uint32_t bsp_cycle_counter(void)
{
    return DWT->CYCCNT;
}

uint32_t bsp_micros(void)
{
    return (htim2.Instance == TIM2) ? __HAL_TIM_GET_COUNTER(&htim2) : 0U;
}

void bsp_watchdog_refresh(void)
{
    if (hiwdg.Instance == IWDG) {
        (void)HAL_IWDG_Refresh(&hiwdg);
    }
}

void bsp_status_led_set(bool on)
{
#if BOARD_STATUS_LED_ACTIVE_HIGH
    const GPIO_PinState state = on ? GPIO_PIN_SET : GPIO_PIN_RESET;
#else
    const GPIO_PinState state = on ? GPIO_PIN_RESET : GPIO_PIN_SET;
#endif
    HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, state);
}

bool bsp_lab_retention_allowed(void)
{
    return lab_retention_allowed;
}

void bsp_fatal_shutdown(void)
{
    bsp_gate_disarm();
    if (htim1.Instance == TIM1) {
        CLEAR_BIT(htim1.Instance->BDTR, TIM_BDTR_MOE);
        __DSB();
    }
    HAL_GPIO_WritePin(DRIVER_RESET_GPIO_Port, DRIVER_RESET_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DRIVER_SLEEP_GPIO_Port, DRIVER_SLEEP_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CAN_STANDBY_GPIO_Port, CAN_STANDBY_Pin, GPIO_PIN_SET);
    bsp_status_led_set(true);
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef oscillator = {0};
    RCC_ClkInitTypeDef clocks = {0};
    RCC_PeriphCLKInitTypeDef peripheral_clocks = {0};

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

#if BOARD_USE_8MHZ_HSE
    oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    oscillator.HSEState = RCC_HSE_ON;
#if BOARD_I2C1_USE_HSI
    oscillator.OscillatorType |= RCC_OSCILLATORTYPE_HSI;
    oscillator.HSIState = RCC_HSI_ON;
    oscillator.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
#endif
    oscillator.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    oscillator.PLL.PLLM = RCC_PLLM_DIV2;
#else
    oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    oscillator.HSIState = RCC_HSI_ON;
    oscillator.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    oscillator.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    oscillator.PLL.PLLM = RCC_PLLM_DIV4;
#endif
    oscillator.PLL.PLLState = RCC_PLL_ON;
    oscillator.PLL.PLLN = 85U;
    oscillator.PLL.PLLP = RCC_PLLP_DIV2;
    oscillator.PLL.PLLQ = RCC_PLLQ_DIV2;
    oscillator.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&oscillator) != HAL_OK) {
        Error_Handler();
    }

    clocks.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                       RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clocks.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clocks.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clocks.APB1CLKDivider = RCC_HCLK_DIV1;
    clocks.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clocks, FLASH_LATENCY_4) != HAL_OK) {
        Error_Handler();
    }

    peripheral_clocks.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
#if BOARD_CAN_ENABLED
    peripheral_clocks.PeriphClockSelection |= RCC_PERIPHCLK_FDCAN;
#endif
#if BOARD_HARDWARE_I2C1_ENABLED
    peripheral_clocks.PeriphClockSelection |= RCC_PERIPHCLK_I2C1;
#endif
    peripheral_clocks.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
#if BOARD_CAN_ENABLED
    peripheral_clocks.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
#endif
#if BOARD_HARDWARE_I2C1_ENABLED && BOARD_I2C1_USE_HSI
    peripheral_clocks.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
#elif BOARD_HARDWARE_I2C1_ENABLED
    peripheral_clocks.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
#endif
    if (HAL_RCCEx_PeriphCLKConfig(&peripheral_clocks) != HAL_OK) {
        Error_Handler();
    }
#if BOARD_USE_8MHZ_HSE
    /* A crystal loss raises NMI; NMI_Handler performs an immediate power-stage
     * shutdown before the independent watchdog resets the MCU. */
    HAL_RCC_EnableCSS();
#endif
}

void MX_ADC1_Init(void)
{
    ADC_InjectionConfTypeDef injected = {0};
    ADC_ChannelConfTypeDef regular = {0};
    ADC_MultiModeTypeDef multimode = {0};

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.GainCompensation = 0U;
    /* G4 HAL only programs InjectedNbrOfConversion > 1 while scan mode is
     * enabled.  Without this, JSQR silently collapses to one rank and JDR2
     * never contains the ES0431-protected current sample. */
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.NbrOfConversion = 2U;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc1.Init.OversamplingMode = DISABLE;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    /* ES0431 2.5.11 explicitly exempts ADC1/ADC2 operating in a dual mode
     * other than alternate-trigger mode from the cross-instance accuracy
     * limitation.  This is stronger than relying on the timer/ADC prescaler
     * relationship of workaround 3. */
    multimode.Mode = ADC_DUALMODE_INJECSIMULT;
    multimode.DMAAccessMode = ADC_DMAACCESSMODE_DISABLED;
    multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_1CYCLE;
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) {
        Error_Handler();
    }

    regular.Channel = ADC_CHANNEL_4;
    regular.Rank = ADC_REGULAR_RANK_1;
    regular.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
    regular.SingleDiff = ADC_SINGLE_ENDED;
    regular.OffsetNumber = ADC_OFFSET_NONE;
    regular.Offset = 0U;
    if (HAL_ADC_ConfigChannel(&hadc1, &regular) != HAL_OK) {
        Error_Handler();
    }
    regular.Rank = ADC_REGULAR_RANK_2;
    if (HAL_ADC_ConfigChannel(&hadc1, &regular) != HAL_OK) {
        Error_Handler();
    }

    injected.InjectedChannel = ADC_CHANNEL_2;
    injected.InjectedRank = ADC_INJECTED_RANK_1;
    injected.InjectedSamplingTime = ADC_SAMPLETIME_12CYCLES_5;
    injected.InjectedSingleDiff = ADC_SINGLE_ENDED;
    injected.InjectedOffsetNumber = ADC_OFFSET_NONE;
    injected.InjectedOffset = 0U;
    /* Repeat the channel in both ranks. Rev Y uses rank 1 as the ES0431
     * 2.5.10 dummy and keeps rank 2; Rev Z keeps rank 1 per the distinct
     * 2.5.8 workaround. ADC1/ADC2 convert as a dual injected-simultaneous
     * pair, explicitly exempt from the 2.5.11 accuracy limitation. */
    injected.InjectedNbrOfConversion = 2U;
    injected.InjectedDiscontinuousConvMode = DISABLE;
    injected.AutoInjectedConv = DISABLE;
    injected.QueueInjectedContext = DISABLE;
    injected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_CC4;
    injected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
    injected.InjecOversamplingMode = DISABLE;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &injected) != HAL_OK) {
        Error_Handler();
    }
    injected.InjectedRank = ADC_INJECTED_RANK_2;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &injected) != HAL_OK) {
        Error_Handler();
    }
}

void MX_ADC2_Init(void)
{
    ADC_InjectionConfTypeDef injected = {0};

    hadc2.Instance = ADC2;
    hadc2.Init = hadc1.Init;
    hadc2.Init.NbrOfConversion = 1U;
    if (HAL_ADC_Init(&hadc2) != HAL_OK) {
        Error_Handler();
    }

    injected.InjectedChannel = ADC_CHANNEL_4;
    injected.InjectedRank = ADC_INJECTED_RANK_1;
    injected.InjectedSamplingTime = ADC_SAMPLETIME_12CYCLES_5;
    injected.InjectedSingleDiff = ADC_SINGLE_ENDED;
    injected.InjectedOffsetNumber = ADC_OFFSET_NONE;
    injected.InjectedOffset = 0U;
    injected.InjectedNbrOfConversion = 2U;
    injected.InjectedDiscontinuousConvMode = DISABLE;
    injected.AutoInjectedConv = DISABLE;
    injected.QueueInjectedContext = DISABLE;
    injected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_CC4;
    injected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
    injected.InjecOversamplingMode = DISABLE;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &injected) != HAL_OK) {
        Error_Handler();
    }
    injected.InjectedRank = ADC_INJECTED_RANK_2;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &injected) != HAL_OK) {
        Error_Handler();
    }
}

void MX_TIM1_Init(void)
{
    TIM_ClockConfigTypeDef clock_source = {0};
    TIM_MasterConfigTypeDef master = {0};
    TIM_OC_InitTypeDef pwm = {0};
    TIM_BreakDeadTimeConfigTypeDef break_deadtime = {0};

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0U;
    htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
    htim1.Init.Period = BOARD_PWM_PERIOD_TICKS - 1UL;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0U;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
        Error_Handler();
    }

    clock_source.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &clock_source) != HAL_OK) {
        Error_Handler();
    }
    if ((HAL_TIM_PWM_Init(&htim1) != HAL_OK) || (HAL_TIM_OC_Init(&htim1) != HAL_OK)) {
        Error_Handler();
    }

    master.MasterOutputTrigger = TIM_TRGO_RESET;
    master.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &master) != HAL_OK) {
        Error_Handler();
    }

    pwm.OCMode = TIM_OCMODE_PWM1;
    pwm.Pulse = BOARD_PWM_PERIOD_TICKS / 2UL;
    pwm.OCPolarity = TIM_OCPOLARITY_HIGH;
    pwm.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    pwm.OCFastMode = TIM_OCFAST_DISABLE;
    pwm.OCIdleState = TIM_OCIDLESTATE_RESET;
    pwm.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if ((HAL_TIM_PWM_ConfigChannel(&htim1, &pwm, TIM_CHANNEL_1) != HAL_OK) ||
        (HAL_TIM_PWM_ConfigChannel(&htim1, &pwm, TIM_CHANNEL_2) != HAL_OK) ||
        (HAL_TIM_PWM_ConfigChannel(&htim1, &pwm, TIM_CHANNEL_3) != HAL_OK)) {
        Error_Handler();
    }

    /*
     * PWM2 produces one rising OC4REF edge per center-aligned carrier. Place
     * it in the middle of the guaranteed all-low-side sampling window, after
     * deadtime and current-amplifier settling rather than on a switching edge.
     */
    pwm.OCMode = TIM_OCMODE_PWM2;
    /* Place the second injected sampling aperture around the carrier crest,
     * centered inside the guaranteed low-side window at maximum duty. */
    pwm.Pulse = BOARD_PWM_PERIOD_TICKS -
                (BOARD_PWM_MIN_PULSE_TICKS / 4UL) - 1UL;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &pwm, TIM_CHANNEL_4) != HAL_OK) {
        Error_Handler();
    }

    break_deadtime.OffStateRunMode = TIM_OSSR_ENABLE;
    break_deadtime.OffStateIDLEMode = TIM_OSSI_ENABLE;
    break_deadtime.LockLevel = TIM_LOCKLEVEL_OFF;
    break_deadtime.DeadTime = BOARD_PWM_DEADTIME_TICKS;
    break_deadtime.BreakState = TIM_BREAK_ENABLE;
    break_deadtime.BreakPolarity = TIM_BREAKPOLARITY_LOW;
    break_deadtime.BreakFilter = 4U;
    break_deadtime.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
    break_deadtime.Break2State = TIM_BREAK2_DISABLE;
    break_deadtime.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
    break_deadtime.Break2Filter = 0U;
    break_deadtime.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
    break_deadtime.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &break_deadtime) != HAL_OK) {
        Error_Handler();
    }
    HAL_TIM_MspPostInit(&htim1);
}

void MX_TIM2_Init(void)
{
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = (BOARD_SYSCLK_HZ / 1000000UL) - 1UL;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = UINT32_MAX;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }
}

void MX_FDCAN1_Init(void)
{
    FDCAN_FilterTypeDef reject_remote = {0};
    FDCAN_FilterTypeDef mit_command = {0};

    hfdcan1.Instance = FDCAN1;
    hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
    hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
    hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
    hfdcan1.Init.AutoRetransmission = ENABLE;
    hfdcan1.Init.TransmitPause = ENABLE;
    hfdcan1.Init.ProtocolException = DISABLE;
    hfdcan1.Init.NominalPrescaler = 10U;
    hfdcan1.Init.NominalSyncJumpWidth = 4U;
    hfdcan1.Init.NominalTimeSeg1 = 27U;
    hfdcan1.Init.NominalTimeSeg2 = 6U;
    hfdcan1.Init.DataPrescaler = 5U;
    hfdcan1.Init.DataSyncJumpWidth = 3U;
    hfdcan1.Init.DataTimeSeg1 = 13U;
    hfdcan1.Init.DataTimeSeg2 = 3U;
    hfdcan1.Init.StdFiltersNbr = 8U;
    hfdcan1.Init.ExtFiltersNbr = 4U;
    hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
        Error_Handler();
    }

    reject_remote.IdType = FDCAN_STANDARD_ID;
    reject_remote.FilterIndex = 0U;
    reject_remote.FilterType = FDCAN_FILTER_RANGE;
    reject_remote.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    reject_remote.FilterID1 = 0x000U;
    reject_remote.FilterID2 = 0x7FFU;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &reject_remote) != HAL_OK) {
        Error_Handler();
    }

    mit_command.IdType = FDCAN_EXTENDED_ID;
    mit_command.FilterIndex = 0U;
    mit_command.FilterType = FDCAN_FILTER_MASK;
    mit_command.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    mit_command.FilterID1 = BOARD_MIT_COMMAND_EXT_ID;
    mit_command.FilterID2 = 0x1FFFFFFFU;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &mit_command) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                     FDCAN_REJECT,
                                     FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE) != HAL_OK) {
        Error_Handler();
    }
}

void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = BOARD_I2C1_TIMING;
    hi2c1.Init.OwnAddress1 = 0U;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0U;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }
    if ((HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) ||
        (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0U) != HAL_OK)) {
        Error_Handler();
    }
}

void MX_SPI3_Init(void)
{
    hspi3.Instance = SPI3;
    hspi3.Init.Mode = SPI_MODE_MASTER;
    hspi3.Init.Direction = SPI_DIRECTION_2LINES;
    hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
    /* MT6701 SSI changes data on the rising edge; sample the falling edge. */
    hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi3.Init.NSS = SPI_NSS_SOFT;
    hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi3.Init.CRCPolynomial = 7U;
    hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    if (HAL_SPI_Init(&hspi3) != HAL_OK) {
        Error_Handler();
    }
}

void MX_IWDG_Init(void)
{
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
    /* Approximately 100 ms at the nominal 32 kHz LSI. */
    hiwdg.Init.Reload = 99U;
    hiwdg.Init.Window = 4095U;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
        Error_Handler();
    }
}

bool bsp_i2c1_recover(void)
{
    if (hi2c1.Instance != I2C1) {
        return false;
    }

    /* A blocking HAL timeout restores the software state but can leave START
     * set in CR2.  The peripheral then owns SCL/SDA indefinitely and every
     * later probe sees BUSY.  A kernel reset is sufficient on this board: a
     * halted diagnostic verified that both externally pulled-up lines release
     * high as soon as I2C1 is disabled. */
    __HAL_I2C_DISABLE(&hi2c1);
    __HAL_RCC_I2C1_FORCE_RESET();
    __DSB();
    __HAL_RCC_I2C1_RELEASE_RESET();
    NVIC_ClearPendingIRQ(I2C1_EV_IRQn);
    NVIC_ClearPendingIRQ(I2C1_ER_IRQn);

    hi2c1.State = HAL_I2C_STATE_RESET;
    hi2c1.Lock = HAL_UNLOCKED;
    hi2c1.Mode = HAL_I2C_MODE_NONE;
    hi2c1.ErrorCode = HAL_I2C_ERROR_NONE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        return false;
    }
    if ((HAL_I2CEx_ConfigAnalogFilter(&hi2c1,
                                      I2C_ANALOGFILTER_ENABLE) != HAL_OK) ||
        (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0U) != HAL_OK)) {
        return false;
    }
    return true;
}
