#include "main.h"

#include "bsp.h"

void app_init(void) __attribute__((weak));
void app_background(void) __attribute__((weak));

void app_init(void)
{
}

void app_background(void)
{
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    bsp_init();
    app_init();
    bsp_start_realtime();

    for (;;) {
        app_background();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    bsp_fatal_shutdown();
    for (;;) {
        __NOP();
    }
}

void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
    Error_Handler();
}
