#!/usr/bin/env python3
"""Check regeneration contracts and optional ELF invariants, without MCU access."""
import argparse
import re
from pathlib import Path
import subprocess
from vendor import check_source_layout

ROOT = Path(__file__).resolve().parents[1]


def require(condition, message):
    if not condition:
        raise SystemExit("Project contract failed: " + message)


def contains(file, token):
    require(
        token in (ROOT / file).read_text(encoding="utf8"), f"{file}: missing {token}"
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--elf", type=Path)
    parser.add_argument(
        "--bench", action="store_true", help="Expect the opt-in SWD transport"
    )
    args = parser.parse_args()
    ioc = dict(
        line.split("=", 1)
        for line in (ROOT / "canopen_foc.ioc").read_text().splitlines()
        if "=" in line and not line.startswith("#")
    )
    expected = {
        "ProjectManager.KeepUserCode": "true",
        "ProjectManager.TargetToolchain": "CMake",
        "ProjectManager.FirmwarePackage": "STM32Cube FW_G4 V1.6.3",
        "ProjectManager.LastFirmware": "false",
        "ProjectManager.HeapSize": "0x0",
        "ProjectManager.StackSize": "0x1000",
        "RCC.SYSCLKFreq_VALUE": "160000000",
        "TIM1.PeriodNoDither": "4000",
        "TIM1.PulseNoDither_4": "3680",
        "TIM1.CounterMode": "TIM_COUNTERMODE_CENTERALIGNED1",
        "NVIC.TIM1_UP_TIM16_IRQn": r"true\:2\:0\:false\:false\:true\:true\:true\:true",
        "PA8.GPIO_Label": "PWM_C",
        "PA9.GPIO_Label": "PWM_B",
        "PA10.GPIO_Label": "PWM_A",
        "PB0.GPIO_Label": "DRV_FAULT",
        "PB1.GPIO_Label": "DRV_ENABLE",
        "PB2.GPIO_Label": "PWM_ENABLE",
        "PB4.GPIO_Label": "CAN_STB",
        "PA4.GPIO_Label": "ENC_CS",
        "VP_CORDIC_VS_CORDIC.Mode": "CORDIC_Activate",
        "FDCAN1.FrameFormat": "FDCAN_FRAME_FD_BRS",
        "FDCAN1.DataPrescaler": "1",
        "FDCAN1.DataSyncJumpWidth": "4",
        "FDCAN1.DataTimeSeg1": "23",
        "FDCAN1.DataTimeSeg2": "8",
        "ADC1.NbrOfConversion": "4",
        "ADC1.InjNumberOfConversion": "2",
        "ADC1.DiscontinuousConvMode": "DISABLE",
        "ADC1.DMAContinuousRequests": "ENABLE",
        "ADC1.ExternalTrigConv": "ADC_EXTERNALTRIG_T1_TRGO",
        "ADC1.ExternalTrigConvEdge": "ADC_EXTERNALTRIGCONVEDGE_RISING",
        "ADC1.ExternalTrigInjecConv": "ADC_INJECTED_SOFTWARE_START",
        "ADC1.Overrun": "ADC_OVR_DATA_PRESERVED",
        "Dma.ADC1.0.Instance": "DMA1_Channel1",
        "Dma.ADC1.0.Mode": "DMA_CIRCULAR",
        "Dma.ADC1.0.PeriphDataAlignment": "DMA_PDATAALIGN_HALFWORD",
        "Dma.ADC1.0.MemDataAlignment": "DMA_MDATAALIGN_HALFWORD",
        "Dma.ADC1.0.Priority": "DMA_PRIORITY_VERY_HIGH",
    }
    for key, value in expected.items():
        require(ioc.get(key) == value, f"{key}: expected {value}, got {ioc.get(key)}")
    for file, token in [
        ("Core/Src/main.c", "roller_app_start();"),
        ("Core/Src/main.c", "MX_CORDIC_Init();"),
        ("Core/Src/cordic.c", "__HAL_RCC_CORDIC_CLK_ENABLE();"),
        ("Core/Src/stm32g4xx_it.c", "HAL_ADC_IRQHandler(&hadc1);"),
        ("Core/Src/stm32g4xx_it.c", "HAL_GPIO_EXTI_IRQHandler(DRV_FAULT_Pin);"),
        ("Core/Src/stm32g4xx_it.c", "HAL_FDCAN_IRQHandler(&hfdcan1);"),
        ("Core/Src/stm32g4xx_it.c", "HAL_TIM_IRQHandler(&htim6);"),
        ("Core/Src/stm32g4xx_it.c", "roller_app_encoder_trigger();"),
        ("Core/Src/adc.c", "ADC_EOC_SEQ_CONV"),
        ("Core/Src/adc.c", "ADC_CLOCK_SYNC_PCLK_DIV4"),
        ("Core/Src/adc.c", "ADC_EXTERNALTRIG_T1_TRGO"),
        ("Core/Src/adc.c", "ADC_INJECTED_SOFTWARE_START"),
        ("Core/Src/adc.c", "hadc1.Init.DiscontinuousConvMode = DISABLE;"),
        ("Core/Src/adc.c", "hadc1.Init.DMAContinuousRequests = ENABLE;"),
        ("Core/Src/adc.c", "DMA_REQUEST_ADC1"),
        ("Core/Src/adc.c", "__HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc1);"),
        ("Core/Src/main.c", "MX_DMA_Init();"),
        ("Core/Src/dma.c", "__HAL_RCC_DMAMUX1_CLK_ENABLE();"),
        ("Core/Src/stm32g4xx_it.c", "HAL_DMA_IRQHandler(&hdma_adc1);"),
        ("Core/Src/adc.c", "ADC_CHANNEL_TEMPSENSOR_ADC1"),
        ("Core/Src/adc.c", "ADC_CHANNEL_VREFINT"),
        ("Core/Src/spi.c", "SPI_DATASIZE_16BIT"),
        ("Core/Src/spi.c", "SPI_PHASE_2EDGE"),
        ("Core/Src/tim.c", "sConfigOC.Pulse = 3680;"),
        ("Core/Src/tim.c", "TIM_COUNTERMODE_CENTERALIGNED1"),
        ("Core/Src/tim.c", "HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 2, 0);"),
        ("Core/Src/fdcan.c", "FDCAN_FRAME_FD_BRS"),
        ("Core/Src/fdcan.c", "hfdcan1.Init.NominalPrescaler = 5;"),
        ("Core/Src/fdcan.c", "hfdcan1.Init.NominalTimeSeg1 = 25;"),
        ("Core/Src/fdcan.c", "hfdcan1.Init.NominalTimeSeg2 = 6;"),
        ("Core/Src/fdcan.c", "hfdcan1.Init.DataPrescaler = 1;"),
        ("Core/Src/fdcan.c", "hfdcan1.Init.DataTimeSeg1 = 23;"),
        ("Core/Src/fdcan.c", "hfdcan1.Init.DataTimeSeg2 = 8;"),
        ("RTOS/FreeRTOSConfig.h", "#define configSUPPORT_DYNAMIC_ALLOCATION 0"),
        ("linker/STM32G431CB_FLASH.ld", "LENGTH = 124K"),
        ("CMakeLists.txt", "set(CMAKE_CXX_STANDARD 23)"),
    ]:
        contains(file, token)
    interrupts = (ROOT / "Core/Src/stm32g4xx_it.c").read_text()
    for handler in ("SVC_Handler", "PendSV_Handler", "SysTick_Handler"):
        require(
            not re.search(r"void\s+" + handler + r"\s*\(", interrupts),
            "FreeRTOS must own " + handler,
        )
    check_source_layout()
    if args.elf:
        output = subprocess.check_output(
            ["arm-none-eabi-nm", "--defined-only", str(args.elf)], text=True
        )
        symbol_types = {
            parts[-1]: parts[-2]
            for line in output.splitlines()
            if len(parts := line.split()) >= 2
        }
        symbols = set(symbol_types)
        bench_symbols = {
            "roller_bench_rx",
            "roller_bench_tx",
            "roller_bench_state",
            "roller_bench_request",
        }
        require(
            (
                bench_symbols.issubset(symbols)
                if args.bench
                else not bench_symbols.intersection(symbols)
            ),
            "Bench transport must match --bench; production firmware must exclude it",
        )
        for symbol in (
            "ADC1_2_IRQHandler",
            "TIM1_UP_TIM16_IRQHandler",
            "DMA1_Channel1_IRQHandler",
            "DMA1_Channel2_IRQHandler",
            "DMA1_Channel3_IRQHandler",
            "EXTI0_IRQHandler",
            "FDCAN1_IT0_IRQHandler",
            "SVC_Handler",
            "PendSV_Handler",
            "SysTick_Handler",
            "roller_app_start",
            "roller_debug",
        ):
            require(symbol in symbols, "missing ELF symbol " + symbol)
            require(
                symbol_types[symbol].upper() != "W",
                "weak default handler in ELF: " + symbol,
            )
        banned = {
            "malloc",
            "calloc",
            "realloc",
            "free",
            "_malloc_r",
            "_calloc_r",
            "_realloc_r",
            "_free_r",
            "_sbrk",
            "pvPortMalloc",
            "vPortFree",
        }
        require(
            not symbols.intersection(banned),
            "allocator linked: " + str(symbols.intersection(banned)),
        )
        require(
            not any(re.match(r"_Zn[aw]", s) for s in symbols), "C++ operator new linked"
        )
    print(
        "PASS: CubeMX, interrupt ownership, local dependencies"
        + (", static-allocation ELF" if args.elf else "")
    )


if __name__ == "__main__":
    main()
