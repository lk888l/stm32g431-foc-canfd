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
        "ProjectManager.HeapSize": "0x0",
        "RCC.SYSCLKFreq_VALUE": "160000000",
        "TIM1.PeriodNoDither": "4000", "TIM1.PulseNoDither_4": "3920",
        "TIM1.CounterMode": "TIM_COUNTERMODE_CENTERALIGNED1",
        "PA8.GPIO_Label": "PWM_A", "PA9.GPIO_Label": "PWM_B", "PA10.GPIO_Label": "PWM_C",
        "PA0.Signal": "ADC1_IN1", "PA1.Signal": "ADC2_IN2", "PB0.Signal": "ADC1_IN15",
        "PB1.Signal": "ADC1_IN12", "PC4.Signal": "ADC2_IN5",
        "PB10.GPIO_Label": "DRV_ENABLE", "PA15.GPIO_Label": "ENC_CS",
        "PC10.Signal": "SPI3_SCK", "PC11.Signal": "SPI3_MISO", "PB5.Signal": "SPI3_MOSI",
        "PB8.Signal": "FDCAN1_RX", "PB9.Signal": "FDCAN1_TX", "PC6.GPIO_Label": "STATUS_LED",
        "SPI3.DataSize": "SPI_DATASIZE_8BIT", "SPI3.CLKPhase": "SPI_PHASE_2EDGE",
        "SPI3.CLKPolarity": "SPI_POLARITY_LOW",
        "ADC1.NbrOfConversion": "3", "ADC2.NbrOfConversion": "3",
        "ADC1.Mode": "ADC_DUALMODE_REGSIMULT", "ADC1.DMAAccessMode": "ADC_DMAACCESSMODE_12_10_BITS",
        "Dma.ADC1.0.PeriphDataAlignment": "DMA_PDATAALIGN_WORD",
        "Dma.ADC1.0.MemDataAlignment": "DMA_MDATAALIGN_WORD",
        "Dma.ADC1.0.Mode": "DMA_CIRCULAR",
        "FDCAN1.FrameFormat": "FDCAN_FRAME_FD_BRS",
    }
    for key, value in expected.items():
        require(ioc.get(key) == value, f"{key}: expected {value}, got {ioc.get(key)}")
    for file, token in [
        ("Core/Src/main.c", "roller_app_start();"),
        ("Core/Src/main.c", "MX_ADC2_Init();"),
        ("Core/Src/main.c", "MX_SPI3_Init();"),
        ("Core/Src/stm32g4xx_it.c", "HAL_ADC_IRQHandler(&hadc2);"),
        ("Core/Src/stm32g4xx_it.c", "roller_app_encoder_trigger();"),
        ("Core/Src/adc.c", "ADC_DUALMODE_REGSIMULT"),
        ("Core/Src/adc.c", "DMA_MDATAALIGN_WORD"),
        ("Core/Src/spi.c", "SPI_DATASIZE_8BIT"),
        ("Core/Src/spi.c", "GPIO_AF6_SPI3"),
        ("Core/Src/tim.c", "sConfigOC.Pulse = 3920;"),
        ("Core/Src/tim.c", "#ifdef ROLLER_NO_MOTION"),
        ("Core/Src/fdcan.c", "GPIO_PIN_8|GPIO_PIN_9"),
        ("Platform/STM32/board.cpp", "HAL_ADCEx_MultiModeStart_DMA"),
        ("Platform/STM32/ssi_dma.cpp", "DMA_REQUEST_SPI3_RX"),
        ("Platform/STM32/pwm.cpp", "TIM_CR1_UDIS"),
        ("RTOS/FreeRTOSConfig.h", "#define configSUPPORT_DYNAMIC_ALLOCATION 0"),
        ("linker/STM32G431CB_FLASH.ld", "LENGTH = 124K"),
    ]:
        contains(file, token)
    main_source = (ROOT / "Core/Src/main.c").read_text()
    require(main_source.index("MX_DMA_Init();") < main_source.index("MX_ADC1_Init();"), "DMA must initialize before ADC")
    for obsolete in ("DRV_FAULT_Pin", "PWM_ENABLE_Pin", "CAN_STB_Pin", "USER_BUTTON_Pin"):
        require(obsolete not in (ROOT / "Core/Inc/main.h").read_text(), "obsolete pin " + obsolete)
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
            "roller_hardware",
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
        "PASS: peripheral source contract, interrupt ownership, local dependencies"
        + (", static-allocation ELF" if args.elf else "")
    )


if __name__ == "__main__":
    main()
