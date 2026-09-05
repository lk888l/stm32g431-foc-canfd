#!/usr/bin/env python3
"""Check regeneration contracts and optional ELF invariants, without MCU access."""
import argparse
import re
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def require(condition, message):
    if not condition:
        raise SystemExit("Project contract failed: " + message)


def contains(file, token):
    require(token in (ROOT / file).read_text(encoding="utf8"), f"{file}: missing {token}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--elf", type=Path)
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
        "PA8.GPIO_Label": "PWM_C",
        "PA9.GPIO_Label": "PWM_B",
        "PA10.GPIO_Label": "PWM_A",
        "PB0.GPIO_Label": "DRV_FAULT",
        "PB1.GPIO_Label": "DRV_ENABLE",
        "PB2.GPIO_Label": "PWM_ENABLE",
        "PB4.GPIO_Label": "CAN_STB",
        "PA4.GPIO_Label": "ENC_CS",
    }
    for key, value in expected.items():
        require(ioc.get(key) == value, f"{key}: expected {value}, got {ioc.get(key)}")
    for file, token in [
        ("Core/Src/main.c", "roller_app_start();"),
        ("Core/Src/stm32g4xx_it.c", "HAL_ADC_IRQHandler(&hadc1);"),
        ("Core/Src/stm32g4xx_it.c", "HAL_GPIO_EXTI_IRQHandler(DRV_FAULT_Pin);"),
        ("Core/Src/stm32g4xx_it.c", "HAL_FDCAN_IRQHandler(&hfdcan1);"),
        ("Core/Src/stm32g4xx_it.c", "HAL_TIM_IRQHandler(&htim6);"),
        ("Core/Src/adc.c", "ADC_EOC_SEQ_CONV"),
        ("Core/Src/adc.c", "ADC_CLOCK_SYNC_PCLK_DIV4"),
        ("Core/Src/adc.c", "ADC_EXTERNALTRIGINJEC_T1_TRGO"),
        ("Core/Src/adc.c", "hadc1.Init.DiscontinuousConvMode = ENABLE;"),
        ("Core/Src/adc.c", "ADC_CHANNEL_TEMPSENSOR_ADC1"),
        ("Core/Src/adc.c", "ADC_CHANNEL_VREFINT"),
        ("Core/Src/spi.c", "SPI_DATASIZE_16BIT"),
        ("Core/Src/spi.c", "SPI_PHASE_2EDGE"),
        ("Core/Src/tim.c", "sConfigOC.Pulse = 3680;"),
        ("Core/Src/fdcan.c", "FDCAN_FRAME_CLASSIC"),
        ("Core/Src/fdcan.c", "hfdcan1.Init.NominalPrescaler = 5;"),
        ("Core/Src/fdcan.c", "hfdcan1.Init.NominalTimeSeg1 = 25;"),
        ("Core/Src/fdcan.c", "hfdcan1.Init.NominalTimeSeg2 = 6;"),
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
    require(not (ROOT / ".gitmodules").exists(), "submodules are prohibited")
    for entry in (ROOT / "ThirdParty").rglob(".git"):
        raise SystemExit("Nested git checkout is prohibited: " + str(entry))
    if args.elf:
        output = subprocess.check_output(
            ["arm-none-eabi-nm", "--defined-only", str(args.elf)], text=True
        )
        symbols = {line.split()[-1] for line in output.splitlines() if line.split()}
        for symbol in (
            "ADC1_2_IRQHandler",
            "EXTI0_IRQHandler",
            "FDCAN1_IT0_IRQHandler",
            "SVC_Handler",
            "PendSV_Handler",
            "SysTick_Handler",
            "roller_app_start",
            "roller_debug",
        ):
            require(symbol in symbols, "missing ELF symbol " + symbol)
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
        require(not any(re.match(r"_Zn[aw]", s) for s in symbols), "C++ operator new linked")
    print(
        "PASS: CubeMX, interrupt ownership, local dependencies"
        + (", static-allocation ELF" if args.elf else "")
    )


if __name__ == "__main__":
    main()
