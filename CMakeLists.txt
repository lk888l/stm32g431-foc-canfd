cmake_minimum_required(VERSION 3.22)

project(brushless_control VERSION 0.1.0 LANGUAGES C ASM)

if(DEFINED CMAKE_TOOLCHAIN_FILE)
    # Reading the preset-provided value also prevents newer CMake versions
    # from reporting it as an unused command-line cache entry on reconfigure.
    message(STATUS "Toolchain file: ${CMAKE_TOOLCHAIN_FILE}")
endif()

include(CTest)

if(CMAKE_CROSSCOMPILING)
    set(_firmware_default ON)
    set(_tests_default OFF)
else()
    set(_firmware_default OFF)
    set(_tests_default ON)
endif()

option(BRUSHLESS_BUILD_FIRMWARE "Build the STM32G431CBU6 firmware image" ${_firmware_default})
option(BRUSHLESS_BUILD_TESTS "Build hardware-independent host unit tests" ${_tests_default})
set(BRUSHLESS_BOARD_PROFILE "PRODUCTION" CACHE STRING
    "Motor-control hardware profile: PRODUCTION or LAB")
set_property(CACHE BRUSHLESS_BOARD_PROFILE PROPERTY STRINGS PRODUCTION LAB)
option(BRUSHLESS_USE_8MHZ_HSE
    "Use the production 8 MHz crystal required for robust CAN-FD timing" ON)

string(TOUPPER "${BRUSHLESS_BOARD_PROFILE}" _brushless_board_profile)
if(NOT _brushless_board_profile STREQUAL "PRODUCTION" AND
   NOT _brushless_board_profile STREQUAL "LAB")
    message(FATAL_ERROR
        "BRUSHLESS_BOARD_PROFILE must be PRODUCTION or LAB, got: ${BRUSHLESS_BOARD_PROFILE}")
endif()

set(BRUSHLESS_PORTABLE_SOURCES
    App/control/angle_pll.c
    App/control/control_pi.c
    App/control/control_trajectory.c
    App/control/foc_math.c
    App/control/mit_controller.c
    App/motor/encoder_tracker.c
    App/motor/motor_foc.c
    App/drive/drive_mode.c
    App/drive/drive_pds.c
    App/protocol/cia402_od.c
    App/protocol/motor_vendor_canfd.c
    App/safety/drive_safety.c
)

set(BRUSHLESS_PORTABLE_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/App/control
    ${CMAKE_CURRENT_SOURCE_DIR}/App/drive
    ${CMAKE_CURRENT_SOURCE_DIR}/App/motor
    ${CMAKE_CURRENT_SOURCE_DIR}/App/protocol
    ${CMAKE_CURRENT_SOURCE_DIR}/App/safety
)

add_library(brushless_portable STATIC ${BRUSHLESS_PORTABLE_SOURCES})
target_compile_features(brushless_portable PUBLIC c_std_11)
target_include_directories(brushless_portable PUBLIC ${BRUSHLESS_PORTABLE_INCLUDE_DIRS})

if(BRUSHLESS_BUILD_TESTS)
    if(CMAKE_CROSSCOMPILING)
        message(FATAL_ERROR "BRUSHLESS_BUILD_TESTS requires a native compiler; configure the host preset separately")
    endif()

    if(MSVC)
        target_compile_options(brushless_portable PRIVATE /W4 /WX)
    else()
        target_compile_options(brushless_portable PRIVATE
            -Wall -Wextra -Wpedantic -Werror
            -Wconversion -Wdouble-promotion -Wshadow
        )
    endif()

    add_executable(test_control Tests/unit/test_control.c)
    target_link_libraries(test_control PRIVATE brushless_portable)
    if(NOT MSVC)
        target_link_libraries(test_control PRIVATE m)
    endif()
    add_test(NAME control COMMAND test_control)

    add_executable(test_drive_protocol Tests/unit/test_drive_protocol.c)
    target_link_libraries(test_drive_protocol PRIVATE brushless_portable)
    if(NOT MSVC)
        target_link_libraries(test_drive_protocol PRIVATE m)
    endif()
    add_test(NAME drive_protocol COMMAND test_drive_protocol)

    add_executable(test_motor_foc Tests/unit/test_motor_foc.c)
    target_link_libraries(test_motor_foc PRIVATE brushless_portable)
    if(NOT MSVC)
        target_link_libraries(test_motor_foc PRIVATE m)
    endif()
    add_test(NAME motor_foc COMMAND test_motor_foc)
endif()

if(BRUSHLESS_BUILD_FIRMWARE)
    if(NOT CMAKE_CROSSCOMPILING)
        message(FATAL_ERROR
            "The firmware target requires an Arm toolchain. Use the firmware preset "
            "or pass -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake")
    endif()

    set(STM32_HAL_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver)
    set(STM32_CMSIS_DEVICE_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32G4xx)
    set(STM32_CMSIS_CORE_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Core/CMSIS/Core)

    foreach(_required_path
            ${STM32_HAL_ROOT}/Inc/stm32g4xx_hal.h
            ${STM32_CMSIS_DEVICE_ROOT}/Include/stm32g431xx.h
            ${STM32_CMSIS_CORE_ROOT}/Include/core_cm4.h
            ${CMAKE_CURRENT_SOURCE_DIR}/STM32G431CBUX_FLASH.ld)
        if(NOT EXISTS ${_required_path})
            message(FATAL_ERROR
                "Required STM32 dependency is missing: ${_required_path}. "
                "Run 'git submodule update --init --recursive'.")
        endif()
    endforeach()

    set(STM32_HAL_SOURCES
        ${STM32_HAL_ROOT}/Src/stm32g4xx_hal.c
        ${STM32_HAL_ROOT}/Src/stm32g4xx_hal_adc.c
        ${STM32_HAL_ROOT}/Src/stm32g4xx_hal_adc_ex.c
        ${STM32_HAL_ROOT}/Src/stm32g4xx_hal_cortex.c
        ${STM32_HAL_ROOT}/Src/stm32g4xx_hal_dma.c
        ${STM32_HAL_ROOT}/Src/stm32g4xx_hal_dma_ex.c
        ${STM32_HAL_ROOT}/Src/stm32g4xx_hal_exti.c
        ${STM32_HAL_ROOT}/Src/stm32g4xx_hal_fdcan.c
        ${STM32_HAL_ROOT}/Src/stm32g4xx_hal_flash.c
        ${STM32_HAL_ROOT}/Src/stm32g4xx_hal_flash_ex.c
        ${STM32_HAL_ROOT}/Src/stm32g4xx_hal_flash_ramfunc.c
        ${STM32_HAL_ROOT}/Src/stm32g4xx_hal_gpio.c
        ${STM32_HAL_ROOT}/Src/stm32g4xx_hal_i2c.c
        ${STM32_HAL_ROOT}/Src/stm32g4xx_hal_i2c_ex.c
        ${STM32_HAL_ROOT}/Src/stm32g4xx_hal_iwdg.c
        ${STM32_HAL_ROOT}/Src/stm32g4xx_hal_pwr.c
        ${STM32_HAL_ROOT}/Src/stm32g4xx_hal_pwr_ex.c
        ${STM32_HAL_ROOT}/Src/stm32g4xx_hal_rcc.c
        ${STM32_HAL_ROOT}/Src/stm32g4xx_hal_rcc_ex.c
        ${STM32_HAL_ROOT}/Src/stm32g4xx_hal_spi.c
        ${STM32_HAL_ROOT}/Src/stm32g4xx_hal_spi_ex.c
        ${STM32_HAL_ROOT}/Src/stm32g4xx_hal_tim.c
        ${STM32_HAL_ROOT}/Src/stm32g4xx_hal_tim_ex.c
    )

    # Root-level integration and BSP sources are deliberately discovered so a
    # newly added app.c or transport adapter is picked up after CMake regenerates.
    file(GLOB BRUSHLESS_APP_INTEGRATION CONFIGURE_DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/App/*.c)
    file(GLOB BRUSHLESS_BSP_SOURCES CONFIGURE_DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/App/bsp/*.c)
    file(GLOB BRUSHLESS_STM32_PROTOCOL_SOURCES CONFIGURE_DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/App/protocol/*_stm32g4.c)

    set(CANOPENNODE_ROOT
        ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/CANopenNode)
    set(BRUSHLESS_CANOPEN_PORT_SOURCES
        App/canopen/CO_driver.c
        App/canopen/OD.c
        App/canopen/canopen_classic_od_adapter.c
        App/canopen/canopen_classic.c
    )
    set(CANOPENNODE_SOURCES
        ${CANOPENNODE_ROOT}/CANopen.c
        ${CANOPENNODE_ROOT}/301/CO_ODinterface.c
        ${CANOPENNODE_ROOT}/301/CO_Emergency.c
        ${CANOPENNODE_ROOT}/301/CO_NMT_Heartbeat.c
        ${CANOPENNODE_ROOT}/301/CO_HBconsumer.c
        ${CANOPENNODE_ROOT}/301/CO_SDOserver.c
        ${CANOPENNODE_ROOT}/301/CO_PDO.c
        ${CANOPENNODE_ROOT}/301/CO_fifo.c
    )

    foreach(_canopen_required
            ${CMAKE_CURRENT_SOURCE_DIR}/App/canopen/CO_driver_target.h
            ${CMAKE_CURRENT_SOURCE_DIR}/App/canopen/OD.h
            ${CANOPENNODE_ROOT}/CANopen.h
            ${CANOPENNODE_ROOT}/301/CO_driver.h)
        if(NOT EXISTS ${_canopen_required})
            message(FATAL_ERROR "Required CANopenNode integration file is missing: ${_canopen_required}")
        endif()
    endforeach()

    add_executable(brushless_control
        Core/Src/main.c
        Core/Src/syscalls.c
        Core/Src/stm32g4xx_hal_msp.c
        Core/Src/stm32g4xx_it.c
        ${STM32_CMSIS_DEVICE_ROOT}/Source/Templates/system_stm32g4xx.c
        ${STM32_CMSIS_DEVICE_ROOT}/Source/Templates/gcc/startup_stm32g431xx.s
        ${STM32_HAL_SOURCES}
        ${BRUSHLESS_APP_INTEGRATION}
        ${BRUSHLESS_BSP_SOURCES}
        ${BRUSHLESS_STM32_PROTOCOL_SOURCES}
        ${BRUSHLESS_CANOPEN_PORT_SOURCES}
        ${CANOPENNODE_SOURCES}
    )

    target_link_libraries(brushless_control PRIVATE brushless_portable m)
    target_compile_features(brushless_control PRIVATE c_std_11)
    target_compile_definitions(brushless_control PRIVATE USE_HAL_DRIVER STM32G431xx)
    if(_brushless_board_profile STREQUAL "PRODUCTION")
        target_compile_definitions(brushless_control PRIVATE
            BOARD_PROFILE=BOARD_PROFILE_PRODUCTION_3SHUNT_MT6701)
    else()
        target_compile_definitions(brushless_control PRIVATE
            BOARD_PROFILE=BOARD_PROFILE_LAB_SIMPLEFOC_AS5600)
    endif()
    if(BRUSHLESS_USE_8MHZ_HSE)
        target_compile_definitions(brushless_control PRIVATE BOARD_USE_8MHZ_HSE=1)
    else()
        message(FATAL_ERROR
            "CAN-FD is enabled in every firmware profile and requires "
            "BRUSHLESS_USE_8MHZ_HSE=ON for release-qualified clock accuracy")
    endif()
    target_include_directories(brushless_control PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/App/canopen
        ${CANOPENNODE_ROOT}
        ${CANOPENNODE_ROOT}/301
        ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc
        ${CMAKE_CURRENT_SOURCE_DIR}/App
        ${CMAKE_CURRENT_SOURCE_DIR}/App/bsp
        ${CMAKE_CURRENT_SOURCE_DIR}/App/motor
        ${CMAKE_CURRENT_SOURCE_DIR}/App/protocol
        ${STM32_HAL_ROOT}/Inc
        ${STM32_HAL_ROOT}/Inc/Legacy
        ${STM32_CMSIS_DEVICE_ROOT}/Include
        ${STM32_CMSIS_CORE_ROOT}/Include
    )

    # Upstream CANopenNode intentionally leaves a parameter unused in this
    # reduced stack configuration. Keep third-party warning policy local.
    set_source_files_properties(${CANOPENNODE_SOURCES} PROPERTIES
        COMPILE_OPTIONS "-Wno-unused-parameter")

    set(STM32_CPU_FLAGS
        -mcpu=cortex-m4
        -mthumb
        -mfpu=fpv4-sp-d16
        -mfloat-abi=hard
    )

    target_compile_options(brushless_portable PRIVATE
        ${STM32_CPU_FLAGS}
        -ffunction-sections -fdata-sections -fno-common
        -Wall -Wextra -Wpedantic
        $<$<CONFIG:Debug>:-Og;-g3>
        $<$<CONFIG:Release>:-O2;-g1>
        $<$<COMPILE_LANGUAGE:C>:-fstack-usage>
    )
    target_compile_options(brushless_control PRIVATE
        ${STM32_CPU_FLAGS}
        -ffunction-sections -fdata-sections -fno-common
        -Wall -Wextra
        $<$<CONFIG:Debug>:-Og;-g3>
        $<$<CONFIG:Release>:-O2;-g1>
        $<$<COMPILE_LANGUAGE:C>:-fstack-usage>
    )

    target_link_options(brushless_control PRIVATE
        ${STM32_CPU_FLAGS}
        -T${CMAKE_CURRENT_SOURCE_DIR}/STM32G431CBUX_FLASH.ld
        --specs=nano.specs
        --specs=nosys.specs
        -Wl,--gc-sections
        -Wl,--cref
        -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/brushless_control.map
        -Wl,--print-memory-usage
    )
    set_target_properties(brushless_control PROPERTIES OUTPUT_NAME brushless_control)

    add_custom_command(TARGET brushless_control POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex
            $<TARGET_FILE:brushless_control>
            ${CMAKE_CURRENT_BINARY_DIR}/brushless_control.hex
        COMMAND ${CMAKE_OBJCOPY} -O binary
            $<TARGET_FILE:brushless_control>
            ${CMAKE_CURRENT_BINARY_DIR}/brushless_control.bin
        COMMAND ${CMAKE_SIZE} --format=berkeley $<TARGET_FILE:brushless_control>
        BYPRODUCTS
            ${CMAKE_CURRENT_BINARY_DIR}/brushless_control.hex
            ${CMAKE_CURRENT_BINARY_DIR}/brushless_control.bin
        COMMENT "Generating HEX/BIN images and reporting firmware size"
    )
endif()

if(NOT BRUSHLESS_BUILD_FIRMWARE AND NOT BRUSHLESS_BUILD_TESTS)
    message(WARNING "Both firmware and test builds are disabled; no final targets will be built")
endif()
