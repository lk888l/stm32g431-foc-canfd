set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(TOOLCHAIN_PREFIX "arm-none-eabi-" CACHE STRING "GNU Arm Embedded tool prefix")
set(ARM_GNU_TOOLCHAIN_ROOT "" CACHE PATH
    "Optional Arm GNU Toolchain installation root (the directory containing bin)")

set(_arm_toolchain_hints)
if(ARM_GNU_TOOLCHAIN_ROOT)
    list(APPEND _arm_toolchain_hints "${ARM_GNU_TOOLCHAIN_ROOT}/bin")
elseif(DEFINED ENV{ARM_GNU_TOOLCHAIN_ROOT})
    list(APPEND _arm_toolchain_hints "$ENV{ARM_GNU_TOOLCHAIN_ROOT}/bin")
endif()

find_program(CMAKE_C_COMPILER
    NAMES ${TOOLCHAIN_PREFIX}gcc
    HINTS ${_arm_toolchain_hints}
    REQUIRED)
find_program(CMAKE_ASM_COMPILER
    NAMES ${TOOLCHAIN_PREFIX}gcc
    HINTS ${_arm_toolchain_hints}
    REQUIRED)
find_program(CMAKE_OBJCOPY
    NAMES ${TOOLCHAIN_PREFIX}objcopy
    HINTS ${_arm_toolchain_hints}
    REQUIRED)
find_program(CMAKE_SIZE
    NAMES ${TOOLCHAIN_PREFIX}size
    HINTS ${_arm_toolchain_hints}
    REQUIRED)

set(CMAKE_C_COMPILER_ID GNU)
set(CMAKE_ASM_COMPILER_ID GNU)
set(CMAKE_EXECUTABLE_SUFFIX_C .elf)
set(CMAKE_EXECUTABLE_SUFFIX_ASM .elf)

unset(_arm_toolchain_hints)
