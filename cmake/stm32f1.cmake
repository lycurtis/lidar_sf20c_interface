# cmake/stm32f1.cmake
# STM32F103RB Cortex-M3 specific flags

# MCU definition for headers
add_compile_definitions(STM32F103xB)

# CPU flags for Cortex-M3
set(CPU_FLAGS "-mcpu=cortex-m3 -mthumb")

# C flags
set(CMAKE_C_FLAGS "${CPU_FLAGS} -Wall -Wextra -fdata-sections -ffunction-sections")
set(CMAKE_C_FLAGS_DEBUG "-Og -g3 -DDEBUG")
set(CMAKE_C_FLAGS_RELEASE "-O2 -DNDEBUG")

# ASM flags
set(CMAKE_ASM_FLAGS "${CPU_FLAGS} -x assembler-with-cpp")

# Linker flags
set(CMAKE_EXE_LINKER_FLAGS "${CPU_FLAGS} -specs=nano.specs -specs=nosys.specs -Wl,--gc-sections -Wl,-Map=${PROJECT_NAME}.map")