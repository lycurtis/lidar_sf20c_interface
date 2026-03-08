cmake_minimum_required(VERSION 3.22)

# Must be set before project()
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/cmake/arm-none-eabi-gcc.cmake)

project(lidar_sf20c_interface
    VERSION 1.0.0
    LANGUAGES C ASM
)

# MCU-specific compiler/linker flags
include(cmake/stm32f1.cmake)

# Executable name
set(TARGET ${PROJECT_NAME})

# Create executable
add_executable(${PROJECT_NAME}
    bsp/bsp.c
    drivers/mcu/usart.c
    drivers/devices/lidar.c
    app/main.c
    sys/interrupts.c
    sys/syscall.c
    device/startup_stm32f103xb.s
    device/system_stm32f1xx.c
)

target_include_directories(${PROJECT_NAME} PRIVATE
    bsp
    drivers/mcu
    drivers/devices
    app
    sys
    include
    CMSIS
    device
)

# Linker script
set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/device/STM32F103RBTx_FLASH.ld)
target_link_options(${TARGET} PRIVATE -T${LINKER_SCRIPT})

# Generate .hex and .bin files
add_custom_command(TARGET ${TARGET} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${TARGET}> ${TARGET}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${TARGET}> ${TARGET}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${TARGET}>
    COMMENT "Building ${TARGET}.hex and ${TARGET}.bin"
)