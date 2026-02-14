# =============================================================================
# Firmware CMakeLists.txt
# =============================================================================
#
# This file defines the main firmware executable that runs on the target
# microcontroller.
#
# =============================================================================

# Only build firmware for embedded platforms
if(NOT PLATFORM STREQUAL "stm32")
    message(STATUS "Skipping firmware build for platform: ${PLATFORM}")
    return()
endif()

# -----------------------------------------------------------------------------
# Firmware Executable
# -----------------------------------------------------------------------------

add_executable(firmware
    src/main.c
    ${STM32_IT_SOURCE}  # ISRs compiled directly with firmware for proper weak symbol override
)

# -----------------------------------------------------------------------------
# Link Libraries
# -----------------------------------------------------------------------------

target_link_libraries(firmware PRIVATE
    hal_stm32
    motor_control
    task_manager
)

# Link FreeRTOS if available
if(TARGET freertos_kernel)
    target_link_libraries(firmware PRIVATE freertos_kernel)
endif()

# -----------------------------------------------------------------------------
# Include Directories
# -----------------------------------------------------------------------------

target_include_directories(firmware PRIVATE
    ${CMAKE_SOURCE_DIR}/config
    ${CMAKE_SOURCE_DIR}/hal
    ${CMAKE_SOURCE_DIR}/os
    ${CMAKE_SOURCE_DIR}/application
)

# -----------------------------------------------------------------------------
# Output Configuration
# -----------------------------------------------------------------------------

set_target_properties(firmware PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/executables
    SUFFIX ".elf"
)

# -----------------------------------------------------------------------------
# Linker Options
# -----------------------------------------------------------------------------

target_link_options(firmware PRIVATE
    -Wl,--gc-sections
    -Wl,-Map=${CMAKE_BINARY_DIR}/executables/firmware.map
    -Wl,--print-memory-usage
)

# -----------------------------------------------------------------------------
# Post-Build: Generate .bin and .hex
# -----------------------------------------------------------------------------

add_embedded_post_build(firmware)
