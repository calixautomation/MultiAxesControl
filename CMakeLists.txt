cmake_minimum_required(VERSION 3.16)

project(MultiAxesControl VERSION 1.0.0 LANGUAGES C CXX ASM)

# =============================================================================
# Project Configuration
# =============================================================================

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Platform selection
set(PLATFORM "stm32" CACHE STRING "Target platform (stm32, esp32, arduino)")
set_property(CACHE PLATFORM PROPERTY STRINGS stm32 esp32 arduino)

# Firmware type selection
set(FWTYPE "firmware" CACHE STRING "Firmware to build (firmware, blinky)")

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type")
endif()

# Compiler optimization flags
set(CMAKE_C_FLAGS_DEBUG "-g -O0 -DDEBUG")
set(CMAKE_C_FLAGS_RELEASE "-Os -DNDEBUG")
set(CMAKE_C_FLAGS_MINSIZEREL "-Os -DNDEBUG")
set(CMAKE_C_FLAGS_RELWITHDEBINFO "-O2 -g -DNDEBUG")

# =============================================================================
# Third-Party Dependencies Paths
# =============================================================================

set(CMSIS_DEVICE_DIR ${CMAKE_SOURCE_DIR}/hal/cmsis_device_g4 CACHE PATH "CMSIS Device")
set(CMSIS_CORE_DIR ${CMAKE_SOURCE_DIR}/hal/CMSIS_5/CMSIS/Core CACHE PATH "CMSIS Core")
set(HAL_DRIVER_DIR ${CMAKE_SOURCE_DIR}/hal/platforms/stm32/stm32g4xx_hal_driver CACHE PATH "STM32 HAL Driver")
set(FREERTOS_DIR ${CMAKE_SOURCE_DIR}/os/FreeRTOS-Kernel CACHE PATH "FreeRTOS Kernel")

# =============================================================================
# Platform-Specific Toolchain Configuration
# =============================================================================

if(PLATFORM STREQUAL "stm32")
    set(CMAKE_C_COMPILER arm-none-eabi-gcc)
    set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
    set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
    set(CMAKE_SIZE arm-none-eabi-size)

    # Cortex-M4F flags
    set(MCU_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${MCU_FLAGS} -ffunction-sections -fdata-sections -fno-builtin")
    set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} ${MCU_FLAGS}")

    # Linker script
    set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/hal/platforms/stm32/STM32G474RE_FLASH.ld)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T${LINKER_SCRIPT} -Wl,--gc-sections -specs=nosys.specs -specs=nano.specs")
endif()

# =============================================================================
# Helper Functions
# =============================================================================

# Post-build function to generate .bin and .hex files
function(add_embedded_post_build TARGET_NAME)
    add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${TARGET_NAME}> ${CMAKE_BINARY_DIR}/executables/${TARGET_NAME}.bin
        COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${TARGET_NAME}> ${CMAKE_BINARY_DIR}/executables/${TARGET_NAME}.hex
        COMMENT "Generating .bin and .hex for ${TARGET_NAME}"
    )
endfunction()

# =============================================================================
# Add Subdirectories
# =============================================================================

# Operating System (FreeRTOS)
add_subdirectory(os)

# Hardware Abstraction Layer
add_subdirectory(hal)

# Application Logic
add_subdirectory(application)

# Firmware Executables
add_subdirectory(firmware)

# Examples
add_subdirectory(examples)

# =============================================================================
# Testing (Optional)
# =============================================================================

option(BUILD_TESTS "Build tests" OFF)
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# =============================================================================
# Documentation (Optional)
# =============================================================================

option(BUILD_DOCS "Build documentation" OFF)
if(BUILD_DOCS)
    find_package(Doxygen)
    if(DOXYGEN_FOUND)
        configure_file(${CMAKE_SOURCE_DIR}/docs/Doxyfile.in
                       ${CMAKE_BINARY_DIR}/Doxyfile @ONLY)
        add_custom_target(docs
            ${DOXYGEN_EXECUTABLE} ${CMAKE_BINARY_DIR}/Doxyfile
            WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen"
        )
    endif()
endif()

# =============================================================================
# Configuration Summary
# =============================================================================

message(STATUS "")
message(STATUS "MultiAxesControl Configuration:")
message(STATUS "  Platform:    ${PLATFORM}")
message(STATUS "  Firmware:    ${FWTYPE}")
message(STATUS "  Build type:  ${CMAKE_BUILD_TYPE}")
message(STATUS "  C compiler:  ${CMAKE_C_COMPILER}")
message(STATUS "")
