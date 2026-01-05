cmake_minimum_required(VERSION 3.16)

project(MultiAxesControl VERSION 1.0.0 LANGUAGES C CXX ASM)

# Set C standard
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Platform detection
if(NOT DEFINED PLATFORM)
    set(PLATFORM "stm32" CACHE STRING "Target platform (arduino, stm32, esp32)")
endif()

# Platform detection
if(NOT DEFINED FWTYPE)
    set(FWTYPE "firmware" CACHE STRING "Target platform (firmware, blinky)")
endif()

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (Debug, Release, MinSizeRel, RelWithDebInfo)")
endif()

# Compiler flags
set(CMAKE_C_FLAGS_DEBUG "-g -O0 -DDEBUG")
set(CMAKE_C_FLAGS_RELEASE "-Os -DNDEBUG")
set(CMAKE_C_FLAGS_MINSIZEREL "-Os -DNDEBUG")
set(CMAKE_C_FLAGS_RELWITHDEBINFO "-O2 -g -DNDEBUG")

# Include directories
include_directories(
    .
    config
    hal
    hal/platforms/${PLATFORM}
    application
    os
)

# Minimal external deps via submodules
set(CMSIS_DEVICE_DIR ${CMAKE_CURRENT_LIST_DIR}/hal/third_party/cmsis_device_g4)
set(CMSIS_CORE_DIR   ${CMAKE_CURRENT_LIST_DIR}/hal/third_party/CMSIS_5/CMSIS/Core)
set(HAL_DIR          ${CMAKE_CURRENT_LIST_DIR}/hal/third_party/stm32g4xx_hal_driver)

# Platform-specific sources
if(PLATFORM STREQUAL "stm32")
    list(APPEND SOURCES
        hal/platforms/stm32/hal_stm32.c
        ${CMSIS_DEVICE_DIR}/Source/Templates/gcc/startup_stm32g474xx.s
        ${CMSIS_DEVICE_DIR}/Source/Templates/system_stm32g4xx.c
    )
    # CMSIS + HAL/LL includes
    include_directories(
        ${CMSIS_DEVICE_DIR}/Include
        ${CMSIS_CORE_DIR}/Include
        ${HAL_DIR}/Inc
    )
    # HAL/LL sources
    set(STM32_HAL_SOURCES
        ${HAL_DIR}/Src/stm32g4xx_hal.c
        ${HAL_DIR}/Src/stm32g4xx_hal_cortex.c
        ${HAL_DIR}/Src/stm32g4xx_hal_dma.c
        ${HAL_DIR}/Src/stm32g4xx_hal_dma_ex.c
        ${HAL_DIR}/Src/stm32g4xx_hal_exti.c
        ${HAL_DIR}/Src/stm32g4xx_hal_flash.c
        ${HAL_DIR}/Src/stm32g4xx_hal_flash_ex.c
        ${HAL_DIR}/Src/stm32g4xx_hal_gpio.c
        ${HAL_DIR}/Src/stm32g4xx_hal_hrtim.c
        ${HAL_DIR}/Src/stm32g4xx_hal_pwr.c
        ${HAL_DIR}/Src/stm32g4xx_hal_pwr_ex.c
        ${HAL_DIR}/Src/stm32g4xx_hal_rcc.c
        ${HAL_DIR}/Src/stm32g4xx_hal_rcc_ex.c
        ${HAL_DIR}/Src/stm32g4xx_hal_tim.c
        ${HAL_DIR}/Src/stm32g4xx_hal_tim_ex.c
        ${HAL_DIR}/Src/stm32g4xx_hal_uart.c
        ${HAL_DIR}/Src/stm32g4xx_hal_iwdg.c
    )
    set(STM32_LL_SOURCES
        ${HAL_DIR}/Src/stm32g4xx_ll_gpio.c
        ${HAL_DIR}/Src/stm32g4xx_ll_pwr.c
        ${HAL_DIR}/Src/stm32g4xx_ll_rcc.c
        ${HAL_DIR}/Src/stm32g4xx_ll_utils.c
    )
    list(APPEND SOURCES ${STM32_HAL_SOURCES} ${STM32_LL_SOURCES})
    # Toolchain and MCU flags
    set(CMAKE_C_COMPILER arm-none-eabi-gcc)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections -fno-builtin -Os -DUSE_HAL_DRIVER -DSTM32G474xx")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T${CMAKE_CURRENT_LIST_DIR}/hal/platforms/stm32/STM32G474RE_FLASH.ld -Wl,--gc-sections -Wl,-Map=${CMAKE_BINARY_DIR}/firmware.map")
endif()

# FreeRTOS kernel sources (upstream submodule)
set(FREERTOS_DIR ${CMAKE_CURRENT_LIST_DIR}/os/freertos/FreeRTOS-Kernel)
if(EXISTS ${FREERTOS_DIR})
    set(FREERTOS_SOURCE_DIR ${FREERTOS_DIR})
    set(FREERTOS_PORT_DIR ${FREERTOS_SOURCE_DIR}/portable/GCC/ARM_CM4F)
    set(FREERTOS_MEMMANG_DIR ${FREERTOS_SOURCE_DIR}/portable/MemMang)

    set(FREERTOS_SOURCES
        ${FREERTOS_SOURCE_DIR}/tasks.c
        ${FREERTOS_SOURCE_DIR}/queue.c
        ${FREERTOS_SOURCE_DIR}/list.c
        ${FREERTOS_SOURCE_DIR}/timers.c
        ${FREERTOS_SOURCE_DIR}/event_groups.c
        ${FREERTOS_SOURCE_DIR}/stream_buffer.c
        ${FREERTOS_PORT_DIR}/port.c
        ${FREERTOS_MEMMANG_DIR}/heap_4.c
    )
    add_library(freertos_kernel ${FREERTOS_SOURCES})
    target_include_directories(freertos_kernel PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/config
        ${FREERTOS_SOURCE_DIR}/include
        ${FREERTOS_PORT_DIR}
    )
endif()

# Create library
add_library(multi_axes_control ${SOURCES})

# Set target properties
set_target_properties(multi_axes_control PROPERTIES
    C_STANDARD 99
    C_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON
)

# Platform-specific configurations
if(PLATFORM STREQUAL "arduino")
    target_compile_definitions(multi_axes_control PRIVATE
        ARDUINO_PLATFORM=1
        F_CPU=16000000UL
    )
elseif(PLATFORM STREQUAL "stm32")
    target_compile_definitions(multi_axes_control PRIVATE
        STM32_PLATFORM=1
        STM32G4
    )
elseif(PLATFORM STREQUAL "esp32")
    target_compile_definitions(multi_axes_control PRIVATE
        ESP32_PLATFORM=1
        ESP32
    )
endif()

# Firmware executable (stm32)
if(PLATFORM STREQUAL "stm32")
    # Firmware executable (stm32)
    # NOTE: The existing hal_stm32.c has API incompatibilities with STM32G4
    # For now, firmware uses a simplified build without the custom HAL layer
    if(FWTYPE STREQUAL "firmware")
    add_executable(firmware
        firmware/src/main.c
        application/motor_control.c
        hal/platforms/stm32/hal_stm32.c
        os/task_manager.c
    )
    else()
    add_executable(${FWTYPE}
        examples/${FWTYPE}/src/main.c
    )
    endif()
    
    # Add HAL sources directly (same as blinky)
    target_sources(${FWTYPE} PRIVATE
        ${STM32_HAL_SOURCES}
        ${STM32_LL_SOURCES}
        ${CMSIS_DEVICE_DIR}/Source/Templates/gcc/startup_stm32g474xx.s
        ${CMSIS_DEVICE_DIR}/Source/Templates/system_stm32g4xx.c
    )
    
    target_include_directories(${FWTYPE} PRIVATE
        ${CMSIS_DEVICE_DIR}/Include
        ${CMSIS_CORE_DIR}/Include
        ${HAL_DIR}/Inc
        hal/platforms/stm32
    )
    
    target_compile_definitions(${FWTYPE} PRIVATE
        STM32_PLATFORM=1
        STM32G4
        USE_HAL_DRIVER
        STM32G474xx
    )
    
    # Link FreeRTOS
    if(TARGET freertos_kernel)
        target_link_libraries(${FWTYPE} freertos_kernel)
    endif()
    
    # Set output directory for firmware
    set_target_properties(${FWTYPE} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/executables
        SUFFIX ".elf"
    )

    # Link options for firmware
    target_link_options(${FWTYPE} PRIVATE 
        -Wl,--gc-sections 
        -Wl,-Map=${CMAKE_BINARY_DIR}/${OUTPUT_DIR}/${FWTYPE}.map
        -Wl,--print-memory-usage
    )

    # Helper function for post-build steps
    function(add_embedded_post_build TARGET_NAME)
        add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
            COMMAND arm-none-eabi-objcopy -O binary ${CMAKE_BINARY_DIR}/executables/${TARGET_NAME}.elf ${CMAKE_BINARY_DIR}/executables/${TARGET_NAME}.bin
            COMMAND arm-none-eabi-objcopy -O ihex ${CMAKE_BINARY_DIR}/executables/${TARGET_NAME}.elf ${CMAKE_BINARY_DIR}/executables/${TARGET_NAME}.hex
            COMMENT "Generating artifacts and analyzing memory usage for ${TARGET_NAME}..."
            WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        )
    endfunction()

    # Apply to both targets
    add_embedded_post_build(${FWTYPE})
endif()

# Installation
install(TARGETS multi_axes_control
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY src/
    DESTINATION include/multi_axes_control
    FILES_MATCHING PATTERN "*.h"
)

# Documentation
option(BUILD_DOCS "Build documentation" OFF)
if(BUILD_DOCS)
    find_package(Doxygen)
    if(DOXYGEN_FOUND)
        configure_file(${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile.in
                       ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
        add_custom_target(docs
            ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen" VERBATIM
        )
    endif()
endif()

# Testing
option(BUILD_TESTS "Build tests" OFF)
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# Summary
message(STATUS "MultiAxesControl Configuration:")
message(STATUS "  Platform: ${PLATFORM}")
message(STATUS "  Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  C compiler: ${CMAKE_C_COMPILER}")
message(STATUS "  C++ compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "  FreeRTOS: ${FREERTOS_DIR}")
