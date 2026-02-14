# =============================================================================
# STM32 HAL Platform Implementation CMakeLists.txt
# =============================================================================
#
# This file defines the STM32-specific HAL implementation including:
# - CMSIS Device and Core
# - STM32G4 HAL Drivers
# - Platform-specific HAL implementation
#
# =============================================================================

# -----------------------------------------------------------------------------
# STM32 HAL Implementation Library
# -----------------------------------------------------------------------------

add_library(hal_stm32 STATIC
    hal_stm32.c
)

# Export the path to stm32g4xx_it.c for targets to include directly
set(STM32_IT_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/stm32g4xx_it.c CACHE INTERNAL "STM32 ISR source file")

# -----------------------------------------------------------------------------
# Include Directories
# -----------------------------------------------------------------------------

target_include_directories(hal_stm32 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMSIS_DEVICE_DIR}/Include
    ${CMSIS_CORE_DIR}/Include
    ${HAL_DRIVER_DIR}/Inc
)

# -----------------------------------------------------------------------------
# STM32 HAL Driver Sources
# -----------------------------------------------------------------------------

set(STM32_HAL_SOURCES
    # Core HAL
    ${HAL_DRIVER_DIR}/Src/stm32g4xx_hal.c
    ${HAL_DRIVER_DIR}/Src/stm32g4xx_hal_cortex.c

    # Clock and Power
    ${HAL_DRIVER_DIR}/Src/stm32g4xx_hal_rcc.c
    ${HAL_DRIVER_DIR}/Src/stm32g4xx_hal_rcc_ex.c
    ${HAL_DRIVER_DIR}/Src/stm32g4xx_hal_pwr.c
    ${HAL_DRIVER_DIR}/Src/stm32g4xx_hal_pwr_ex.c
    ${HAL_DRIVER_DIR}/Src/stm32g4xx_hal_flash.c
    ${HAL_DRIVER_DIR}/Src/stm32g4xx_hal_flash_ex.c

    # Peripherals
    ${HAL_DRIVER_DIR}/Src/stm32g4xx_hal_gpio.c
    ${HAL_DRIVER_DIR}/Src/stm32g4xx_hal_dma.c
    ${HAL_DRIVER_DIR}/Src/stm32g4xx_hal_dma_ex.c
    ${HAL_DRIVER_DIR}/Src/stm32g4xx_hal_exti.c

    # Timers
    ${HAL_DRIVER_DIR}/Src/stm32g4xx_hal_tim.c
    ${HAL_DRIVER_DIR}/Src/stm32g4xx_hal_tim_ex.c
    ${HAL_DRIVER_DIR}/Src/stm32g4xx_hal_hrtim.c

    # Communication
    ${HAL_DRIVER_DIR}/Src/stm32g4xx_hal_uart.c

    # Watchdog
    ${HAL_DRIVER_DIR}/Src/stm32g4xx_hal_iwdg.c
)

# -----------------------------------------------------------------------------
# STM32 LL Driver Sources (Optional low-level drivers)
# -----------------------------------------------------------------------------

set(STM32_LL_SOURCES
    ${HAL_DRIVER_DIR}/Src/stm32g4xx_ll_gpio.c
    ${HAL_DRIVER_DIR}/Src/stm32g4xx_ll_pwr.c
    ${HAL_DRIVER_DIR}/Src/stm32g4xx_ll_rcc.c
    ${HAL_DRIVER_DIR}/Src/stm32g4xx_ll_utils.c
)

# -----------------------------------------------------------------------------
# CMSIS Startup and System Sources
# -----------------------------------------------------------------------------

set(CMSIS_SOURCES
    ${CMSIS_DEVICE_DIR}/Source/Templates/gcc/startup_stm32g474xx.s
    ${CMSIS_DEVICE_DIR}/Source/Templates/system_stm32g4xx.c
)

# Add all sources to the library
target_sources(hal_stm32 PRIVATE
    ${STM32_HAL_SOURCES}
    ${STM32_LL_SOURCES}
    ${CMSIS_SOURCES}
)

# -----------------------------------------------------------------------------
# Compile Definitions
# -----------------------------------------------------------------------------

target_compile_definitions(hal_stm32 PUBLIC
    USE_HAL_DRIVER
    STM32G474xx
    STM32G4
    STM32_PLATFORM=1
)

# -----------------------------------------------------------------------------
# Link HAL Interface
# -----------------------------------------------------------------------------

target_link_libraries(hal_stm32 PUBLIC
    hal_interface
)

# -----------------------------------------------------------------------------
# Link FreeRTOS (for ISR integration)
# -----------------------------------------------------------------------------

if(TARGET freertos_kernel)
    target_link_libraries(hal_stm32 PUBLIC freertos_kernel)
endif()
