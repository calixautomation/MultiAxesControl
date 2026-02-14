# =============================================================================
# Blinky Example CMakeLists.txt
# =============================================================================
#
# Simple LED blinky example to verify the build toolchain and hardware.
#
# =============================================================================

add_executable(blinky
    src/main.c
    ${STM32_IT_SOURCE}  # ISRs compiled directly for proper weak symbol override
)

# -----------------------------------------------------------------------------
# Link Libraries
# -----------------------------------------------------------------------------

target_link_libraries(blinky PRIVATE
    hal_stm32
)

# Link FreeRTOS if available
if(TARGET freertos_kernel)
    target_link_libraries(blinky PRIVATE freertos_kernel)
endif()

# -----------------------------------------------------------------------------
# Include Directories
# -----------------------------------------------------------------------------

target_include_directories(blinky PRIVATE
    ${CMAKE_SOURCE_DIR}/config
)

# -----------------------------------------------------------------------------
# Output Configuration
# -----------------------------------------------------------------------------

set_target_properties(blinky PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/executables
    SUFFIX ".elf"
)

# -----------------------------------------------------------------------------
# Linker Options
# -----------------------------------------------------------------------------

target_link_options(blinky PRIVATE
    -Wl,--gc-sections
    -Wl,-Map=${CMAKE_BINARY_DIR}/executables/blinky.map
    -Wl,--print-memory-usage
)

# -----------------------------------------------------------------------------
# Post-Build: Generate .bin and .hex
# -----------------------------------------------------------------------------

add_embedded_post_build(blinky)
