# =============================================================================
# Application CMakeLists.txt
# =============================================================================
#
# This file defines the application-level libraries including motor control
# and other business logic components.
#
# =============================================================================

# -----------------------------------------------------------------------------
# Motor Control Library
# -----------------------------------------------------------------------------

add_library(motor_control STATIC
    motor_control.c
)

target_include_directories(motor_control PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Motor control depends on HAL interface
target_link_libraries(motor_control PUBLIC hal_interface)

# Platform-specific HAL linkage
if(PLATFORM STREQUAL "stm32")
    target_link_libraries(motor_control PUBLIC hal_stm32)
endif()
