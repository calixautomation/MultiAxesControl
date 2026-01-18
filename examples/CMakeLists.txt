# =============================================================================
# Examples CMakeLists.txt
# =============================================================================
#
# This file includes all example projects.
#
# =============================================================================

# Only build examples for embedded platforms
if(NOT PLATFORM STREQUAL "stm32")
    message(STATUS "Skipping examples for platform: ${PLATFORM}")
    return()
endif()

# Include individual examples
add_subdirectory(blinky)
