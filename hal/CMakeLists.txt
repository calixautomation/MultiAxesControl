# =============================================================================
# HAL (Hardware Abstraction Layer) CMakeLists.txt
# =============================================================================
#
# This file defines the HAL interface library and includes platform-specific
# implementations based on the PLATFORM variable.
#
# =============================================================================

# HAL interface library (header-only interface)
add_library(hal_interface INTERFACE)

target_include_directories(hal_interface INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Include platform-specific implementation
if(PLATFORM STREQUAL "stm32")
    add_subdirectory(platforms/stm32)
elseif(PLATFORM STREQUAL "esp32")
    # add_subdirectory(platforms/esp32)
    message(WARNING "ESP32 platform not yet implemented")
elseif(PLATFORM STREQUAL "arduino")
    # add_subdirectory(platforms/arduino)
    message(WARNING "Arduino platform not yet implemented")
else()
    message(FATAL_ERROR "Unknown platform: ${PLATFORM}")
endif()
