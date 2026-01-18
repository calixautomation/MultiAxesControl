# =============================================================================
# OS (FreeRTOS) CMakeLists.txt
# =============================================================================
#
# This file defines the FreeRTOS kernel library and the task manager.
#
# =============================================================================

# -----------------------------------------------------------------------------
# FreeRTOS Kernel Library
# -----------------------------------------------------------------------------

if(EXISTS ${FREERTOS_DIR})
    # Port directory based on platform
    if(PLATFORM STREQUAL "stm32")
        set(FREERTOS_PORT_DIR ${FREERTOS_DIR}/portable/GCC/ARM_CM4F)
    elseif(PLATFORM STREQUAL "esp32")
        set(FREERTOS_PORT_DIR ${FREERTOS_DIR}/portable/ThirdParty/GCC/Xtensa_ESP32)
    else()
        message(FATAL_ERROR "FreeRTOS port not configured for platform: ${PLATFORM}")
    endif()

    set(FREERTOS_MEMMANG_DIR ${FREERTOS_DIR}/portable/MemMang)

    # FreeRTOS kernel sources
    set(FREERTOS_SOURCES
        ${FREERTOS_DIR}/tasks.c
        ${FREERTOS_DIR}/queue.c
        ${FREERTOS_DIR}/list.c
        ${FREERTOS_DIR}/timers.c
        ${FREERTOS_DIR}/event_groups.c
        ${FREERTOS_DIR}/stream_buffer.c
        ${FREERTOS_PORT_DIR}/port.c
        ${FREERTOS_MEMMANG_DIR}/heap_4.c
    )

    add_library(freertos_kernel STATIC ${FREERTOS_SOURCES})

    target_include_directories(freertos_kernel PUBLIC
        ${CMAKE_SOURCE_DIR}/config
        ${FREERTOS_DIR}/include
        ${FREERTOS_PORT_DIR}
    )

    # FreeRTOS needs CMSIS for STM32
    if(PLATFORM STREQUAL "stm32")
        target_include_directories(freertos_kernel PUBLIC
            ${CMSIS_DEVICE_DIR}/Include
            ${CMSIS_CORE_DIR}/Include
        )
        target_compile_definitions(freertos_kernel PUBLIC
            STM32G474xx
        )
    endif()

    message(STATUS "FreeRTOS: ${FREERTOS_DIR}")
else()
    message(WARNING "FreeRTOS not found at: ${FREERTOS_DIR}")
endif()

# -----------------------------------------------------------------------------
# Task Manager Library
# -----------------------------------------------------------------------------

add_library(task_manager STATIC
    task_manager.c
)

target_include_directories(task_manager PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Task manager depends on FreeRTOS and HAL
if(TARGET freertos_kernel)
    target_link_libraries(task_manager PUBLIC freertos_kernel)
endif()

target_link_libraries(task_manager PUBLIC hal_interface)

# Platform-specific HAL linkage
if(PLATFORM STREQUAL "stm32")
    target_link_libraries(task_manager PUBLIC hal_stm32)
endif()
