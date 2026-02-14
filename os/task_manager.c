/**
 * @file task_manager.c
 * @brief FreeRTOS task management implementation
 * @author MultiAxesControl
 * @date 2025
 */

#include "task_manager.h"
#include "../application/motor_control.h"
#include "../hal/hal_interface.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string.h>

// Configuration
#define UART_RX_BUFFER_SIZE     16
#define COMMAND_QUEUE_SIZE      10

// FreeRTOS handles
static QueueHandle_t g_command_queue = NULL;
static TaskHandle_t g_uart_task_handle = NULL;
static TaskHandle_t g_motor_task_handle = NULL;
static TaskHandle_t g_watchdog_task_handle = NULL;
static TaskHandle_t g_led_task_handle = NULL;

// Status flag
static bool g_initialized = false;

//=============================================================================
// Initialization
//=============================================================================

task_status_t task_manager_init(void) {
    if (g_initialized) {
        return TASK_OK;
    }

    // Create command queue
    g_command_queue = xQueueCreate(COMMAND_QUEUE_SIZE, UART_RX_BUFFER_SIZE);
    if (g_command_queue == NULL) {
        return TASK_ERROR;
    }

    // Create UART handler task (priority 2 - high)
    BaseType_t result = xTaskCreate(
        task_uart_handler,
        "UART",
        256,  // Stack size in words
        NULL,
        2,    // Priority
        &g_uart_task_handle
    );
    if (result != pdPASS) {
        return TASK_ERROR;
    }

    // Create motor update task (priority 1 - normal)
    result = xTaskCreate(
        task_motor_update,
        "MOTOR",
        256,
        NULL,
        1,
        &g_motor_task_handle
    );
    if (result != pdPASS) {
        return TASK_ERROR;
    }

    // Create watchdog refresh task (priority 3 - highest)
    result = xTaskCreate(
        task_watchdog_refresh,
        "WATCHDOG",
        128,
        NULL,
        3,
        &g_watchdog_task_handle
    );
    if (result != pdPASS) {
        return TASK_ERROR;
    }

    // Create status LED task (priority 1 - normal)
    result = xTaskCreate(
        task_status_led,
        "LED",
        128,
        NULL,
        1,
        &g_led_task_handle
    );
    if (result != pdPASS) {
        return TASK_ERROR;
    }

    g_initialized = true;
    return TASK_OK;
}

//=============================================================================
// Communication Functions
//=============================================================================

task_status_t task_manager_send_command(const char *command) {
    if (!command || !g_initialized) {
        return TASK_INVALID_PARAM;
    }

    // Send to queue (non-blocking from ISR context)
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (xQueueSendFromISR(g_command_queue, command, &xHigherPriorityTaskWoken) != pdTRUE) {
        return TASK_QUEUE_FULL;
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    return TASK_OK;
}

task_status_t task_manager_send_response(const char *response) {
    if (!response) {
        return TASK_INVALID_PARAM;
    }

    // Send via HAL UART
    hal_status_t status = hal_uart_send_string(response);
    return (status == CUSTOM_HAL_OK) ? TASK_OK : TASK_ERROR;
}

//=============================================================================
// UART ISR Handler
//=============================================================================

void task_manager_uart_rx_isr(void) {
    static char rx_buffer[UART_RX_BUFFER_SIZE];
    static uint8_t rx_index = 0;
    uint8_t ch;

    // Read character from HAL
    if (hal_uart_receive_byte(&ch) == CUSTOM_HAL_OK) {
        // Check for command terminator or buffer overflow
        if (ch == 'Z' || rx_index >= sizeof(rx_buffer) - 1) {
            rx_buffer[rx_index] = '\0';

            // Send command to queue
            //task_manager_send_command(rx_buffer);
            task_manager_send_response(rx_buffer);

            // Reset buffer
            rx_index = 0;
        } else {
            // Accumulate character
            rx_buffer[rx_index++] = ch;
        }
    }
}

//=============================================================================
// FreeRTOS Task Implementations
//=============================================================================

/**
 * @brief UART Handler Task
 * Processes commands from the command queue
 */
void task_uart_handler(void *pvParameters) {
    (void)pvParameters;

    char command_str[UART_RX_BUFFER_SIZE];
    motor_command_t command;

    while (1) {
        // Wait for command from queue
        if (xQueueReceive(g_command_queue, command_str, portMAX_DELAY) == pdTRUE) {
            // Parse command
            app_status_t status = motor_control_parse_command(command_str, &command);

            if (status == APP_OK) {
                // Process command
                status = motor_control_process_command(&command);

                if (status != APP_OK) {
                    task_manager_send_response("ERROR: Command execution failed\n");
                }
            } else {
                task_manager_send_response("ERROR: Invalid command format\n");
            }
        }
    }
}

/**
 * @brief Motor Update Task
 * Periodically updates motor states (for incremental mode completion check)
 */
void task_motor_update(void *pvParameters) {
    (void)pvParameters;

    while (1) {
        // Update motor states
        motor_control_update();

        // Update every 10ms
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief Watchdog Refresh Task
 * Refreshes the hardware watchdog to prevent system reset
 */
void task_watchdog_refresh(void *pvParameters) {
    (void)pvParameters;

    while (1) {
        // Refresh watchdog
        hal_watchdog_refresh();

        // Refresh every 500ms (watchdog timeout is 2s)
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/**
 * @brief Status LED Task
 * Blinks LED to indicate system is alive
 */
void task_status_led(void *pvParameters) {
    (void)pvParameters;

    while (1) {
        // Toggle LED (PA5)
        hal_gpio_toggle(5);

        // Toggle every 500ms (1Hz blink)
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
