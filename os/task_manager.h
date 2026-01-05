/**
 * @file task_manager.h
 * @brief FreeRTOS task management layer
 * @author MultiAxesControl
 * @date 2025
 *
 * This layer handles all RTOS-specific functionality including:
 * - Task creation and management
 * - Queue and semaphore handling
 * - ISR integration
 * - UART command processing tasks
 */

#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Task manager status codes
 */
typedef enum {
    TASK_OK = 0,
    TASK_ERROR,
    TASK_QUEUE_FULL,
    TASK_INVALID_PARAM
} task_status_t;

/**
 * @brief Initialize task manager and create all FreeRTOS tasks
 * This should be called after motor_control_init() but before vTaskStartScheduler()
 */
task_status_t task_manager_init(void);

/**
 * @brief Send command string to command queue
 * This is called from UART ISR when a complete command is received
 */
task_status_t task_manager_send_command(const char *command);

/**
 * @brief Send response string via UART
 * Thread-safe wrapper for UART transmission
 */
task_status_t task_manager_send_response(const char *response);

/**
 * @brief UART RX ISR Handler
 * Call this from USART2_IRQHandler (or equivalent UART ISR)
 */
void task_manager_uart_rx_isr(void);

/**
 * @brief FreeRTOS Task Functions
 * These are created automatically by task_manager_init()
 * They should NOT be called directly.
 */
void task_uart_handler(void *pvParameters);
void task_motor_update(void *pvParameters);
void task_watchdog_refresh(void *pvParameters);
void task_status_led(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif /* TASK_MANAGER_H */
