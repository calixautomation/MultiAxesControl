/**
 * @file hal_interface.h
 * @brief Hardware Abstraction Layer interface
 * @author MultiAxesControl
 * @date 2025
 */

#ifndef HAL_INTERFACE_H
#define HAL_INTERFACE_H

#include <stdint.h>
#include <stdbool.h>
#include "../config/register_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Custom HAL return codes
 * Note: Uses CUSTOM_ prefix to avoid conflicts with STM32 HAL
 */
typedef enum {
    CUSTOM_HAL_OK = 0,
    CUSTOM_HAL_ERROR,
    CUSTOM_HAL_BUSY,
    CUSTOM_HAL_TIMEOUT,
    CUSTOM_HAL_INVALID_PARAM
} hal_status_t;

/**
 * @brief Import motor types from application layer
 * HAL needs these for motor control functions
 */
#ifndef MOTOR_TYPES_DEFINED
#define MOTOR_TYPES_DEFINED

typedef enum {
    MOTOR_DIR_FORWARD = 0,
    MOTOR_DIR_REVERSE = 1
} motor_direction_t;

typedef enum {
    MOTOR_LINEAR = 0,
    MOTOR_ROTARY = 1,
    MOTOR_MAX = 2
} motor_id_t;

typedef enum {
    MOTOR_MODE_STOP = 0,
    MOTOR_MODE_JOG,
    MOTOR_MODE_INCREMENTAL,
    MOTOR_MODE_CYCLIC
} motor_mode_t;

#endif /* MOTOR_TYPES_DEFINED */

/**
 * @brief UART callback function type
 */
typedef void (*uart_rx_callback_t)(uint8_t data);
typedef void (*uart_tx_callback_t)(void);

/**
 * @brief Timer callback function type
 */
typedef void (*timer_callback_t)(void);

/**
 * @brief Motor control structure
 */
typedef struct {
    motor_id_t id;
    motor_mode_t mode;
    motor_direction_t direction;
    uint32_t step_count;
    uint32_t target_steps;
    bool is_running;
    timer_callback_t step_callback;
} motor_control_t;

/**
 * @brief HAL initialization functions
 */
hal_status_t hal_init(void);
hal_status_t hal_deinit(void);

/**
 * @brief Timer HAL functions
 */
hal_status_t hal_timer_init(const timer_config_t *config);
hal_status_t hal_timer_start(uint8_t timer_id);
hal_status_t hal_timer_stop(uint8_t timer_id);
hal_status_t hal_timer_set_frequency(uint8_t timer_id, uint32_t frequency);
hal_status_t hal_timer_register_callback(uint8_t timer_id, timer_callback_t callback);

/**
 * @brief UART HAL functions
 */
hal_status_t hal_uart_init(const uart_config_t *config);
hal_status_t hal_uart_send_byte(uint8_t data);
hal_status_t hal_uart_send_string(const char *str);
hal_status_t hal_uart_receive_byte(uint8_t *data);
hal_status_t hal_uart_register_rx_callback(uart_rx_callback_t callback);
hal_status_t hal_uart_register_tx_callback(uart_tx_callback_t callback);

/**
 * @brief GPIO HAL functions
 */
hal_status_t hal_gpio_init(const gpio_config_t *config);
hal_status_t hal_gpio_write(uint8_t pin, bool state);
hal_status_t hal_gpio_read(uint8_t pin, bool *state);
hal_status_t hal_gpio_toggle(uint8_t pin);

/**
 * @brief Motor HAL functions
 */
hal_status_t hal_motor_init(const motor_config_t *config, motor_id_t motor_id);
hal_status_t hal_motor_set_direction(motor_id_t motor_id, motor_direction_t direction);
hal_status_t hal_motor_start(motor_id_t motor_id);
hal_status_t hal_motor_stop(motor_id_t motor_id);
hal_status_t hal_motor_step(motor_id_t motor_id);
hal_status_t hal_motor_set_mode(motor_id_t motor_id, motor_mode_t mode);
hal_status_t hal_motor_get_status(motor_id_t motor_id, motor_control_t *status);

/**
 * @brief System HAL functions
 */
hal_status_t hal_system_reset(void);
hal_status_t hal_watchdog_init(uint32_t timeout_ms);
hal_status_t hal_watchdog_refresh(void);
hal_status_t hal_delay_ms(uint32_t delay);
hal_status_t hal_delay_us(uint32_t delay);

/**
 * @brief Interrupt HAL functions
 */
hal_status_t hal_interrupt_enable(uint32_t irq_number);
hal_status_t hal_interrupt_disable(uint32_t irq_number);
hal_status_t hal_interrupt_register_handler(uint32_t irq_number, void (*handler)(void));

#ifdef __cplusplus
}
#endif

#endif /* HAL_INTERFACE_H */
