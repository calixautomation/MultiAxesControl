/**
 * @file register_config.h
 * @brief Generic register configuration system for multi-platform support
 * @author MultiAxesControl
 * @date 2025
 */

#ifndef REGISTER_CONFIG_H
#define REGISTER_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Generic register access structure
 * This structure provides a platform-agnostic way to access hardware registers
 */
typedef struct {
    volatile uint32_t *base_addr;    ///< Base address of the peripheral
    uint32_t offset;                 ///< Register offset from base address
    uint32_t mask;                   ///< Bit mask for the register
    uint32_t shift;                  ///< Bit shift for the field
} register_config_t;

/**
 * @brief Timer configuration structure
 * Generic timer configuration that can be mapped to any platform
 */
typedef struct {
    register_config_t control_reg;       ///< Timer control register
    register_config_t prescaler_reg;     ///< Prescaler register
    register_config_t compare_reg;       ///< Compare register
    register_config_t counter_reg;       ///< Counter register
    register_config_t interrupt_reg;     ///< Interrupt enable register
    uint32_t prescaler_value;            ///< Prescaler value
    uint32_t compare_value;              ///< Compare value
    bool interrupt_enable;               ///< Interrupt enable flag
} timer_config_t;

/**
 * @brief UART configuration structure
 * Generic UART configuration for serial communication
 */
typedef struct {
    register_config_t control_reg;       ///< UART control register
    register_config_t baud_reg;          ///< Baud rate register
    register_config_t status_reg;        ///< Status register
    register_config_t data_reg;          ///< Data register
    register_config_t interrupt_reg;     ///< Interrupt enable register
    uint32_t baud_rate;                  ///< Baud rate value
    uint8_t data_bits;                   ///< Data bits (7, 8, 9)
    uint8_t stop_bits;                   ///< Stop bits (1, 2)
    uint8_t parity;                      ///< Parity (0=none, 1=odd, 2=even)
    bool interrupt_enable;               ///< Interrupt enable flag
} uart_config_t;

/**
 * @brief GPIO configuration structure
 * Generic GPIO configuration for pin control
 */
typedef struct {
    register_config_t direction_reg;     ///< Direction register
    register_config_t output_reg;        ///< Output register
    register_config_t input_reg;         ///< Input register
    register_config_t pullup_reg;        ///< Pull-up register
    uint8_t pin_number;                  ///< Pin number
    bool is_output;                      ///< Output mode flag
    bool pullup_enable;                  ///< Pull-up enable flag
} gpio_config_t;

/**
 * @brief Motor configuration structure
 * Configuration for stepper motor control
 */
typedef struct {
    gpio_config_t step_pin;              ///< Step pin configuration
    gpio_config_t dir_pin;               ///< Direction pin configuration
    gpio_config_t enable_pin;            ///< Enable pin configuration
    timer_config_t timer;                ///< Timer configuration for step generation
    uint32_t max_frequency;              ///< Maximum step frequency
    uint32_t microsteps;                 ///< Microstepping value
    float steps_per_mm;                  ///< Steps per millimeter
} motor_config_t;

/**
 * @brief System configuration structure
 * Main system configuration container
 */
typedef struct {
    motor_config_t motors[2];            ///< Motor configurations (linear, rotary)
    uart_config_t uart;                  ///< UART configuration
    uint32_t system_clock;               ///< System clock frequency
    uint32_t watchdog_timeout;           ///< Watchdog timeout in ms
} system_config_t;

/**
 * @brief Register access macros
 * These macros provide generic register access functionality
 */
#define REG_READ(reg)           (*(reg.base_addr + reg.offset))
#define REG_WRITE(reg, value)   (*(reg.base_addr + reg.offset) = (value))
#define REG_SET_BITS(reg, bits) (*(reg.base_addr + reg.offset) |= (bits))
#define REG_CLEAR_BITS(reg, bits) (*(reg.base_addr + reg.offset) &= ~(bits))
#define REG_READ_FIELD(reg)     ((REG_READ(reg) & reg.mask) >> reg.shift)
#define REG_WRITE_FIELD(reg, value) do { \
    uint32_t temp = REG_READ(reg); \
    temp &= ~reg.mask; \
    temp |= ((value) << reg.shift) & reg.mask; \
    REG_WRITE(reg, temp); \
} while(0)

/**
 * @brief Platform-specific register mapping functions
 * These functions must be implemented for each platform
 */
void platform_register_init(void);
void platform_timer_init(const timer_config_t *config);
void platform_uart_init(const uart_config_t *config);
void platform_gpio_init(const gpio_config_t *config);
void platform_motor_init(const motor_config_t *config);

/**
 * @brief Interrupt handler registration
 */
typedef void (*interrupt_handler_t)(void);
void platform_register_interrupt(uint32_t irq_number, interrupt_handler_t handler);

#ifdef __cplusplus
}
#endif

#endif /* REGISTER_CONFIG_H */
