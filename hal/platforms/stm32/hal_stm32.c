/**
 * @file hal_stm32.c
 * @brief STM32G474RE HAL implementation using STM32 HAL drivers
 * @author MultiAxesControl
 * @date 2025
 *
 * @details This file implements the hardware abstraction layer for STM32G474RE
 *          using official STM32 HAL drivers for consistency and maintainability.
 *
 * Hardware Configuration:
 * - MCU: STM32G474RE (Cortex-M4F @ 170MHz)
 * - Motor Control:
 *   - Linear Motor: TIM2 for step generation, PA0=STEP, PA1=DIR
 *   - Rotary Motor: TIM3 for step generation, PA2=STEP, PA3=DIR
 * - Communication: USART1 @ 115200 baud for PC communication
 * - Watchdog: IWDG with configurable timeout
 * - PWM: HRTIM for high-resolution PWM (configured separately)
 */

#include "hal_stm32.h"
#include "stm32g4xx_hal.h"
#include "../../hal_interface.h"
#include "../../../config/register_config.h"
#include <string.h>

/* ============================================================================
 * PRIVATE TYPES AND DEFINITIONS
 * ============================================================================ */

/**
 * @brief HAL handles for STM32 peripherals
 */
static TIM_HandleTypeDef htim2;          // Timer 2 for linear motor
static TIM_HandleTypeDef htim3;          // Timer 3 for rotary motor
static UART_HandleTypeDef huart1;        // UART1 for communication
static IWDG_HandleTypeDef hiwdg;         // Independent watchdog

extern UART_HandleTypeDef huart2;        // UART1 for communication

/**
 * @brief Motor control state
 */
static motor_control_t g_motors[MOTOR_MAX];

/**
 * @brief Registered callbacks
 */
static timer_callback_t g_timer_callbacks[2] = {NULL, NULL};

/**
 * @brief Initialization flags
 */
static bool g_watchdog_initialized = false;

/**
 * @brief UART receive buffer for HAL_UART_Receive_IT (single byte mode)
 */
static uint8_t g_uart_rx_byte;

// External callback from task_manager
extern void task_manager_uart_rx_isr(void) __attribute__((weak));

/* ============================================================================
 * PRIVATE FUNCTION PROTOTYPES
 * ============================================================================ */

/* ============================================================================
 * GENERAL HAL FUNCTIONS
 * ============================================================================ */

/**
 * @brief Initialize the HAL layer
 * @return CUSTOM_HAL_OK on success, CUSTOM_HAL_ERROR on failure
 *
 * @note This function initializes:
 *       - Motor control structures
 *       - GPIO pins for motor control
 *       - Does NOT start timers or UART (use specific init functions)
 */
hal_status_t hal_init(void) {
    gpio_config_t gpioConfig = {0U};
    uart_config_t uartConfig = {0U};

    // Initialize motor control structures
    memset(g_motors, 0, sizeof(g_motors));
    g_motors[MOTOR_LINEAR].id = MOTOR_LINEAR;
    g_motors[MOTOR_ROTARY].id = MOTOR_ROTARY;

    gpioConfig.pin_number = 0U;
    gpioConfig.is_output = true;
    gpioConfig.pullup_enable = false;

    uartConfig.baud_rate = 9600U;
    uartConfig.data_bits = 8U;
    uartConfig.stop_bits = 1U;
    uartConfig.parity = 2U; // even
    uartConfig.interrupt_enable = true;

    // Initialize peripherals
    hal_gpio_init(&gpioConfig);
    hal_uart_init(&uartConfig);

    return CUSTOM_HAL_OK;
}

/**
 * @brief Deinitialize the HAL layer
 * @return CUSTOM_HAL_OK on success
 *
 * @note Stops all timers, closes UART, and resets GPIO
 */
hal_status_t hal_deinit(void) {

    // Stop all timers
    HAL_TIM_Base_Stop_IT(&htim2);
    HAL_TIM_Base_Stop_IT(&htim3);

    // Deinitialize peripherals
    HAL_TIM_Base_DeInit(&htim2);
    HAL_TIM_Base_DeInit(&htim3);
    HAL_UART_DeInit(&huart1);

    // Reset GPIO pins
    HAL_GPIO_DeInit(GPIOA,  GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_5);

    return CUSTOM_HAL_OK;
}

/* ============================================================================
 * TIMER FUNCTIONS (Motor Step Generation)
 * ============================================================================ */

/**
 * @brief Start a timer (begins generating motor steps)
 * @param timer_id Timer identifier (0=TIM2/Linear, 1=TIM3/Rotary)
 * @return CUSTOM_HAL_OK on success, CUSTOM_HAL_INVALID_PARAM if invalid timer
 *
 * @warning Timer must be initialized first via hal_timer_set_frequency()
 */
hal_status_t hal_timer_start(uint8_t timer_id) {
    HAL_StatusTypeDef status;

    if (timer_id == 0) {
        // Start TIM2 for linear motor
        status = HAL_TIM_Base_Start_IT(&htim2);
    } else if (timer_id == 1) {
        // Start TIM3 for rotary motor
        status = HAL_TIM_Base_Start_IT(&htim3);
    } else {
        return CUSTOM_HAL_INVALID_PARAM;
    }

    return (status == HAL_OK) ? CUSTOM_HAL_OK : CUSTOM_HAL_ERROR;
}

/**
 * @brief Stop a timer (stops generating motor steps)
 * @param timer_id Timer identifier (0=TIM2/Linear, 1=TIM3/Rotary)
 * @return CUSTOM_HAL_OK on success, CUSTOM_HAL_INVALID_PARAM if invalid timer
 */
hal_status_t hal_timer_stop(uint8_t timer_id) {
    HAL_StatusTypeDef status;

    if (timer_id == 0) {
        // Stop TIM2 for linear motor
        status = HAL_TIM_Base_Stop_IT(&htim2);
    } else if (timer_id == 1) {
        // Stop TIM3 for rotary motor
        status = HAL_TIM_Base_Stop_IT(&htim3);
    } else {
        return CUSTOM_HAL_INVALID_PARAM;
    }

    return (status == HAL_OK) ? CUSTOM_HAL_OK : CUSTOM_HAL_ERROR;
}

/**
 * @brief Set timer frequency (motor step frequency)
 * @param timer_id Timer identifier (0=TIM2, 1=TIM3)
 * @param frequency Desired frequency in Hz (steps per second)
 * @return CUSTOM_HAL_OK on success
 *
 * @note Frequency range: 1 Hz to 100 kHz (typical stepper motor range)
 *       APB1 Timer Clock = 170 MHz (STM32G474RE)
 *       Formula: Freq = APB1_CLK / ((PSC + 1) * (ARR + 1))
 *
 *       For example, 1000 Hz:
 *       PSC = 169, ARR = 999 -> 170MHz / (170 * 1000) = 1000 Hz
 */
hal_status_t hal_timer_set_frequency(uint8_t timer_id, uint32_t frequency) {
    if (frequency == 0 || frequency > 100000) {
        return CUSTOM_HAL_INVALID_PARAM;  // Invalid frequency range
    }

    // Calculate prescaler and period for desired frequency
    // APB1 Timer Clock = 170 MHz
    const uint32_t timer_clock = 170000000;  // 170 MHz

    // Use prescaler = 169 (divide by 170), then adjust period
    uint32_t prescaler = 169;  // PSC = 169 -> divide by 170
    uint32_t period = (timer_clock / (prescaler + 1) / frequency) - 1;

    // Ensure period is within valid range
    if (period > 65535 || period == 0) {
        // Adjust prescaler for very low or very high frequencies
        prescaler = (timer_clock / frequency / 65536);
        if (prescaler > 65535) {
            prescaler = 65535;  // Maximum prescaler
        }
        period = (timer_clock / (prescaler + 1) / frequency) - 1;
    }

    TIM_HandleTypeDef *htim;
    if (timer_id == 0) {
        htim = &htim2;
        __HAL_RCC_TIM2_CLK_ENABLE();
    } else if (timer_id == 1) {
        htim = &htim3;
        __HAL_RCC_TIM3_CLK_ENABLE();
    } else {
        return CUSTOM_HAL_INVALID_PARAM;
    }

    // Configure timer base
    htim->Instance = (timer_id == 0) ? TIM2 : TIM3;
    htim->Init.Prescaler = prescaler;
    htim->Init.CounterMode = TIM_COUNTERMODE_UP;
    htim->Init.Period = period;
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    HAL_StatusTypeDef status = HAL_TIM_Base_Init(htim);
    if (status != HAL_OK) {
        return CUSTOM_HAL_ERROR;
    }

    // Configure interrupt
    HAL_NVIC_SetPriority((timer_id == 0) ? TIM2_IRQn : TIM3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ((timer_id == 0) ? TIM2_IRQn : TIM3_IRQn);

    return CUSTOM_HAL_OK;
}

/* ============================================================================
 * UART FUNCTIONS (Communication)
 * ============================================================================ */

/**
 * @brief Initialize UART for communication
 * @param config UART configuration (baud rate, parity, etc.)
 * @return CUSTOM_HAL_OK on success, CUSTOM_HAL_ERROR on failure
 *
 * @note Default configuration:
 *       - Baud: 115200
 *       - Data: 8 bits
 *       - Stop: 1 bit
 *       - Parity: None
 *       - Flow control: None
 *       - RX interrupt enabled for asynchronous reception
 */
hal_status_t hal_uart_init(const uart_config_t *config) {
    if (!config) {
        return CUSTOM_HAL_INVALID_PARAM;
    }

    // Configure UART
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 9600;
    huart2.Init.WordLength = UART_WORDLENGTH_9B; // 8 data bits + 1 parity bit
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_EVEN;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);

    // Enable NVIC for UART interrupts (used by HAL_UART_Receive_IT)
    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

    // Start interrupt-driven reception (1 byte at a time)
    HAL_UART_Receive_IT(&huart2, &g_uart_rx_byte, 1);

    return CUSTOM_HAL_OK;
}

/**
 * @brief Send a single byte via UART
 * @param data Byte to send
 * @return CUSTOM_HAL_OK on success, CUSTOM_HAL_TIMEOUT if timeout
 *
 * @note Blocking function with 100ms timeout
 *       Watchdog should be refreshed by calling task if needed
 */
hal_status_t hal_uart_send_byte(uint8_t data) {
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart1, &data, 1, 100);
    return (status == HAL_OK) ? CUSTOM_HAL_OK : CUSTOM_HAL_TIMEOUT;
}

/**
 * @brief Send a string via UART
 * @param str Null-terminated string to send
 * @return CUSTOM_HAL_OK on success, CUSTOM_HAL_INVALID_PARAM if NULL
 *
 * @note Blocking function. For long strings, watchdog refresh may be needed
 */
hal_status_t hal_uart_send_string(const char *str) {
    if (!str) {
        return CUSTOM_HAL_INVALID_PARAM;
    }

    uint16_t len = strlen(str);
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart2, (uint8_t*)str, len, HAL_MAX_DELAY);
    return (status == HAL_OK) ? CUSTOM_HAL_OK : CUSTOM_HAL_TIMEOUT;
}

/**
 * @brief Receive a byte from UART (non-blocking)
 * @param data Pointer to store received byte
 * @return CUSTOM_HAL_OK if data available, CUSTOM_HAL_BUSY if no data
 *
 * @note Non-blocking. Check if data is available before reading
 *       In interrupt mode, use registered callback instead
 */
hal_status_t hal_uart_receive_byte(uint8_t *data) {
    if (!data) {
        return CUSTOM_HAL_INVALID_PARAM;
    }

    // Return the last received byte (from interrupt)
    *data = g_uart_rx_byte;
    return CUSTOM_HAL_OK;
}

/**
 * @brief HAL UART RX Complete Callback
 * Called by HAL_UART_IRQHandler when reception is complete
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        // Process the received byte via task manager
        task_manager_uart_rx_isr();


        // Re-arm reception for next byte
        HAL_UART_Receive_IT(&huart2, &g_uart_rx_byte, 1);
    }
}

/**
 * @brief HAL UART Error Callback
 * Called when UART errors occur (overrun, framing, etc.)
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        // Clear errors and restart reception
        __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_OREF | UART_CLEAR_FEF |
                                        UART_CLEAR_NEF | UART_CLEAR_PEF);
        HAL_UART_Receive_IT(&huart2, &g_uart_rx_byte, 1);
    }
}

/* ============================================================================
 * GPIO FUNCTIONS (General Purpose I/O)
 * ============================================================================ */

/**
 * @brief Initialize GPIO pin
 * @param config GPIO configuration (pin, mode, pull, speed)
 * @return CUSTOM_HAL_OK on success
 *
 * @note GPIO pins for motor control are initialized in hal_init()
 *       This function is for additional GPIO configuration if needed
 */
hal_status_t hal_gpio_init(const gpio_config_t *config) {

    hal_status_t status = CUSTOM_HAL_ERROR;
    
    if (!config) {
        return CUSTOM_HAL_INVALID_PARAM;
    }

    // Enable GPIOA clock for LED (PA5)
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // Configure GPIO pins for motor control (PA0-PA1) & onboard LED (PA5)
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Initialize all motor control pins to LOW
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);

    return CUSTOM_HAL_OK;
}

/**
 * @brief Write to GPIO pin
 * @param pin Pin number (0=PA0/Lin_STEP, 1=PA1/Lin_DIR, 2=PA2/Rot_STEP, 3=PA3/Rot_DIR)
 * @param state true=HIGH, false=LOW
 * @return CUSTOM_HAL_OK on success
 */
hal_status_t hal_gpio_write(uint8_t pin, bool state) {
    uint16_t gpio_pin;

    switch (pin) {
        case 0: gpio_pin = GPIO_PIN_0; break;  // Linear STEP
        case 1: gpio_pin = GPIO_PIN_1; break;  // Linear DIR
        case 2: gpio_pin = GPIO_PIN_2; break;  // Rotary STEP
        case 3: gpio_pin = GPIO_PIN_3; break;  // Rotary DIR
        default: return CUSTOM_HAL_INVALID_PARAM;
    }

    HAL_GPIO_WritePin(GPIOA, gpio_pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    return CUSTOM_HAL_OK;
}

/**
 * @brief Read from GPIO pin
 * @param pin Pin number
 * @param state Pointer to store pin state
 * @return CUSTOM_HAL_OK on success
 */
hal_status_t hal_gpio_read(uint8_t pin, bool *state) {
    if (!state) {
        return CUSTOM_HAL_INVALID_PARAM;
    }

    uint16_t gpio_pin;

    switch (pin) {
        case 0: gpio_pin = GPIO_PIN_0; break;
        case 1: gpio_pin = GPIO_PIN_1; break;
        case 2: gpio_pin = GPIO_PIN_2; break;
        case 3: gpio_pin = GPIO_PIN_3; break;
        default: return CUSTOM_HAL_INVALID_PARAM;
    }

    *state = (HAL_GPIO_ReadPin(GPIOA, gpio_pin) == GPIO_PIN_SET);
    return CUSTOM_HAL_OK;
}

/**
 * @brief Toggle GPIO pin state
 * @param pin Pin number
 * @return CUSTOM_HAL_OK on success
 */
hal_status_t hal_gpio_toggle(uint8_t pin) {
    uint16_t gpio_pin;

    switch (pin) {
        case 0: gpio_pin = GPIO_PIN_0; break;
        case 1: gpio_pin = GPIO_PIN_1; break;
        case 2: gpio_pin = GPIO_PIN_2; break;
        case 3: gpio_pin = GPIO_PIN_3; break;
        default: return CUSTOM_HAL_INVALID_PARAM;
    }

    HAL_GPIO_TogglePin(GPIOA, gpio_pin);
    return CUSTOM_HAL_OK;
}

/* ============================================================================
 * MOTOR CONTROL FUNCTIONS
 * ============================================================================ */

/**
 * @brief Set motor direction
 * @param motor_id Motor identifier
 * @param direction MOTOR_DIR_FORWARD or MOTOR_DIR_REVERSE
 * @return CUSTOM_HAL_OK on success
 */
hal_status_t hal_motor_set_direction(motor_id_t motor_id, motor_direction_t direction) {
    if (motor_id >= MOTOR_MAX) {
        return CUSTOM_HAL_INVALID_PARAM;
    }

    g_motors[motor_id].direction = direction;

    // Set direction pin: PA1 for linear, PA3 for rotary
    uint8_t dir_pin = (motor_id == MOTOR_LINEAR) ? 1 : 3;
    hal_gpio_write(dir_pin, direction == MOTOR_DIR_FORWARD);

    return CUSTOM_HAL_OK;
}

/**
 * @brief Start motor (enable timer)
 * @param motor_id Motor identifier
 * @return CUSTOM_HAL_OK on success
 */
hal_status_t hal_motor_start(motor_id_t motor_id) {
    if (motor_id >= MOTOR_MAX) {
        return CUSTOM_HAL_INVALID_PARAM;
    }

    g_motors[motor_id].is_running = true;
    return hal_timer_start(motor_id);
}

/**
 * @brief Stop motor (disable timer)
 * @param motor_id Motor identifier
 * @return CUSTOM_HAL_OK on success
 */
hal_status_t hal_motor_stop(motor_id_t motor_id) {
    if (motor_id >= MOTOR_MAX) {
        return CUSTOM_HAL_INVALID_PARAM;
    }

    g_motors[motor_id].is_running = false;
    return hal_timer_stop(motor_id);
}

/* ============================================================================
 * WATCHDOG FUNCTIONS (System Monitoring)
 * ============================================================================ */

/**
 * @brief Initialize independent watchdog (IWDG)
 * @param timeout_ms Timeout in milliseconds (4 to 32000 ms typical range)
 * @return CUSTOM_HAL_OK on success, CUSTOM_HAL_ERROR on failure
 *
 * @note IWDG uses LSI clock (~32 kHz). Once started, it cannot be stopped!
 *       The watchdog must be refreshed periodically or system will reset.
 *
 *       Calculation: Timeout = (Prescaler * Reload) / LSI_Freq
 *
 *       Example for 1000ms timeout:
 *       LSI = 32kHz, Prescaler = 32 -> 1kHz
 *       Reload = 1000 -> 1000ms timeout
 */
hal_status_t hal_watchdog_init(uint32_t timeout_ms) {
    if (timeout_ms < 4 || timeout_ms > 32000) {
        return CUSTOM_HAL_INVALID_PARAM;  // Out of valid range
    }

    // Calculate prescaler and reload value
    // LSI frequency = 32 kHz (typical)
    // Available prescalers: 4, 8, 16, 32, 64, 128, 256

    uint32_t prescaler_div;
    uint32_t reload;

    if (timeout_ms <= 512) {
        prescaler_div = IWDG_PRESCALER_4;   // 4/32kHz = 125us tick
        reload = (timeout_ms * 32000) / (4 * 1000);
    } else if (timeout_ms <= 1024) {
        prescaler_div = IWDG_PRESCALER_8;   // 8/32kHz = 250us tick
        reload = (timeout_ms * 32000) / (8 * 1000);
    } else if (timeout_ms <= 2048) {
        prescaler_div = IWDG_PRESCALER_16;  // 16/32kHz = 500us tick
        reload = (timeout_ms * 32000) / (16 * 1000);
    } else if (timeout_ms <= 4096) {
        prescaler_div = IWDG_PRESCALER_32;  // 32/32kHz = 1ms tick
        reload = (timeout_ms * 32000) / (32 * 1000);
    } else if (timeout_ms <= 8192) {
        prescaler_div = IWDG_PRESCALER_64;  // 64/32kHz = 2ms tick
        reload = (timeout_ms * 32000) / (64 * 1000);
    } else if (timeout_ms <= 16384) {
        prescaler_div = IWDG_PRESCALER_128; // 128/32kHz = 4ms tick
        reload = (timeout_ms * 32000) / (128 * 1000);
    } else {
        prescaler_div = IWDG_PRESCALER_256; // 256/32kHz = 8ms tick
        reload = (timeout_ms * 32000) / (256 * 1000);
    }

    // Ensure reload is within valid range (0-4095)
    if (reload > 4095) {
        reload = 4095;
    }

    // Configure IWDG
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = prescaler_div;
    hiwdg.Init.Reload = reload;
    hiwdg.Init.Window = IWDG_WINDOW_DISABLE;

    HAL_StatusTypeDef status = HAL_IWDG_Init(&hiwdg);
    if (status != HAL_OK) {
        return CUSTOM_HAL_ERROR;
    }

    g_watchdog_initialized = true;
    return CUSTOM_HAL_OK;
}

/**
 * @brief Refresh the watchdog timer (reset counter)
 * @return CUSTOM_HAL_OK on success
 *
 * @note This function MUST be called periodically (before timeout)
 *       to prevent system reset. Typically called from watchdog refresh task.
 *
 * @warning Failure to refresh causes immediate system reset!
 */
hal_status_t hal_watchdog_refresh(void) {
    if (!g_watchdog_initialized) {
        return CUSTOM_HAL_ERROR;  // Watchdog not initialized
    }

    HAL_StatusTypeDef status = HAL_IWDG_Refresh(&hiwdg);
    return (status == HAL_OK) ? CUSTOM_HAL_OK : CUSTOM_HAL_ERROR;
}

/* ============================================================================
 * SYSTEM FUNCTIONS (Reset, Delay, Interrupts)
 * ============================================================================ */

/**
 * @brief Perform system reset
 * @return Does not return (system resets)
 *
 * @note This function triggers an immediate system reset via NVIC
 */
hal_status_t hal_system_reset(void) {
    NVIC_SystemReset();
    // Never returns
    return CUSTOM_HAL_OK;
}

/**
 * @brief Delay in milliseconds
 * @param delay Delay time in milliseconds
 * @return CUSTOM_HAL_OK
 *
 * @note Uses HAL_Delay() which relies on SysTick timer
 *       This is a blocking delay - watchdog should be considered for long delays
 *       For delays > watchdog timeout, call hal_watchdog_refresh() in loop
 */
hal_status_t hal_delay_ms(uint32_t delay) {
    // For delays longer than watchdog timeout, refresh in loop
    if (g_watchdog_initialized && delay > 1000) {
        uint32_t remaining = delay;
        while (remaining > 0) {
            uint32_t chunk = (remaining > 500) ? 500 : remaining;
            HAL_Delay(chunk);
            hal_watchdog_refresh();  // Keep watchdog happy
            remaining -= chunk;
        }
    } else {
        HAL_Delay(delay);
    }
    return CUSTOM_HAL_OK;
}

/**
 * @brief Delay in microseconds
 * @param delay Delay time in microseconds
 * @return CUSTOM_HAL_OK
 *
 * @note Busy-wait loop for short delays. Not suitable for long delays.
 *       Max recommended: ~10ms (10000us)
 */
hal_status_t hal_delay_us(uint32_t delay) {
    // Use DWT cycle counter for precise microsecond delay
    // At 170MHz: 170 cycles = 1us
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = delay * (SystemCoreClock / 1000000);

    while ((DWT->CYCCNT - start) < cycles) {
        // Busy wait - watchdog will NOT be affected for short delays
        __NOP();
    }

    return CUSTOM_HAL_OK;
}

/**
 * @brief Enable interrupt
 * @param irq_number IRQ number to enable
 * @return CUSTOM_HAL_OK
 */
hal_status_t hal_interrupt_enable(uint32_t irq_number) {
    HAL_NVIC_EnableIRQ((IRQn_Type)irq_number);
    return CUSTOM_HAL_OK;
}

/**
 * @brief Disable interrupt
 * @param irq_number IRQ number to disable
 * @return CUSTOM_HAL_OK
 */
hal_status_t hal_interrupt_disable(uint32_t irq_number) {
    HAL_NVIC_DisableIRQ((IRQn_Type)irq_number);
    return CUSTOM_HAL_OK;
}

/* ============================================================================
 * STM32-SPECIFIC UTILITY FUNCTIONS
 * ============================================================================ */

/**
 * @brief Error handler - called on HAL errors
 * @note Disables interrupts and enters infinite loop
 *       Watchdog will eventually reset the system if enabled
 */
void STM32_Error_Handler(void) {
    __disable_irq();
    while (1) {
        // Infinite loop - watchdog will reset system
        // In debug mode, execution stops here
    }
}

/**
 * @brief System clock configuration (170 MHz from HSI)
 * @note Called from main.c during system initialization
 *       Uses HSI (16 MHz internal oscillator) with PLL
 *       PLL: 16MHz / 4 * 85 / 2 = 170 MHz
 */
void STM32_SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    // Enable power control clock
    __HAL_RCC_PWR_CLK_ENABLE();

    // Configure voltage scaling for maximum performance (170 MHz)
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    // Configure HSI oscillator and PLL
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 4;   // Divide by 4: 16MHz / 4 = 4MHz
    RCC_OscInitStruct.PLL.PLLN = 85;  // Multiply by 85: 4MHz * 85 = 340MHz
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;  // Divide by 2: 340MHz / 2 = 170MHz

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        STM32_Error_Handler();
    }

    // Configure system clocks
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;   // AHB = 170 MHz
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;    // APB1 = 170 MHz
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;    // APB2 = 170 MHz

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_8) != HAL_OK) {
        STM32_Error_Handler();
    }

    // Update SystemCoreClock variable
    SystemCoreClockUpdate();
}

/**
 * @brief GPIO initialization (example for status LED)
 * @note Called from main.c if needed for additional GPIO
 */
void STM32_GPIO_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // Configure PA5 as output (Nucleo onboard LED)
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
}
