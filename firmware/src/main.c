/**
 * @file main.c
 * @brief Multi-Axis Motor Control System for STM32G474RE
 * @author MultiAxesControl
 * @date 2026
 *
 * Main entry point - initializes HAL, application, and RTOS layers
 */

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_uart.h"
#include "stm32g4xx_hal_iwdg.h"
#include "hal_interface.h"
#include "FreeRTOS.h"
#include "task.h"
#include "motor_control.h"
#include "task_manager.h"
#include <string.h>

// System ready flag
volatile uint8_t g_system_ready = 0;

// UART Handle (for basic init only)
UART_HandleTypeDef huart2;

/**
 * @brief System Clock Configuration
 * Configure the system clock to 170MHz using HSI and PLL
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    // Configure voltage scaling for max performance
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    // Configure HSI oscillator and PLL
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;      // 16MHz / 4 = 4MHz
    RCC_OscInitStruct.PLL.PLLN = 85;                  // 4MHz * 85 = 340MHz
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;      // 340MHz / 2 = 170MHz
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;      // 340MHz / 2 = 170MHz (SYSCLK)
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    // Configure system clocks
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;   // 170MHz
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;    // 170MHz
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;    // 170MHz
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
}

/**
 * @brief GPIO Initialization
 * Configure GPIO pins for LED
 */
void GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Enable GPIOA clock for LED (PA5)
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // Configure LED pin (PA5) as output
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
 * @brief UART2 Initialization
 * Basic UART setup for communication (detailed config in HAL layer)
 */
void UART_Init(void) {

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
}

/**
 * @brief Independent Watchdog Initialization
 */
void IWDG_Init(void) {
    // IWDG Handle
    IWDG_HandleTypeDef hiwdg;

    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
    hiwdg.Init.Reload = 1000; // ~2 seconds at 32kHz LSI / 64
    HAL_IWDG_Init(&hiwdg);
}

/**
 * @brief FreeRTOS Idle Hook - Send ready message once
 */
void vApplicationIdleHook(void) {
    if (!g_system_ready) {
        task_manager_send_response("System is Ready\n");
        g_system_ready = 1;
    }
}

/**
 * @brief FreeRTOS assertion failure hook
 */
void vAssertCalled(const char *file, int line) {
    (void)file;
    (void)line;
    taskDISABLE_INTERRUPTS();
    for(;;) {
        // Halt on assertion failure
    }
}

/**
 * @brief FreeRTOS malloc failure hook
 */
void vApplicationMallocFailedHook(void) {
    taskDISABLE_INTERRUPTS();
    for(;;) {
        // Halt on malloc failure
    }
}

/**
 * @brief FreeRTOS stack overflow hook
 */
void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name) {
    (void)task;
    (void)task_name;
    taskDISABLE_INTERRUPTS();
    for(;;) {
        // Halt on stack overflow
    }
}

/**
 * @brief Main entry point
 */
int main(void) {
    // Initialize HAL
    HAL_Init();

    // Configure system clock to 170MHz
    SystemClock_Config();

    // Initialize peripherals
    GPIO_Init();
    UART_Init();

    // Send startup message
    task_manager_send_response("System is preparing...");
    task_manager_send_response("Is this working fine..");

    // Initialize motor control subsystem
    if (motor_control_init() != APP_OK) {
        while(1); // Halt on error
    }
    #if 1
    // Initialize task manager and create all FreeRTOS tasks
    if (task_manager_init() != TASK_OK) {
        while(1); // Halt on error
    }

    // Clear any pending flags, then enable RX interrupt
    __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_PEF);
    while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {
        (void)huart2.Instance->RDR;  // Flush RX buffer
    }
    // NOW enable UART RX interrupt (queue exists, safe to use ISR)
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    // Initialize watchdog (after all init is done)
    //IWDG_Init();
    #endif

    // Start FreeRTOS scheduler
    
    task_manager_send_response("Starting scheduler...\n");
    vTaskStartScheduler();
    task_manager_send_response("Scheduler returned!\n");  // Should NEVER print


    // Should never reach here
    for(;;) {
        // Infinite loop in case scheduler returns
    }
}
