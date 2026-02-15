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

    // Send startup message
    task_manager_send_response("System is preparing...");
    task_manager_send_response("Is this working fine..");

    // Initialize HAL layer
    hal_status_t hal_status = hal_init();
    if (hal_status != CUSTOM_HAL_OK) {
        while(1); // Halt on error
    }

    // Initialize task manager and create all FreeRTOS tasks
    if (task_manager_init() != TASK_OK) {
        while(1); // Halt on error
    }

    // Initialize motor control subsystem
    if (motor_control_init() != APP_OK) {
        while(1); // Halt on error
    }
    
    // Initialize watchdog (after all init is done)
    //IWDG_Init();

    // Start FreeRTOS scheduler
    
    task_manager_send_response("Starting scheduler...\n");
    vTaskStartScheduler();
    task_manager_send_response("Scheduler returned!\n");  // Should NEVER print


    // Should never reach here
    for(;;) {
        // Infinite loop in case scheduler returns
    }
}
