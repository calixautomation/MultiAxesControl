/**
 * @file stm32g4xx_it.c
 * @brief STM32G4xx Interrupt Service Routines (Strong Implementations)
 *
 * This file contains all ISR handlers and strong function overrides
 * that replace weak definitions from startup code and HAL libraries.
 */

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_uart.h"
#include "FreeRTOS.h"
#include "task.h"

/* =============================================================================
 * External References
 * ============================================================================= */

// UART handle from main firmware (when used)
extern UART_HandleTypeDef huart2 __attribute__((weak));

// Application callbacks (weak - may not exist in all builds)
extern void motor_control_step_notification(uint8_t motor_id) __attribute__((weak));
extern void task_manager_uart_rx_isr(void) __attribute__((weak));

static TIM_HandleTypeDef htim6;

/* =============================================================================
 * Cortex-M4 Core Interrupt Handlers
 * ============================================================================= */

/**
 * @brief Non-Maskable Interrupt handler
 */
void NMI_Handler(void) {
    while (1) {
        // Stay here
    }
}

/**
 * @brief Hard Fault handler
 */
void HardFault_Handler(void) {
    while (1) {
        // Stay here
    }
}

/**
 * @brief Memory Management Fault handler
 */
void MemManage_Handler(void) {
    while (1) {
        // Stay here
    }
}

/**
 * @brief Bus Fault handler
 */
void BusFault_Handler(void) {
    while (1) {
        // Stay here
    }
}

/**
 * @brief Usage Fault handler
 */
void UsageFault_Handler(void) {
    while (1) {
        // Stay here
    }
}

/**
 * @brief Debug Monitor handler
 */
void DebugMon_Handler(void) {
    // Nothing to do
}


/* =============================================================================
 * Peripheral Interrupt Handlers
 * ============================================================================= */


/**
 * @brief USART2 interrupt handler
 * Uses HAL's interrupt handler - callbacks handle data processing
 */
void USART2_IRQHandler(void) {
    // Guard: huart2 might not be initialized yet
    if (huart2.Instance == NULL) {
        return;
    }

    // HAL handles all UART interrupts and calls HAL_UART_RxCpltCallback when done
    HAL_UART_IRQHandler(&huart2);
}

/**
 * @brief TIM2 interrupt handler (Linear motor step generation)
 */
void TIM2_IRQHandler(void) {
    static TIM_HandleTypeDef htim2 = {.Instance = TIM2};

    if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) != RESET) {
        if (__HAL_TIM_GET_IT_SOURCE(&htim2, TIM_IT_UPDATE) != RESET) {
            __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);

            if (motor_control_step_notification) {
                motor_control_step_notification(0);  // MOTOR_LINEAR = 0
            }
        }
    }
}

/**
 * @brief TIM3 interrupt handler (Rotary motor step generation)
 */
void TIM3_IRQHandler(void) {
    static TIM_HandleTypeDef htim3 = {.Instance = TIM3};

    if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE) != RESET) {
        if (__HAL_TIM_GET_IT_SOURCE(&htim3, TIM_IT_UPDATE) != RESET) {
            __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);

            if (motor_control_step_notification) {
                motor_control_step_notification(1);  // MOTOR_ROTARY = 1
            }
        }
    }
}

/**
 * @brief WWDG interrupt handler (Window Watchdog Early Wakeup)
 */
void WWDG_IRQHandler(void) {
    // Clear the early wakeup interrupt flag
    WWDG->SR = 0;
}

/**
 * @brief  TIM6 interrupt handler — increments the HAL tick.
 */
void TIM6_DAC_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim6);
}

/* =============================================================================
 * HAL MSP Initialization (called by HAL_xxx_Init functions)
 * ============================================================================= */

/**
 * @brief UART MSP Initialization
 * @param huart UART handle
 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (huart->Instance == LPUART1) {
        __HAL_RCC_LPUART1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        // PA2 = TX, PA3 = RX
        GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF12_LPUART1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(LPUART1_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(LPUART1_IRQn);
    }
    else if (huart->Instance == USART2) {
        __HAL_RCC_USART2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        // PA2 = TX (push-pull, no pull)
        GPIO_InitStruct.Pin = GPIO_PIN_2;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        // PA3 = RX (with pull-up to prevent floating input noise)
        GPIO_InitStruct.Pin = GPIO_PIN_3;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        // Note: NVIC interrupt NOT enabled here - application must enable after
        // system is ready (e.g., after FreeRTOS queues are created)
    }
}

/**
 * @brief UART MSP De-Initialization
 * @param huart UART handle
 */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart) {
    if (huart->Instance == LPUART1) {
        __HAL_RCC_LPUART1_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2 | GPIO_PIN_3);
        HAL_NVIC_DisableIRQ(LPUART1_IRQn);
    }
    else if (huart->Instance == USART2) {
        __HAL_RCC_USART2_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2 | GPIO_PIN_3);
        HAL_NVIC_DisableIRQ(USART2_IRQn);
    }
}

/**
 * @brief TIM MSP Initialization
 * @param htim TIM handle
 */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        __HAL_RCC_TIM2_CLK_ENABLE();
        HAL_NVIC_SetPriority(TIM2_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(TIM2_IRQn);
    }
    else if (htim->Instance == TIM3) {
        __HAL_RCC_TIM3_CLK_ENABLE();
        HAL_NVIC_SetPriority(TIM3_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(TIM3_IRQn);
    }
}

/**
 * @brief TIM MSP De-Initialization
 * @param htim TIM handle
 */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        __HAL_RCC_TIM2_CLK_DISABLE();
        HAL_NVIC_DisableIRQ(TIM2_IRQn);
    }
    else if (htim->Instance == TIM3) {
        __HAL_RCC_TIM3_CLK_DISABLE();
        HAL_NVIC_DisableIRQ(TIM3_IRQn);
    }
}

/**
 * @brief  Override the weak HAL_InitTick to use TIM6 instead of SysTick.
 * @param  TickPriority  Tick interrupt priority.
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
    RCC_ClkInitTypeDef clkconfig;
    uint32_t           uwTimclock;
    uint32_t           uwPrescalerValue;
    uint32_t           pFLatency;
    HAL_StatusTypeDef  status;

    __HAL_RCC_TIM6_CLK_ENABLE();

    HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);

    /* TIM6 is on APB1. If APB1 prescaler != 1, timer clock is 2x APB1. */
    if (clkconfig.APB1CLKDivider == RCC_HCLK_DIV1) {
        uwTimclock = HAL_RCC_GetPCLK1Freq();
    } else {
        uwTimclock = 2UL * HAL_RCC_GetPCLK1Freq();
    }

    /* Prescale to 10 kHz, then count 10 ticks for 1 ms period. */
    uwPrescalerValue = (uint32_t)((uwTimclock / 10000U) - 1U);

    htim6.Instance               = TIM6;
    htim6.Init.Prescaler         = uwPrescalerValue;
    htim6.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim6.Init.Period            = (10U - 1U);  /* 10 kHz / 10 = 1 kHz (1 ms) */
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    status = HAL_TIM_Base_Init(&htim6);
    if (status != HAL_OK) {
        return status;
    }

    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, TickPriority, 0U);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

    return HAL_TIM_Base_Start_IT(&htim6);
}

/**
 * @brief  Suspend the HAL tick (stop TIM6).
 */
void HAL_SuspendTick(void)
{
    __HAL_TIM_DISABLE_IT(&htim6, TIM_IT_UPDATE);
}

/**
 * @brief  Resume the HAL tick (restart TIM6).
 */
void HAL_ResumeTick(void)
{
    __HAL_TIM_ENABLE_IT(&htim6, TIM_IT_UPDATE);
}

/**
 * @brief  Period elapsed callback — called by HAL_TIM_IRQHandler.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        HAL_IncTick();
    }
}
