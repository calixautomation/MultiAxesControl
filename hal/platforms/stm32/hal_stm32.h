/**
 * @file hal_stm32.h
 * @brief STM32 platform HAL header
 * @author MultiAxesControl
 * @date 2025
 */

#ifndef HAL_STM32_H
#define HAL_STM32_H

#include "stm32g4xx.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_usart.h"
#include "stm32g4xx_ll_tim.h"

#ifdef __cplusplus
extern "C" {
#endif

// STM32-specific definitions
#define STM32_SYSTEM_CLOCK_FREQ    170000000UL   // 170 MHz system clock
#define STM32_APB1_TIMER_FREQ      170000000UL
#define STM32_APB2_TIMER_FREQ      170000000UL

// Timer definitions
#define STM32_TIMER_LINEAR         TIM2
#define STM32_TIMER_ROTARY         TIM3
#define STM32_TIMER_LINEAR_HANDLE  htim2
#define STM32_TIMER_ROTARY_HANDLE  htim3

// UART definitions
#define STM32_UART_HANDLE          huart1
#define STM32_UART_INSTANCE        USART1

// GPIO pin definitions for STM32F4 Discovery
#define STM32_LINEAR_STEP_PIN      GPIO_PIN_0
#define STM32_LINEAR_STEP_PORT     GPIOA
#define STM32_LINEAR_DIR_PIN       GPIO_PIN_1
#define STM32_LINEAR_DIR_PORT      GPIOA

#define STM32_ROTARY_STEP_PIN      GPIO_PIN_2
#define STM32_ROTARY_STEP_PORT     GPIOA
#define STM32_ROTARY_DIR_PIN       GPIO_PIN_3
#define STM32_ROTARY_DIR_PORT      GPIOA

// Timer channels for PWM
#define STM32_LINEAR_TIMER_CHANNEL TIM_CHANNEL_1
#define STM32_ROTARY_TIMER_CHANNEL TIM_CHANNEL_2

// Interrupt priorities
#define STM32_TIMER_IRQ_PRIORITY   5
#define STM32_UART_IRQ_PRIORITY    6
#define STM32_SYSTICK_IRQ_PRIORITY 7

// No HAL handles when using LL

// STM32-specific functions
void STM32_SystemClock_Config(void);
void STM32_GPIO_Init(void);
void STM32_TIM_Init(void);
void STM32_UART_Init(void);
void STM32_Error_Handler(void);

// Interrupt handlers
void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);
void USART1_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* HAL_STM32_H */
