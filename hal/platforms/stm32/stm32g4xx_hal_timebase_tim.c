/**
 * @file    stm32g4xx_hal_timebase_tim.c
 * @brief   HAL timebase using TIM6 instead of SysTick.
 *
 * When FreeRTOS owns SysTick, the HAL needs a separate timer for its
 * internal tick (used by HAL_Delay, HAL_GetTick, and timeout logic in
 * HAL driver functions like HAL_RCC_OscConfig).
 *
 * TIM6 is a basic timer with no external pins, making it ideal for this.
 */

#include "stm32g4xx_hal.h"

static TIM_HandleTypeDef htim6;

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
 * @brief  TIM6 interrupt handler — increments the HAL tick.
 */
void TIM6_DAC_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim6);
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
