#include "timing_utils.h"
#include "stm32g0xx_hal.h"

// Use TIM2 (32-bit timer) for microsecond timing
// TIM2 is available on all STM32G0 variants
static TIM_HandleTypeDef htim2;

/**
 * @brief Initialize TIM2 for microsecond timing
 *
 * Configures TIM2 as a free-running 32-bit counter incremented every microsecond.
 * Assumes system clock is configured (typically 64 MHz for STM32G0).
 */
void timing_init(void) {
    // Enable TIM2 clock
    __HAL_RCC_TIM2_CLK_ENABLE();

    // Configure TIM2 for 1 MHz (1 µs resolution)
    // Assuming APB1 timer clock is 64 MHz (typical for STM32G0 at full speed)
    // Prescaler = (APB1_TIMER_CLK / 1000000) - 1
    // For 64 MHz: prescaler = 63 (64 ticks per µs)

    uint32_t apb1_timer_clock = HAL_RCC_GetPCLK1Freq();

    // If APB1 prescaler is not 1, timer clock is 2x PCLK1
    RCC_ClkInitTypeDef clk_init;
    uint32_t flash_latency;
    HAL_RCC_GetClockConfig(&clk_init, &flash_latency);
    if (clk_init.APB1CLKDivider != RCC_HCLK_DIV1) {
        apb1_timer_clock *= 2;
    }

    uint32_t prescaler = (apb1_timer_clock / 1000000) - 1;

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = prescaler;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xFFFFFFFF;  // 32-bit, free-running
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
        // Initialization error - halt for debugging
        while(1);
    }

    // Start the timer
    HAL_TIM_Base_Start(&htim2);
}

/**
 * @brief Get current timestamp in microseconds
 *
 * @return uint32_t Current value of TIM2 counter (microseconds since init)
 */
uint32_t get_time_us(void) {
    return __HAL_TIM_GET_COUNTER(&htim2);
}

/**
 * @brief Blocking delay in microseconds
 *
 * @param delay_us Number of microseconds to delay
 *
 * @note This is a busy-wait delay, use sparingly
 */
void delay_us(uint32_t delay_us) {
    uint32_t start = get_time_us();
    while (elapsed_us(start, get_time_us()) < delay_us) {
        // Busy wait
    }
}

/**
 * @brief HAL MSP initialization for TIM2
 *
 * Called by HAL_TIM_Base_Init(). No additional peripheral config needed
 * for basic timing (no pins, no interrupts).
 */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base) {
    if(htim_base->Instance == TIM2) {
        // Clock already enabled in timing_init()
        // No GPIO or NVIC config needed for simple counter mode
    }
}

/**
 * @brief HAL MSP de-initialization for TIM2
 */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base) {
    if(htim_base->Instance == TIM2) {
        __HAL_RCC_TIM2_CLK_DISABLE();
    }
}
