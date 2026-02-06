/**
 * @file uart_callbacks.h
 * @brief HAL UART callback declarations
 *
 * Routes UART events to appropriate driver modules:
 * - USART1: Debug console (per-byte interrupt mode)
 * - USART5: Radio transceiver (DMA + IDLE mode)
 * - USART6: GPS module (DMA + IDLE mode)
 */

#ifndef UART_CALLBACKS_H
#define UART_CALLBACKS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g0xx_hal.h"

/* ============================================================================
 * HAL UART Callback Functions
 * ============================================================================ */

/**
 * @brief Initialize UART callback system
 *
 * Sets up the debug console receive buffer and enables UART1 interrupt mode.
 * Must be called after UART peripheral initialization.
 *
 * @param huart1_handle Pointer to UART1 handle (debug console)
 */
void uart_callbacks_init(UART_HandleTypeDef *huart1_handle);

/**
 * @brief DMA RX Event callback for IDLE/HT/TC events
 *
 * Routes DMA events to GPS (UART6) and Radio (UART5) drivers.
 * Called automatically by HAL from interrupt context.
 *
 * @param huart UART handle
 * @param Size Current position in DMA buffer
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

/**
 * @brief Legacy RX complete callback (for per-byte interrupt mode)
 *
 * Only used for UART1 debug console.
 * Called automatically by HAL from interrupt context.
 *
 * @param huart UART handle
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

/**
 * @brief TX complete callback
 *
 * Routes to GPS driver for debug output.
 * Called automatically by HAL from interrupt context.
 *
 * @param huart UART handle
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

/**
 * @brief UART error callback
 *
 * Routes errors to GPS (UART6) and Radio (UART5) drivers for recovery.
 * Called automatically by HAL from interrupt context.
 *
 * @param huart UART handle
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* UART_CALLBACKS_H */
