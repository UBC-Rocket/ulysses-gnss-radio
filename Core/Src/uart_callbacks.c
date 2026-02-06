/**
 * @file uart_callbacks.c
 * @brief HAL UART callback implementations
 *
 * Routes UART events to appropriate driver modules:
 * - USART1: Debug console (per-byte interrupt mode)
 * - USART5: Radio transceiver (DMA + IDLE mode)
 * - USART6: GPS module (DMA + IDLE mode)
 */

#include "uart_callbacks.h"
#include "gps.h"
#include "radio_driver.h"
#include "stm32g0xx_hal.h"
#ifdef DEBUG
#include "debug_uart.h"
#endif

/* ============================================================================
 * External UART Handles
 * ============================================================================ */

extern UART_HandleTypeDef huart1;  /* Debug console (defined in main.c) */
extern UART_HandleTypeDef huart5;  /* Radio UART (defined in main.c) */
extern UART_HandleTypeDef huart6;  /* GPS UART (defined in main.c) */

/* ============================================================================
 * Private Variables
 * ============================================================================ */

/**
 * @brief Debug console RX buffer (1 byte for interrupt mode)
 *
 * Used by UART1 debug console for forwarding bytes to radio.
 */
static uint8_t debug_rx_buffer[1];

/* ============================================================================
 * Initialization
 * ============================================================================ */

void uart_callbacks_init(UART_HandleTypeDef *huart1_handle)
{
    // Enable UART RX interrupts for debug console (per-byte mode)
    HAL_UART_Receive_IT(huart1_handle, debug_rx_buffer, 1);
}

/* ============================================================================
 * HAL UART Callbacks
 * ============================================================================ */

/**
 * @brief DMA RX Event callback for IDLE/HT/TC events
 *
 * This is the main receive handler for DMA + IDLE mode.
 * Routes events to GPS (UART6) and Radio (UART5) drivers.
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART6)
    {
        // GPS DMA event (IDLE, Half-Transfer, or Transfer-Complete)
        gps_rx_event_callback(huart, Size);
    }
    else if (huart->Instance == USART5)
    {
        // Radio DMA event
        radio_rx_event_callback(huart, Size);
    }
#ifdef DEBUG
    else if (huart->Instance == USART1)
    {
        // Debug injection DMA event
        debug_uart_rx_event_callback(huart, Size);
    }
#endif
}

/**
 * @brief Legacy RX complete callback (for per-byte interrupt mode)
 *
 * Only used for UART1 debug console in Release builds.
 * In Debug builds, UART1 uses DMA + IDLE mode via HAL_UARTEx_RxEventCallback.
 * GPS and Radio use DMA + IDLE via HAL_UARTEx_RxEventCallback.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
#ifndef DEBUG
    // Only in Release builds: forward bytes to radio
    if (huart->Instance == USART1)
    {
        // Forward received byte to Radio (UART5) - NON-BLOCKING
        HAL_UART_Transmit_IT(&huart5, debug_rx_buffer, 1);

        // Echo back to ST-Link (so you see what you typed)
        HAL_UART_Transmit(&huart1, debug_rx_buffer, 1, 100);

        // Re-enable receive interrupt for next byte
        HAL_UART_Receive_IT(&huart1, debug_rx_buffer, 1);
    }
#endif
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    // Route to GPS driver for debug output TX completion
    gps_uart_tx_cplt_callback(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6)
    {
        // GPS UART error - attempt recovery
        gps_uart_error_callback(huart);
    }
    else if (huart->Instance == USART5)
    {
        // Radio UART error - attempt recovery
        radio_uart_error_callback(huart);
    }
}
