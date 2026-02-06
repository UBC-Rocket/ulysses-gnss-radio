#pragma once

/**
 * @file gps.h
 * @brief GPS Driver using DMA + IDLE line detection
 *
 * Efficient NMEA sentence reception with minimal CPU overhead.
 * Uses DMA circular buffer with IDLE/HT/TC interrupts instead of per-byte.
 */

#include "stm32g0xx_hal.h"
#include "gps_nema_queue.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Initialization
 * ============================================================================ */

/**
 * @brief Set the GPS sample queue for SPI push mode
 * @param queue Pointer to shared GPS sample queue
 */
void gps_set_queue(gps_sample_queue_t *queue);

/**
 * @brief Initialize GPS driver with DMA reception
 *
 * Starts DMA circular receive with IDLE line detection.
 * Call after HAL peripheral initialization.
 *
 * @param gps_uart UART handle for GPS module (e.g., UART6)
 * @param out_uart UART handle for debug output (e.g., UART1), can be NULL
 */
void gps_init(UART_HandleTypeDef *gps_uart, UART_HandleTypeDef *out_uart);

/**
 * @brief Process GPS data (main loop call)
 *
 * In DMA mode, most processing happens in callbacks.
 * This can be called from main loop for any deferred processing.
 */
void gps_process(void);

/* ============================================================================
 * UART Callbacks - Wire these from main.c or stm32g0xx_it.c
 * ============================================================================ */

/**
 * @brief DMA RX Event callback (IDLE, Half-Transfer, Transfer-Complete)
 *
 * This is the main receive handler. Call from HAL_UARTEx_RxEventCallback.
 *
 * @param huart UART handle
 * @param Size Current position in DMA buffer
 */
void gps_rx_event_callback(UART_HandleTypeDef *huart, uint16_t Size);

/**
 * @brief UART TX complete callback (for debug output)
 *
 * Call from HAL_UART_TxCpltCallback.
 *
 * @param huart UART handle
 */
void gps_uart_tx_cplt_callback(UART_HandleTypeDef *huart);

/**
 * @brief UART error callback
 *
 * Call from HAL_UART_ErrorCallback. Restarts DMA on error.
 *
 * @param huart UART handle
 */
void gps_uart_error_callback(UART_HandleTypeDef *huart);

/**
 * @brief Legacy RX complete callback (not used in DMA mode)
 *
 * Kept for compatibility. Does nothing in DMA mode.
 *
 * @param huart UART handle
 */
void gps_uart_rx_cplt_callback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif
