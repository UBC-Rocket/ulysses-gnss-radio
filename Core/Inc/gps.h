#pragma once

// Match your project MCU family include.
// If you're not on G0 anymore, update this to your actual HAL header.
#include "stm32g0xx_hal.h" // <-- change if needed (e.g., stm32g0xx_hal.h)

#include <stdbool.h>
#include <stddef.h> // <-- needed for size_t
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Core API
void gps_init(UART_HandleTypeDef *gps_uart, UART_HandleTypeDef *out_uart);
void gps_process(void);

// Hook these from HAL callbacks in main.c
void gps_uart_rx_cplt_callback(UART_HandleTypeDef *huart);
void gps_uart_tx_cplt_callback(UART_HandleTypeDef *huart);
void gps_uart_error_callback(UART_HandleTypeDef *huart);

// ---------------------- TEST / SIM INJECTION ----------------------
void gps_sim_rx_byte(uint8_t b);
void gps_sim_rx_stream(const uint8_t *data, size_t n);
// ------------------------------------------------------------------

#ifdef __cplusplus
}
#endif
