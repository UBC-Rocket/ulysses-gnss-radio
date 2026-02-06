/**
 * @file debug_uart.h
 * @brief Debug UART interface - bidirectional injection and logging
 *
 * RX (Injection): DMA + IDLE mode with binary protocol for message injection
 *   - 0x52 ('R') → Inject radio message (null-terminated binary)
 *   - 0x47 ('G') → Inject GPS NMEA (standard NMEA format with \r\n)
 *
 * TX (Logging): Human-readable printf-style logging of all system events
 *   - Radio messages from UART5
 *   - GPS NMEA sentences from UART6
 *   - Decoded GPS fixes
 *   - SPI radio TX messages from master
 *
 * Only compiled when DEBUG macro is defined (zero impact on Release builds).
 */

#ifndef DEBUG_UART_H
#define DEBUG_UART_H

#ifdef DEBUG

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "stm32g0xx_hal.h"
#include "radio_queue.h"
#include "gps_nema_queue.h"
#include "protocol_config.h"

/* ============================================================================
 * Protocol Constants
 * ============================================================================ */

/** Injection type byte for radio messages (ASCII 'R') */
#define DEBUG_UART_TYPE_RADIO 0x52

/** Injection type byte for GPS messages (ASCII 'G') */
#define DEBUG_UART_TYPE_GPS   0x47

/** DMA circular buffer size for RX injection (matches GPS/Radio pattern) */
#define DEBUG_UART_RX_BUFFER_SIZE 512

/** Maximum pending log messages in queue */
#define DEBUG_UART_LOG_QUEUE_DEPTH 10

/** Maximum characters per log message */
#define DEBUG_UART_LOG_MSG_SIZE 256

/* ============================================================================
 * Data Structures
 * ============================================================================ */

/**
 * @brief Log message queue for buffering debug output
 *
 * Ring buffer of formatted text messages waiting to be transmitted
 * via UART1. Populated by logging functions, drained by debug_uart_process_logs().
 */
typedef struct {
    char messages[DEBUG_UART_LOG_QUEUE_DEPTH][DEBUG_UART_LOG_MSG_SIZE];
    volatile uint8_t head;  /**< Next write position */
    volatile uint8_t tail;  /**< Next read position */
} debug_log_queue_t;

/* ============================================================================
 * External Variables
 * ============================================================================ */

/** DMA circular buffer for RX injection (defined in debug_uart.c) */
extern uint8_t debug_uart_rx_buffer[DEBUG_UART_RX_BUFFER_SIZE];

/* ============================================================================
 * Public API - Initialization
 * ============================================================================ */

/**
 * @brief Initialize debug UART system (injection and logging)
 *
 * Sets up DMA reception for message injection and initializes log queue.
 * Must be called after UART1 peripheral initialization in main().
 *
 * @param radio_q Pointer to radio message queue (for injection)
 * @param gps_q Pointer to GPS NMEA queue (for injection)
 */
void debug_uart_init(radio_message_queue_t *radio_q, gps_sample_queue_t *gps_q);

/* ============================================================================
 * Public API - Injection (RX)
 * ============================================================================ */

/**
 * @brief Process DMA RX event for message injection
 *
 * Called from HAL_UARTEx_RxEventCallback when IDLE/HT/TC interrupt occurs.
 * Parses type byte and routes message to appropriate queue.
 *
 * Protocol:
 *   [0x52][payload...] → Inject to radio queue (null-terminated)
 *   [0x47][payload...] → Inject to GPS queue (NMEA format with \r\n)
 *
 * @param huart UART handle (should be huart1)
 * @param Size Number of bytes received in DMA buffer
 */
void debug_uart_rx_event_callback(UART_HandleTypeDef *huart, uint16_t Size);

/* ============================================================================
 * Public API - Logging (TX)
 * ============================================================================ */

/**
 * @brief Log radio message to debug console
 *
 * Formats and enqueues message: "[RADIO RX] <hex> (<ASCII>)"
 * Called from radio_driver.c when message received from UART5.
 *
 * @param msg Pointer to radio message bytes
 * @param len Length of message in bytes
 */
void debug_uart_log_radio(const uint8_t *msg, uint16_t len);

/**
 * @brief Log GPS NMEA sentence to debug console
 *
 * Formats and enqueues message: "[GPS NMEA] <sentence>"
 * Called from gps.c when NMEA sentence received from UART6.
 *
 * @param nmea Pointer to null-terminated NMEA sentence string
 */
void debug_uart_log_gps_nmea(const char *nmea);

/**
 * @brief Log decoded GPS fix to debug console
 *
 * Formats and enqueues message: "[GPS FIX] Lat: X, Lon: Y, Alt: Z, ..."
 * Called from gps.c after successful GPS fix parse.
 *
 * @param fix Pointer to decoded GPS fix structure
 */
void debug_uart_log_gps_fix(const gps_fix_t *fix);

/**
 * @brief Log SPI radio TX message to debug console
 *
 * Formats and enqueues message: "[SPI TX] Radio msg from master: <hex> (<ASCII>)"
 * Called from spi_slave.c when radio TX received from SPI master.
 *
 * @param msg Pointer to radio message bytes
 * @param len Length of message in bytes
 */
void debug_uart_log_spi_radio_tx(const uint8_t *msg, uint16_t len);

/**
 * @brief Process and transmit pending log messages
 *
 * Dequeues and transmits log messages via UART1 TX (blocking).
 * Should be called regularly from main loop to drain log queue.
 * Transmits one message per call to avoid blocking too long.
 */
void debug_uart_process_logs(void);

#ifdef __cplusplus
}
#endif

#endif // DEBUG

#endif // DEBUG_UART_H
