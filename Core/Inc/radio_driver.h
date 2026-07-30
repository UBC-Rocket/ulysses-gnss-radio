/**
 * @file radio_driver.h
 * @brief Radio transceiver driver using DMA circular + Character Match
 *
 * Provides a simple API for sending and receiving radio messages
 * via UART5. Messages are null-terminated (0x00) on the wire.
 *
 * Uses DMA circular buffer with Character Match on 0x00 as the sole
 * interrupt trigger. DMA never stops — no restart gaps, no data loss.
 */

#ifndef RADIO_DRIVER_H
#define RADIO_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "radio_queue.h"
#include "stm32g0xx_hal.h"

/* ============================================================================
 * Configuration
 * ============================================================================ */

#define RADIO_MAX_MESSAGE_LEN   RADIO_MESSAGE_MAX_LEN  /* 256 bytes */

/* ============================================================================
 * Initialization
 * ============================================================================ */

/**
 * @brief Initialize radio driver with DMA reception
 *
 * Sets up UART5 for radio communication with DMA circular buffer
 * and IDLE line detection. Initializes the RX queue.
 *
 * @param rx_queue Pointer to queue for storing received messages
 */
void radio_init(radio_message_queue_t *rx_queue);

/* ============================================================================
 * TX API
 * ============================================================================ */

/**
 * @brief Send a message over the radio
 *
 * Transmits data followed by a null terminator (0x00).
 * Blocks until transmission is complete.
 *
 * @param data Pointer to data buffer
 * @param len Length of data (1-254 bytes)
 * @return true if sent successfully, false on error
 */
bool radio_send(const uint8_t *data, uint8_t len);

/* ============================================================================
 * RX API
 * ============================================================================ */

/**
 * @brief Check if a received message is available
 *
 * @return true if at least one message is waiting in the RX queue
 */
bool radio_available(void);

/**
 * @brief Read a received message from the queue
 *
 * Copies the oldest message from the RX queue into the provided buffer.
 * The message is removed from the queue.
 *
 * @param buffer Destination buffer (must be at least RADIO_MAX_MESSAGE_LEN bytes)
 * @return Length of the message (0 if queue was empty)
 */
uint8_t radio_read(uint8_t *buffer);

/**
 * @brief Get the number of messages in the RX queue
 *
 * @return Number of messages waiting (0-255)
 */
uint8_t radio_rx_count(void);

/**
 * @brief Get pointer to the radio RX queue
 *
 * Provides direct access for the SPI slave to read from.
 *
 * @return Pointer to the RX message queue
 */
radio_message_queue_t *radio_get_rx_queue(void);

/* ============================================================================
 * UART Callbacks - Wire these from stm32g0xx_it.c / uart_callbacks.c
 * ============================================================================ */

/**
 * @brief Character Match event callback
 *
 * Called from uart_cm_handler when 0x00 is received on USART5.
 * Processes new bytes in the circular DMA buffer.
 *
 * @param huart UART handle
 * @param Size Current position in DMA buffer
 */
void radio_rx_event_callback(UART_HandleTypeDef *huart, uint16_t Size);

/**
 * @brief UART error callback
 *
 * Call from HAL_UART_ErrorCallback. Restarts circular DMA on error.
 *
 * @param huart UART handle
 */
void radio_uart_error_callback(UART_HandleTypeDef *huart);

/**
 * @brief Discard a partial message that has stalled without its terminator
 *
 * Call regularly from the main loop. The parser splits the radio stream on
 * 0x00 and otherwise waits forever, so a stray byte that never gets a
 * terminator is not dropped -- it lingers and is prepended to the next
 * genuine frame, breaking its decode. This expires that debris.
 */
void radio_rx_idle_check(void);

/* ============================================================================
 * AT Passthrough Session
 * ============================================================================
 *
 * An AT session turns UART5 from a message pipe into a raw byte pipe so the
 * SPI master can drive an RFD900x AT command session (+++ / ATSn= / AT&W).
 *
 * Two things change for the duration:
 *
 *  1. RX bypasses the message parser. Normal operation triggers solely on
 *     Character Match (0x00) and splits the stream into null-terminated
 *     messages -- but AT replies ("OK\r\n") contain no 0x00, so that
 *     interrupt would never fire. During a session the CM handler stands
 *     down and radio_at_poll() drains the circular DMA buffer by position
 *     instead, feeding every byte verbatim into a raw ring.
 *
 *  2. TX bypasses radio_send(). That function appends a 0x00 terminator,
 *     which would land inside the silence window "+++" depends on and break
 *     command-mode entry. radio_at_write() emits bytes exactly as given.
 *
 * The DMA itself is never reconfigured -- it runs circular and untouched
 * across the whole session; only the consumer changes.
 */

/**
 * @brief Begin an AT session
 *
 * Discards any bytes in flight, resets the raw rings, and stops the message
 * parser from consuming the DMA buffer. The caller is responsible for
 * keeping UART5 otherwise silent (the modem needs ~1 s of quiet before it
 * will accept "+++").
 */
void radio_at_session_begin(void);

/**
 * @brief End an AT session
 *
 * Drops any unread AT residue so it cannot be mistaken for a radio message,
 * and hands the DMA buffer back to the null-terminated message parser.
 */
void radio_at_session_end(void);

/**
 * @brief Whether an AT session is currently open
 */
bool radio_at_session_active(void);

/**
 * @brief Note master activity, restarting the session watchdog
 *
 * Call on every AT opcode. A session left open strands the modem serial in
 * raw mode and kills the downlink, so if the master goes away mid-session
 * (reset, brownout, SPI fault) the session must not persist -- see
 * RADIO_AT_SESSION_TIMEOUT_MS.
 */
void radio_at_touch(void);

/**
 * @brief Drain newly received UART5 bytes into the raw RX ring
 *
 * Call from the main loop while a session is open. Reads the DMA write
 * position directly rather than waiting on Character Match.
 */
void radio_at_poll(void);

/**
 * @brief Copy pending raw RX bytes without consuming them
 *
 * Split from the consume step so the SPI slave can stage a response in its
 * RXNE ISR and only commit once the master has actually clocked it out.
 *
 * @param dst Destination buffer
 * @param max Maximum bytes to copy
 * @return Number of bytes copied
 */
uint8_t radio_at_peek(uint8_t *dst, uint8_t max);

/**
 * @brief Consume n bytes previously reported by radio_at_peek()
 */
void radio_at_consume(uint8_t n);

/**
 * @brief Queue raw bytes for transmission to the modem (no terminator)
 *
 * Safe to call from interrupt context; bytes go out when radio_at_flush()
 * runs from the main loop.
 *
 * @return true if all bytes were queued, false if the TX ring was full
 */
bool radio_at_write(const uint8_t *data, uint8_t len);

/**
 * @brief Transmit any bytes queued by radio_at_write()
 *
 * Call from the main loop while a session is open. Blocking UART writes.
 */
void radio_at_flush(void);

#endif /* RADIO_DRIVER_H */
