/**
 * @file radio_driver.h
 * @brief Radio transceiver driver interface
 *
 * Provides a simple API for sending and receiving radio messages
 * via UART5. Messages are null-terminated on the wire.
 */

#ifndef RADIO_DRIVER_H
#define RADIO_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "radio_queue.h"

// ============================================================================
// CONFIGURATION
// ============================================================================

#define RADIO_MAX_MESSAGE_LEN   RADIO_MESSAGE_MAX_LEN  // 256 bytes

// ============================================================================
// PUBLIC API
// ============================================================================

/**
 * @brief Initialize radio driver
 *
 * Sets up UART5 for radio communication and initializes the RX queue.
 * Must be called once at startup after HAL_Init() and peripheral init.
 *
 * @param rx_queue Pointer to queue for storing received messages
 */
void radio_init(radio_message_queue_t *rx_queue);

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
radio_message_queue_t* radio_get_rx_queue(void);

// ============================================================================
// ISR INTERFACE (called from stm32g0xx_it.c)
// ============================================================================

/**
 * @brief Process received byte from radio UART
 *
 * Called from the UART5 interrupt handler when a byte is received.
 * Accumulates bytes until null terminator, then enqueues the message.
 *
 * @param byte The received byte
 */
void radio_rx_byte_handler(uint8_t byte);

#endif // RADIO_DRIVER_H
