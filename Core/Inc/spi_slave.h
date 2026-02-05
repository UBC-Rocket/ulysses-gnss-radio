#ifndef SPI_SLAVE_H
#define SPI_SLAVE_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "radio_queue.h"
#include "gps_queue.h"
#include "protocol_config.h"

// ============================================================================
// SPI SLAVE PROTOCOL HANDLER
// Pull & Push Mode Support with Collision Detection
// ============================================================================

// ----------------------------------------------------------------------------
// State Machine States
// ----------------------------------------------------------------------------
typedef enum {
    SPI_STATE_WAIT_CONFIG,    // Initial state, waiting for configuration frame
    SPI_STATE_IDLE,           // Armed for RX, waiting for master or tick
    SPI_STATE_PULL_CMD,       // Receiving/processing pull mode command
    SPI_STATE_PULL_RESPONSE,  // Sending pull mode response data
    SPI_STATE_HAVE_DATA,      // Push mode: data ready, IRQ asserted
    SPI_STATE_PUSH_TYPE,      // Push mode: sending type byte
    SPI_STATE_PUSH_PAYLOAD,   // Push mode: sending payload data
    SPI_STATE_RX_RADIO,       // Receiving radio TX from master
    SPI_STATE_COLLISION,      // Detected race condition, backing off
    SPI_STATE_COMPLETE,       // Transaction complete, cleanup
} spi_handler_state_t;

// ----------------------------------------------------------------------------
// Board State (Error Reporting)
// ----------------------------------------------------------------------------
typedef enum {
    BOARD_STATE_OK = 0,
    BOARD_STATE_ERROR_DMA,
    BOARD_STATE_ERROR_OVERFLOW,
    BOARD_STATE_ERROR_TIMEOUT,
} board_state_t;

// ----------------------------------------------------------------------------
// Transaction Context
// ----------------------------------------------------------------------------
typedef struct {
    // Current command/type being processed
    uint8_t command;              // Pull mode command byte
    uint8_t push_type;            // Push mode data type

    // Buffer pointers for current transaction
    uint8_t *tx_buffer;
    uint8_t *rx_buffer;
    uint16_t transfer_size;

    // Transaction state tracking
    bool dummy_phase_complete;    // Pull mode: finished cmd + dummy bytes
    uint8_t phase;                // Multi-phase transaction tracking
} spi_transaction_ctx_t;

// ----------------------------------------------------------------------------
// Main SPI Handler Structure
// ----------------------------------------------------------------------------
typedef struct {
    // Queues
    radio_message_queue_t *radio_queue;
    gps_sample_queue_t *gps_queue;

    // State
    spi_handler_state_t state;
    spi_protocol_mode_t mode;       // Pull or Push mode
    config_frame_t config;          // Startup configuration
    board_state_t board_state;      // Error state

    // Transaction context
    spi_transaction_ctx_t transaction;

    // Collision detection
    volatile bool irq_asserted;
    volatile uint32_t irq_assert_timestamp;  // In microseconds

    // Ping-pong RX buffers
    uint8_t *rx_active_buffer;
    uint8_t *rx_complete_buffer;

    // Callback for forwarding radio messages to UART
    void (*radio_tx_callback)(uint8_t *data, uint16_t len);

    // Statistics (optional, for debugging)
    uint32_t transactions_completed;
    uint32_t collisions_detected;
    uint32_t errors;
} spi_handler_t;

// ----------------------------------------------------------------------------
// Public API
// ----------------------------------------------------------------------------

/**
 * @brief Initialize SPI slave protocol handler
 *
 * @param radio_queue Pointer to radio message queue
 * @param gps_queue Pointer to GPS sample queue
 *
 * Sets up DMA, GPIO, and arms RX for configuration frame.
 */
void spi_slave_init(radio_message_queue_t *radio_queue, gps_sample_queue_t *gps_queue);

/**
 * @brief Tick function called from main loop
 *
 * Checks for pending data and transitions IDLE -> HAVE_DATA in push mode.
 * Non-blocking, call frequently (1 kHz recommended).
 */
void spi_slave_tick(void);

/**
 * @brief Set callback for forwarding received radio messages to UART
 *
 * @param callback Function pointer to radio TX callback
 */
void spi_slave_set_radio_callback(void (*callback)(uint8_t *data, uint16_t len));

/**
 * @brief Set board error state
 *
 * @param state Error state to set
 */
void spi_slave_set_board_state(board_state_t state);

/**
 * @brief Get current protocol state (for debugging)
 *
 * @return spi_handler_state_t Current state
 */
spi_handler_state_t spi_slave_get_state(void);

/**
 * @brief Get current protocol mode (for debugging)
 *
 * @return spi_protocol_mode_t Current mode (Pull/Push)
 */
spi_protocol_mode_t spi_slave_get_mode(void);

// ----------------------------------------------------------------------------
// Interrupt Callbacks (called from stm32g0xx_it.c)
// ----------------------------------------------------------------------------

/**
 * @brief SPI TX/RX complete callback (DMA complete)
 *
 * Called when a full-duplex DMA transfer completes.
 * Processes received data, transitions state, arms next transfer.
 */
void spi_slave_txrx_complete(void);

/**
 * @brief CS (NSS) falling edge EXTI callback
 *
 * Called when CS line goes low (master starting transaction).
 * Handles collision detection (checks if within t_race of IRQ assertion).
 */
void spi_slave_cs_falling_edge(void);

/**
 * @brief CS (NSS) rising edge EXTI callback
 *
 * Called when CS line goes high (master ending transaction).
 * Signals end of transaction, triggers cleanup.
 */
void spi_slave_cs_rising_edge(void);

// ----------------------------------------------------------------------------
// Internal Helpers (declared for testing, not part of public API)
// ----------------------------------------------------------------------------

/**
 * @brief Assert DATA_READY IRQ line (active low)
 */
void spi_slave_assert_irq(void);

/**
 * @brief Deassert DATA_READY IRQ line
 */
void spi_slave_deassert_irq(void);

/**
 * @brief Transition to new state
 *
 * @param new_state State to transition to
 */
void spi_slave_transition(spi_handler_state_t new_state);

/**
 * @brief Arm DMA for next transaction based on current state and mode
 */
void spi_slave_arm_dma(void);

#endif // SPI_SLAVE_H
