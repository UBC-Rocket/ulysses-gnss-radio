#ifndef SPI_SLAVE_H
#define SPI_SLAVE_H

/**
 * @file spi_slave.h
 * @brief SPI slave driver for STM32G0B1 - Pull and Push mode support
 *
 * Hardware Configuration:
 * - SPI1 on PA4(NSS), PA5(SCK), PA6(MISO), PA7(MOSI)
 * - IRQ output on PB2 (active high, directly driving GPIO for push mode)
 * - DMA1 Channel 1 (RX), Channel 2 (TX)
 * - EXTI line 4 for NSS transaction end detection
 *
 * Protocol:
 * - Pull mode: Master sends [CMD:1][DUMMY:4][PAYLOAD:0-256]
 * - Push mode: Slave asserts IRQ (high), sends [TYPE:1][PAYLOAD:N]
 */

#include <stdint.h>
#include <stdbool.h>
#include "stm32g0xx.h"
#include "protocol_config.h"
#include "radio_queue.h"
#include "gps_nema_queue.h"

// ============================================================================
// HARDWARE DEFINITIONS
// ============================================================================

// SPI Peripheral Selection
#define SPI_PERIPHERAL          SPI1

// DMA Configuration
#define DMA_CONTROLLER          DMA1
#define RX_DMA_CHANNEL          DMA1_Channel1
#define TX_DMA_CHANNEL          DMA1_Channel2
#define RX_DMAMUX_CHANNEL       DMAMUX1_Channel0
#define TX_DMAMUX_CHANNEL       DMAMUX1_Channel1

// DMAMUX Request IDs (RM0444 Table 59)
#define DMAREQ_SPI_RX           16  // SPI1_RX
#define DMAREQ_SPI_TX           17  // SPI1_TX

// ============================================================================
// GPIO PIN DEFINITIONS
// ============================================================================

#define SPI_NSS_PIN             GPIO_PIN_4   // PA4
#define SPI_SCK_PIN             GPIO_PIN_5   // PA5
#define SPI_MISO_PIN            GPIO_PIN_6   // PA6
#define SPI_MOSI_PIN            GPIO_PIN_7   // PA7
#define SPI_GPIO_PORT           GPIOA

// For STM32G0B1, SPI1 is AF0 on PA4-7
#define SPI_AF_NUM              0

// NSS EXTI Configuration
#define NSS_EXTI_LINE           4
#define NSS_EXTI_PORT           0x00  // Port A = 0x00 (for EXTICR register)

// IRQ Line for Push Mode (PB2)
#define IRQ_GPIO_PORT           GPIOB
#define IRQ_GPIO_PIN            GPIO_PIN_2

// ============================================================================
// STATE MACHINE
// ============================================================================

typedef enum {
    SPI_STATE_IDLE,          // No transaction, RX DMA armed, waiting for NSS or data
    SPI_STATE_HAVE_DATA,     // Push mode: IRQ asserted, TX+RX DMA armed, waiting for NSS
    SPI_STATE_ACTIVE,        // Mid-transaction (both TX and RX DMA running)
} spi_slave_state_t;

// ============================================================================
// PUSH MODE CONFIGURATION
// ============================================================================

// Minimum bytes to sample when checking if RX buffer has real data
#define RX_PATTERN_SAMPLE_SIZE  16

// Threshold: if this many bytes are 0x00 or 0xFF, consider it dummy data
#define RX_PATTERN_THRESHOLD    14

// ============================================================================
// CONTEXT STRUCTURE
// ============================================================================

typedef struct {
    // ── State ──
    volatile spi_slave_state_t state;
    volatile uint8_t current_cmd;
    volatile bool payload_processed;  // Prevents double-processing in DMA TC + EXTI

    // ── Buffers ──
    // NOTE: Aligned to 32-byte boundary for optimal DMA performance
    uint8_t tx_buf[MAX_TRANSACTION_SIZE] __attribute__((aligned(32)));
    uint8_t rx_buf[MAX_TRANSACTION_SIZE] __attribute__((aligned(32)));

    // ── Queue Pointers ──
    radio_message_queue_t *radio_queue;
    gps_sample_queue_t *gps_queue;

    // ── Statistics & Diagnostics ──
    volatile uint32_t transactions_completed;
    volatile uint32_t overrun_errors;
    volatile uint32_t transfer_errors;
    volatile uint32_t unknown_commands;

    // ── Push Mode State ──
    volatile bool irq_asserted;           // True when IRQ line is asserted (active low)
    volatile uint8_t pending_push_type;   // PUSH_TYPE_RADIO or PUSH_TYPE_GPS (0 = none)
    volatile uint16_t tx_dma_length;      // Length of current TX DMA transfer
    volatile uint32_t push_transactions;  // Count of push mode transactions
    volatile uint32_t master_tx_received; // Count of master TX while pushing (simultaneous)

} spi_slave_context_t;

// ============================================================================
// PUBLIC API
// ============================================================================

/**
 * @brief Initialize SPI slave peripheral in pull mode
 *
 * Configures SPI1 as slave with hardware NSS, sets up DMA channels,
 * and arms for the first transaction.
 *
 * @param radio_queue Pointer to radio message queue
 * @param gps_queue Pointer to GPS sample queue
 */
void spi_slave_init(radio_message_queue_t *radio_queue, gps_sample_queue_t *gps_queue);

/**
 * @brief Get current state (for debugging)
 */
spi_slave_state_t spi_slave_get_state(void);

/**
 * @brief Get statistics structure (for debugging)
 */
const spi_slave_context_t* spi_slave_get_context(void);

/**
 * @brief Reset error counters
 */
void spi_slave_reset_errors(void);

// ============================================================================
// INTERRUPT HANDLERS (must be called from stm32g0xx_it.c)
// ============================================================================
//
// These functions contain the actual interrupt logic. They are called by the
// corresponding IRQHandler functions in stm32g0xx_it.c.
//
// NOTE: The functions are named with _handler suffix to avoid duplicate symbol
// errors with the standard IRQHandler names which are defined in stm32g0xx_it.c

/**
 * @brief SPI1 RXNE interrupt logic
 * Called when command byte arrives. This is the critical handoff point.
 * Must be called from SPI1_IRQHandler() in stm32g0xx_it.c
 */
void spi_slave_spi1_irq_handler(void);

/**
 * @brief DMA1 Channel 1 (RX) interrupt logic
 * Called when RX DMA transfer completes or errors
 * Must be called from DMA1_Channel1_IRQHandler() in stm32g0xx_it.c
 */
void spi_slave_dma1_ch1_irq_handler(void);

/**
 * @brief NSS EXTI interrupt logic (line 4)
 * Called when NSS rising edge detected (transaction end)
 * Must be called from EXTI4_15_IRQHandler() in stm32g0xx_it.c
 */
void spi_slave_nss_exti_handler(void);

// ============================================================================
// PUSH MODE API
// ============================================================================

/**
 * @brief Assert IRQ line to master (push mode)
 *
 * Sets PB2 high (active high) to signal the master that
 * the slave has data ready to send.
 */
void spi_slave_assert_irq(void);

/**
 * @brief Deassert IRQ line to master (push mode)
 *
 * Sets PB2 low (inactive) after transaction completes.
 */
void spi_slave_deassert_irq(void);

/**
 * @brief Check for pending push data and assert IRQ if needed
 *
 * Call this periodically from the main loop. When data is available
 * (radio RX message or GPS fix) and the slave is idle, this function
 * prepares the TX buffer, arms DMAs, and asserts IRQ.
 */
void spi_slave_tick(void);

/**
 * @brief Check if RX buffer contains valid data (not dummy bytes)
 *
 * Used to detect if master sent a radio TX message during a push
 * transaction. Checks for non-zero/non-0xFF pattern.
 *
 * @return true if RX buffer appears to contain real data
 */
bool spi_slave_rx_has_valid_data(void);

#endif // SPI_SLAVE_H
