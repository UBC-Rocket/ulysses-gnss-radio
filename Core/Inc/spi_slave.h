#ifndef SPI_SLAVE_LL_H
#define SPI_SLAVE_LL_H

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
#define SPI_PERIPHERAL          SPI2

// DMA Configuration
#define DMA_CONTROLLER          DMA1
#define RX_DMA_CHANNEL          DMA1_Channel1
#define TX_DMA_CHANNEL          DMA1_Channel2
#define RX_DMAMUX_CHANNEL       DMAMUX1_Channel0
#define TX_DMAMUX_CHANNEL       DMAMUX1_Channel1

// DMAMUX Request IDs (RM0444 Table 59)
#define DMAREQ_SPI2_RX          18
#define DMAREQ_SPI2_TX          19

// ============================================================================
// GPIO PIN DEFINITIONS
// ============================================================================

#define SPI_NSS_PIN             GPIO_PIN_12  // PB12
#define SPI_SCK_PIN             GPIO_PIN_13  // PB13
#define SPI_MISO_PIN            GPIO_PIN_14  // PB14
#define SPI_MOSI_PIN            GPIO_PIN_15  // PB15
#define SPI_GPIO_PORT           GPIOB

// For STM32G0B1, SPI2 is typically AF0 on PB12-15, but VERIFY THIS!
#define SPI_AF_NUM              0

// NSS EXTI Configuration
#define NSS_EXTI_LINE           12
#define NSS_EXTI_PORT           0x01  // Port B = 0x01 (for EXTICR register)

// IRQ Line for Push Mode (PB2)
#define IRQ_GPIO_PORT           GPIOB
#define IRQ_GPIO_PIN            GPIO_PIN_2

// ============================================================================
// STATE MACHINE
// ============================================================================

typedef enum {
    SPI_STATE_IDLE,          // Waiting for command byte (RXNEIE enabled)
    SPI_STATE_ACTIVE,        // Mid-transaction (DMA handling remaining bytes)

    // Push mode states (not used yet, prepared for future)
    // SPI_STATE_HAVE_DATA,     // Push mode: data ready, IRQ asserted
    // SPI_STATE_PUSH_TYPE,     // Push mode: sending type byte
    // SPI_STATE_PUSH_PAYLOAD,  // Push mode: sending payload
    // SPI_STATE_COLLISION,     // Push mode: collision detected
} spi_slave_state_t;

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

    // ── Push Mode Infrastructure (prepared, not used yet) ──
    volatile bool irq_asserted;
    uint32_t irq_assert_timestamp;
    uint32_t collisions_detected;

} spi_slave_context_t;

// ============================================================================
// PUBLIC API
// ============================================================================

/**
 * @brief Initialize SPI slave peripheral in pull mode
 *
 * Configures SPI2 as slave with hardware NSS, sets up DMA channels,
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
 * @brief SPI2 RXNE interrupt logic
 * Called when command byte arrives. This is the critical handoff point.
 * Must be called from SPI2_IRQHandler() in stm32g0xx_it.c
 */
void spi_slave_spi2_irq_handler(void);

/**
 * @brief DMA1 Channel 1 (RX) interrupt logic
 * Called when RX DMA transfer completes or errors
 * Must be called from DMA1_Channel1_IRQHandler() in stm32g0xx_it.c
 */
void spi_slave_dma1_ch1_irq_handler(void);

/**
 * @brief NSS EXTI interrupt logic (line 12)
 * Called when NSS rising edge detected (transaction end)
 * Must be called from EXTI4_15_IRQHandler() in stm32g0xx_it.c
 */
void spi_slave_nss_exti_handler(void);

// ============================================================================
// PUSH MODE API (stubs for future implementation)
// ============================================================================

/**
 * @brief Assert IRQ line to master (push mode)
 * TODO: Implement when push mode is enabled
 */
void spi_slave_assert_irq(void);

/**
 * @brief Deassert IRQ line to master (push mode)
 * TODO: Implement when push mode is enabled
 */
void spi_slave_deassert_irq(void);

/**
 * @brief Check for pending push data and assert IRQ if needed (push mode)
 * TODO: Implement when push mode is enabled
 */
void spi_slave_tick(void);

#endif // SPI_SLAVE_LL_H
