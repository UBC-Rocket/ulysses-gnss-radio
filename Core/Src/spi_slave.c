/**
 * @file spi_slave_ll.c
 * @brief Low-level register-based SPI slave implementation for STM32G0B1
 *
 * Implements hybrid RXNE interrupt + DMA approach:
 * - First byte (command) captured by RXNE interrupt
 * - Remaining bytes handled by DMA (zero CPU overhead)
 * - Hardware NSS for automatic transaction framing
 * - EXTI on NSS rising edge for transaction completion
 *
 * Protocol: Pull mode only (master-initiated transactions)
 * Wire format: [CMD:1][DUMMY:4][PAYLOAD:0-256] = max 261 bytes
 */

#include "spi_slave.h"
#include <string.h>

// ============================================================================
// STATIC CONTEXT
// ============================================================================

static spi_slave_context_t ctx;

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================

static void spi_gpio_init(void);
static void spi_peripheral_init(void);
static void dmamux_init(void);
static void exti_nss_init(void);
static void spi_rx_dma_start(uint8_t *buf, uint16_t len);
static void spi_tx_dma_start(uint8_t *buf, uint16_t len);
static void spi_slave_arm(void);
static void spi_clear_errors(void);
static uint8_t radio_queue_count(radio_message_queue_t *q);

// ============================================================================
// INITIALIZATION FUNCTIONS
// ============================================================================

/**
 * @brief Configure GPIO pins for SPI2 alternate function
 */
static void spi_gpio_init(void) {
    // Enable GPIOB clock
    RCC->IOPENR |= RCC_IOPENR_GPIOBEN;

    // Configure PB12(NSS), PB13(SCK), PB14(MISO), PB15(MOSI) as alternate function mode
    // MODER: 00=input, 01=output, 10=AF, 11=analog
    GPIOB->MODER = (GPIOB->MODER
                    & ~((3 << (12*2)) | (3 << (13*2)) | (3 << (14*2)) | (3 << (15*2))))
                    | (2 << (12*2))   // PB12 NSS  → AF
                    | (2 << (13*2))   // PB13 SCK  → AF
                    | (2 << (14*2))   // PB14 MISO → AF
                    | (2 << (15*2));  // PB15 MOSI → AF

    // Pull-up on NSS (keeps line high when master not connected)
    // PUPDR: 00=no pull, 01=pull-up, 10=pull-down
    GPIOB->PUPDR = (GPIOB->PUPDR & ~(3 << (12*2))) | (1 << (12*2));

    // High speed on MISO and SCK (reduces edge ringing at high frequencies)
    // OSPEEDR: 00=low, 01=medium, 10=high, 11=very high
    GPIOB->OSPEEDR |= (3 << (14*2))  // MISO very high speed
                    | (3 << (13*2)); // SCK very high speed

    // Set alternate function numbers (AFRH for pins 8-15)
    GPIOB->AFR[1] = (GPIOB->AFR[1]
                    & ~((0xF << ((12-8)*4)) | (0xF << ((13-8)*4))
                       | (0xF << ((14-8)*4)) | (0xF << ((15-8)*4))))
                    | (SPI_AF_NUM << ((12-8)*4))    // NSS
                    | (SPI_AF_NUM << ((13-8)*4))    // SCK
                    | (SPI_AF_NUM << ((14-8)*4))    // MISO
                    | (SPI_AF_NUM << ((15-8)*4));   // MOSI

    // Configure IRQ pin (PB2) for push mode - output, initially high (inactive)
    GPIOB->MODER = (GPIOB->MODER & ~(3 << (2*2))) | (1 << (2*2));  // Output mode
    GPIOB->BSRR = IRQ_GPIO_PIN;  // Set high (inactive)
}

/**
 * @brief Configure SPI2 peripheral registers
 */
static void spi_peripheral_init(void) {
    // Enable SPI2 clock
    RCC->APBENR1 |= RCC_APBENR1_SPI2EN;

    // Disable SPI during configuration
    SPI2->CR1 &= ~SPI_CR1_SPE;

    // ── CR1 Configuration ──
    // All zeros = slave mode, CPOL=0, CPHA=0 (SPI Mode 0), MSB first, hardware NSS
    SPI2->CR1 = 0;
    // CPHA=0: data sampled on first (leading) clock edge
    // CPOL=0: clock idles low
    // MSTR=0: slave mode
    // SSM=0: hardware NSS management
    // LSBFIRST=0: MSB transmitted first

    // ── CR2 Configuration ──
    // This is the critical register for STM32G0 SPI operation
    SPI2->CR2 = (7 << SPI_CR2_DS_Pos)    // DS=0b0111 → 8-bit data frames
              | SPI_CR2_FRXTH             // CRITICAL: FIFO RX threshold = 1 byte (not 2!)
              | SPI_CR2_RXNEIE            // RXNE interrupt enabled (for command byte)
              | SPI_CR2_TXDMAEN;          // TX DMA enabled from start
    // Note: RXDMAEN is deliberately OFF - will be enabled after RXNE ISR reads command

    // Enable SPI peripheral
    SPI2->CR1 |= SPI_CR1_SPE;
}

/**
 * @brief Configure DMAMUX to route SPI requests to DMA channels
 */
static void dmamux_init(void) {
    // CRITICAL: DMAMUX channels are 0-indexed, DMA channels are 1-indexed!
    // DMA1_Channel1 → DMAMUX1_Channel0
    // DMA1_Channel2 → DMAMUX1_Channel1

    // Route SPI2_RX to DMA1 Channel 1 (DMAMUX Channel 0)
    DMAMUX1_Channel0->CCR = DMAREQ_SPI2_RX;  // 18

    // Route SPI2_TX to DMA1 Channel 2 (DMAMUX Channel 1)
    DMAMUX1_Channel1->CCR = DMAREQ_SPI2_TX;  // 19
}

/**
 * @brief Configure EXTI for NSS rising edge detection
 */
static void exti_nss_init(void) {
    // Route PB12 to EXTI line 12
    // EXTICR[3] handles lines 12-15
    // Each line gets 8 bits: 0x00=PortA, 0x01=PortB, 0x02=PortC, etc.
    EXTI->EXTICR[3] = (EXTI->EXTICR[3] & ~(0xFF << 0)) | (NSS_EXTI_PORT << 0);

    // Enable rising edge detection (NSS deassert = transaction end)
    EXTI->RTSR1 |= (1 << NSS_EXTI_LINE);

    // Unmask interrupt for line 12
    EXTI->IMR1 |= (1 << NSS_EXTI_LINE);

    // Enable EXTI4_15 interrupt in NVIC (covers lines 4-15)
    NVIC_SetPriority(EXTI4_15_IRQn, 2);  // Lower priority than SPI/DMA
    NVIC_EnableIRQ(EXTI4_15_IRQn);
}

// ============================================================================
// DMA HELPER FUNCTIONS
// ============================================================================

/**
 * @brief Start RX DMA: SPI2_DR → buf (peripheral → memory)
 *
 * @param buf Destination buffer
 * @param len Number of bytes to receive
 */
static void spi_rx_dma_start(uint8_t *buf, uint16_t len) {
    // Disable channel (required before reconfiguration)
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
    while (DMA1_Channel1->CCR & DMA_CCR_EN);  // Wait for disable to take effect

    // Clear all pending interrupt flags for channel 1
    DMA1->IFCR = DMA_IFCR_CGIF1;

    // Set addresses and count
    DMA1_Channel1->CPAR  = (uint32_t)&SPI2->DR;   // Source: SPI data register
    DMA1_Channel1->CMAR  = (uint32_t)buf;         // Destination: our buffer
    DMA1_Channel1->CNDTR = len;                   // Number of bytes

    // Configure channel
    // DIR=0: peripheral → memory (read from peripheral)
    // MINC=1: increment memory address after each transfer
    // PSIZE=00: 8-bit peripheral access (CRITICAL for 8-bit SPI frames)
    // MSIZE=00: 8-bit memory access
    // PL=10: high priority (RX more critical than TX to avoid overrun)
    // TCIE=1: transfer complete interrupt enabled
    // TEIE=1: transfer error interrupt enabled
    DMA1_Channel1->CCR = DMA_CCR_MINC         // Memory increment
                       | DMA_CCR_TCIE         // Transfer complete interrupt
                       | DMA_CCR_TEIE         // Transfer error interrupt
                       | (2 << DMA_CCR_PL_Pos); // High priority

    // Enable the channel (transfer starts on next SPI RXNE → DMA request)
    DMA1_Channel1->CCR |= DMA_CCR_EN;
}

/**
 * @brief Start TX DMA: buf → SPI2_DR (memory → peripheral)
 *
 * @param buf Source buffer
 * @param len Number of bytes to transmit
 */
static void spi_tx_dma_start(uint8_t *buf, uint16_t len) {
    // Disable channel
    DMA1_Channel2->CCR &= ~DMA_CCR_EN;
    while (DMA1_Channel2->CCR & DMA_CCR_EN);

    // Clear flags
    DMA1->IFCR = DMA_IFCR_CGIF2;

    // Set addresses and count
    DMA1_Channel2->CPAR  = (uint32_t)&SPI2->DR;   // Destination: SPI data register
    DMA1_Channel2->CMAR  = (uint32_t)buf;         // Source: our buffer
    DMA1_Channel2->CNDTR = len;                   // Number of bytes

    // Configure channel
    // DIR=1: memory → peripheral (write to peripheral)
    // MINC=1: increment memory address
    // PSIZE=00: 8-bit peripheral access
    // MSIZE=00: 8-bit memory access
    // PL=01: medium priority (TX less critical than RX)
    // No interrupts needed for TX
    DMA1_Channel2->CCR = DMA_CCR_DIR          // Memory → peripheral
                       | DMA_CCR_MINC         // Memory increment
                       | (1 << DMA_CCR_PL_Pos); // Medium priority

    // Enable channel
    DMA1_Channel2->CCR |= DMA_CCR_EN;
}

/**
 * @brief Clear SPI error flags
 *
 * Drains RX FIFO and clears OVR/UDR flags
 */
static void spi_clear_errors(void) {
    volatile uint32_t dummy;

    // Drain RX FIFO (up to 4 bytes deep on STM32G0)
    // FRLVL bits indicate FIFO level: 00=empty, 01=1/4, 10=1/2, 11=full
    while (SPI2->SR & SPI_SR_FRLVL) {
        // 8-bit read to pop exactly one byte
        dummy = *(volatile uint8_t *)&SPI2->DR;
    }

    // Read SR to clear OVR flag
    dummy = SPI2->SR;
    (void)dummy;
}

/**
 * @brief Get radio queue message count
 */
static uint8_t radio_queue_count(radio_message_queue_t *q) {
    if (radio_message_queue_empty(q)) {
        return 0;
    }
    return (q->head - q->tail + RADIO_MESSAGE_QUEUE_LEN) % RADIO_MESSAGE_QUEUE_LEN;
}

// ============================================================================
// ARM FUNCTION (Transaction Preparation)
// ============================================================================

/**
 * @brief Arm SPI slave for next transaction
 *
 * Stops all DMA, clears errors, zeros TX buffer, and restarts DMA
 * in the correct initial state (RXNEIE enabled, RXDMAEN disabled).
 */
static void spi_slave_arm(void) {
    // ── Step 1: Stop everything ──
    SPI2->CR1 &= ~SPI_CR1_SPE;  // Disable SPI (flushes FIFOs on re-enable)
    SPI2->CR2 &= ~(SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN | SPI_CR2_RXNEIE);

    // Disable both DMA channels
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
    DMA1_Channel2->CCR &= ~DMA_CCR_EN;

    // Clear all DMA flags
    DMA1->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CGIF2;

    // Clear SPI error flags
    spi_clear_errors();

    // ── Step 2: Reset state ──
    ctx.state = SPI_STATE_IDLE;
    ctx.current_cmd = 0;
    ctx.payload_processed = false;

    // ── Step 3: Prepare TX buffer ──
    // Zero the entire buffer. For read commands, response data will be
    // patched in at offset CMD_OVERHEAD by the RXNE ISR.
    memset(ctx.tx_buf, 0x00, sizeof(ctx.tx_buf));

    // ── Step 4: Start TX DMA ──
    // TX DMA runs from the start, streaming from tx_buf for the full
    // maximum transaction size. During command byte and dummy bytes,
    // it sends zeros (which master ignores). During response region,
    // it sends whatever we patched in (or zeros for write commands).
    spi_tx_dma_start(ctx.tx_buf, MAX_TRANSACTION_SIZE);

    // ── Step 5: Start RX DMA (but gated) ──
    // RX DMA is configured for all bytes after the command byte.
    // It points at rx_buf starting at index 1 (byte 0 is the command,
    // which the RXNE ISR reads manually).
    // BUT: we don't enable RXDMAEN yet. The DMA channel is armed and waiting,
    // but the SPI won't send it requests until we flip RXDMAEN on.
    spi_rx_dma_start(&ctx.rx_buf[1], MAX_TRANSACTION_SIZE - 1);

    // ── Step 6: Configure SPI CR2 ──
    SPI2->CR2 = (7 << SPI_CR2_DS_Pos)    // 8-bit data frames
              | SPI_CR2_FRXTH            // 1-byte FIFO threshold
              | SPI_CR2_RXNEIE           // RXNE interrupt for command byte
              | SPI_CR2_TXDMAEN;         // TX DMA active from start
    // Note: RXDMAEN deliberately OFF - enabled after RXNE ISR

    // ── Step 7: Enable SPI ──
    SPI2->CR1 |= SPI_CR1_SPE;
}

// ============================================================================
// INTERRUPT HANDLERS
// ============================================================================

/**
 * @brief SPI2 RXNE Interrupt Handler
 *
 * This fires when the first byte (command) arrives and RXNE flag is set.
 * This is the heart of the hybrid design:
 * 1. Read command byte manually (8-bit DR read)
 * 2. Parse command and patch tx_buf if needed (read commands)
 * 3. Disable RXNEIE
 * 4. Enable RXDMAEN (hand off to DMA for remaining bytes)
 *
 * All of this must complete within 4 byte-times (the dummy byte window).
 */
void spi_slave_spi2_irq_handler(void) {
    // Verify this is an RXNE interrupt
    if ((SPI2->SR & SPI_SR_RXNE) && (SPI2->CR2 & SPI_CR2_RXNEIE)) {

        // ── Read command byte ──
        // CRITICAL: 8-bit read to pop exactly one byte from RX FIFO
        // This also clears the RXNE flag
        uint8_t cmd = *(volatile uint8_t *)&SPI2->DR;
        ctx.current_cmd = cmd;
        ctx.rx_buf[0] = cmd;

        // ── Disable RXNE interrupt ──
        // We got the command. Don't want RXNE firing for every subsequent byte.
        SPI2->CR2 &= ~SPI_CR2_RXNEIE;

        // ── Process command and patch TX buffer ──
        // The master is clocking dummy bytes 1-4 right now.
        // We have ~4 byte-times to complete this before real data starts.

        switch (cmd) {

        // ────────────────────────────────────
        // READ COMMANDS: Slave sends data to master
        // Patch tx_buf with response data starting at CMD_OVERHEAD
        // ────────────────────────────────────

        case CMD_RADIO_RX_LIFO: {
            // Get pointer to most recent radio message (don't pop yet)
            uint8_t *msg_ptr;
            if (radio_message_queue_head_pointer(ctx.radio_queue, &msg_ptr)) {
                memcpy(&ctx.tx_buf[CMD_OVERHEAD], msg_ptr, 256);
            }
            // If queue empty, tx_buf already zeroed
            break;
        }

        case CMD_RADIO_RX_FIFO: {
            // Get pointer to oldest radio message (don't pop yet)
            uint8_t *msg_ptr;
            if (radio_message_queue_tail_pointer(ctx.radio_queue, &msg_ptr)) {
                memcpy(&ctx.tx_buf[CMD_OVERHEAD], msg_ptr, 256);
            }
            break;
        }

        case CMD_RADIO_RXBUF_LEN: {
            // Return number of messages in radio queue (0-255)
            ctx.tx_buf[CMD_OVERHEAD] = radio_queue_count(ctx.radio_queue);
            break;
        }

        case CMD_GPS_RX: {
            // Get pointer to oldest GPS sample (don't pop yet)
            uint8_t *gps_ptr;
            if (gps_sample_queue_tail_pointer(ctx.gps_queue, &gps_ptr)) {
                memcpy(&ctx.tx_buf[CMD_OVERHEAD], gps_ptr, GPS_SAMPLE_SIZE);
            }
            break;
        }

        // ────────────────────────────────────
        // WRITE COMMANDS: Master sends data to slave
        // No action needed - RX DMA will capture payload
        // ────────────────────────────────────

        case CMD_RADIO_TX:
            // Payload will land at rx_buf[CMD_OVERHEAD..CMD_OVERHEAD+255]
            // via RX DMA. We'll process it in DMA TC ISR or EXTI ISR.
            break;

        default:
            // Unknown command - DMA will still capture bytes,
            // but we'll ignore them
            ctx.unknown_commands++;
            break;
        }

        // ── Enable RX DMA ──
        // This is the critical handoff from interrupt-driven to DMA-driven.
        // From this point on, all incoming bytes trigger DMA requests instead
        // of RXNE interrupts. The DMA silently handles all remaining bytes.
        SPI2->CR2 |= SPI_CR2_RXDMAEN;

        ctx.state = SPI_STATE_ACTIVE;
    }
}

/**
 * @brief DMA1 Channel 1 (RX) Interrupt Handler
 *
 * Fires when RX DMA transfer completes or errors.
 * Transfer complete means all MAX_TRANSACTION_SIZE-1 bytes have been received.
 * This only happens for full-length transactions (261 bytes total).
 */
void spi_slave_dma1_ch1_irq_handler(void) {

    // ── Transfer Complete ──
    if (DMA1->ISR & DMA_ISR_TCIF1) {
        DMA1->IFCR = DMA_IFCR_CTCIF1;  // Clear flag

        // All 260 bytes received (after command byte).
        // For write commands, process the payload.
        if (ctx.current_cmd == CMD_RADIO_TX && !ctx.payload_processed) {
            // Payload is at rx_buf[CMD_OVERHEAD..CMD_OVERHEAD+255]
            // (byte 0 is cmd from RXNE ISR, bytes 1-4 are dummy, bytes 5-260 are payload)
            if (ctx.radio_queue) {
                radio_message_enqueue(256, &ctx.rx_buf[CMD_OVERHEAD], ctx.radio_queue);
                ctx.payload_processed = true;
            }
        }

        // Transaction cleanup will be handled by NSS rising edge EXTI
    }

    // ── Transfer Error ──
    if (DMA1->ISR & DMA_ISR_TEIF1) {
        DMA1->IFCR = DMA_IFCR_CTEIF1;  // Clear flag
        ctx.transfer_errors++;

        // DMA bus error - reset everything
        spi_slave_arm();
    }
}

/**
 * @brief NSS EXTI Handler (line 12)
 *
 * Fires when NSS rising edge detected (line 12).
 * This is the universal "transaction complete" signal.
 * Handles both full transactions (DMA TC fired) and short transactions
 * (master ended early).
 */
void spi_slave_nss_exti_handler(void) {

    // Check if this is NSS line 12 rising edge
    if (EXTI->RPR1 & (1 << NSS_EXTI_LINE)) {
        // Clear pending flag (write 1 to clear)
        EXTI->RPR1 = (1 << NSS_EXTI_LINE);

        // ── Handle based on state ──
        switch (ctx.state) {

        case SPI_STATE_IDLE:
            // NSS rose before we even got a command byte.
            // Glitch or aborted transaction. Just re-arm.
            break;

        case SPI_STATE_ACTIVE:
            // Normal transaction end.
            // For write commands, check if we got the full payload.
            if (ctx.current_cmd == CMD_RADIO_TX && !ctx.payload_processed) {
                // Check how many bytes actually transferred
                uint16_t remaining = DMA1_Channel1->CNDTR;
                uint16_t total_rx_bytes = (MAX_TRANSACTION_SIZE - 1) - remaining;

                // total_rx_bytes includes dummy bytes + payload
                // Payload bytes = total_rx_bytes - PULL_DUMMY_BYTES
                uint16_t payload_bytes = 0;
                if (total_rx_bytes > PULL_DUMMY_BYTES) {
                    payload_bytes = total_rx_bytes - PULL_DUMMY_BYTES;
                }

                if (payload_bytes == 256) {
                    // Full 256-byte payload received
                    // If DMA TC already processed it, this is a no-op
                    if (ctx.radio_queue) {
                        radio_message_enqueue(256, &ctx.rx_buf[CMD_OVERHEAD], ctx.radio_queue);
                        ctx.payload_processed = true;
                    }
                } else if (payload_bytes > 0) {
                    // Partial payload - master ended transaction early
                    // Could enqueue partial message or discard as error
                    // For now, discard
                }
            }

            // Check for overrun errors
            if (SPI2->SR & SPI_SR_OVR) {
                ctx.overrun_errors++;
            }

            ctx.transactions_completed++;
            break;

        default:
            // Unexpected state
            break;
        }

        // ── Re-arm for next transaction ──
        spi_slave_arm();
    }

    // Handle other EXTI lines if needed (not used in our design)
}

// ============================================================================
// PUBLIC API IMPLEMENTATION
// ============================================================================

/**
 * @brief Initialize SPI slave in pull mode
 */
void spi_slave_init(radio_message_queue_t *radio_queue, gps_sample_queue_t *gps_queue) {
    // Clear context structure
    memset(&ctx, 0, sizeof(ctx));

    // Store queue pointers
    ctx.radio_queue = radio_queue;
    ctx.gps_queue = gps_queue;

    // ── Enable peripheral clocks ──
    RCC->IOPENR  |= RCC_IOPENR_GPIOBEN;    // GPIO port B
    RCC->AHBENR  |= RCC_AHBENR_DMA1EN;     // DMA1
    RCC->APBENR1 |= RCC_APBENR1_SPI2EN;    // SPI2

    // ── Initialize peripherals ──
    spi_gpio_init();
    dmamux_init();
    spi_peripheral_init();
    exti_nss_init();

    // ── Enable interrupts in NVIC ──
    // SPI2/3 RXNE interrupt (highest priority - must respond fast)
    // STM32G0B1 has combined SPI2_3_IRQn
    NVIC_SetPriority(SPI2_3_IRQn, 0);
    NVIC_EnableIRQ(SPI2_3_IRQn);

    // DMA1 Channel 1 interrupt (high priority)
    NVIC_SetPriority(DMA1_Channel1_IRQn, 1);
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    // EXTI already enabled in exti_nss_init()

    // ── Arm for first transaction ──
    spi_slave_arm();
}

/**
 * @brief Get current state (for debugging)
 */
spi_slave_state_t spi_slave_get_state(void) {
    return ctx.state;
}

/**
 * @brief Get context pointer (for debugging)
 */
const spi_slave_context_t* spi_slave_get_context(void) {
    return &ctx;
}

/**
 * @brief Reset error counters
 */
void spi_slave_reset_errors(void) {
    ctx.overrun_errors = 0;
    ctx.transfer_errors = 0;
    ctx.unknown_commands = 0;
    ctx.collisions_detected = 0;
}

// ============================================================================
// PUSH MODE STUBS (for future implementation)
// ============================================================================

void spi_slave_assert_irq(void) {
    // TODO: Implement when push mode is enabled
    // HAL_GPIO_WritePin(IRQ_GPIO_PORT, IRQ_GPIO_PIN, GPIO_PIN_RESET);
    // ctx.irq_asserted = true;
    // ctx.irq_assert_timestamp = get_time_us();
}

void spi_slave_deassert_irq(void) {
    // TODO: Implement when push mode is enabled
    // HAL_GPIO_WritePin(IRQ_GPIO_PORT, IRQ_GPIO_PIN, GPIO_PIN_SET);
    // ctx.irq_asserted = false;
}

void spi_slave_tick(void) {
    // TODO: Implement when push mode is enabled
    // Check for pending data in queues
    // If data available and state == IDLE, assert IRQ
}
