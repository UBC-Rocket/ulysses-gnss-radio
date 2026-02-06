/**
 * @file spi_slave.c
 * @brief Low-level register-based SPI slave implementation for STM32G0B1
 *
 * Hardware Configuration:
 * - SPI1 on PA4(NSS), PA5(SCK), PA6(MISO), PA7(MOSI)
 * - IRQ output on PB2 (active high, for push mode)
 * - DMA1 Channel 1 (RX), Channel 2 (TX)
 *
 * Implements hybrid RXNE interrupt + DMA approach:
 * - First byte (command) captured by RXNE interrupt
 * - Remaining bytes handled by DMA (zero CPU overhead)
 * - Hardware NSS for automatic transaction framing
 * - EXTI on NSS rising edge (line 4) for transaction completion
 *
 * Protocol Modes:
 * - Pull mode: Master-initiated transactions [CMD:1][DUMMY:4][PAYLOAD:0-256]
 * - Push mode: Slave-initiated via IRQ [TYPE:1][PAYLOAD:N]
 */

#include "spi_slave.h"
// #include "gps_driver.h"  // TODO: Add when GPS driver is implemented
#include <string.h>
#ifdef DEBUG
#include "debug_uart.h"
#endif

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
static void spi_slave_arm_push(void);
static void spi_clear_errors(void);
static uint8_t radio_queue_count(radio_message_queue_t *q);
static void spi_slave_prepare_push(uint8_t push_type);

// ============================================================================
// INITIALIZATION FUNCTIONS
// ============================================================================

/**
 * @brief Configure GPIO pins for SPI1 alternate function
 */
static void spi_gpio_init(void) {
    // Enable GPIOA and GPIOB clocks
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN;

    // Configure PA4(NSS), PA5(SCK), PA6(MISO), PA7(MOSI) as alternate function mode
    // MODER: 00=input, 01=output, 10=AF, 11=analog
    GPIOA->MODER = (GPIOA->MODER
                    & ~((3 << (4*2)) | (3 << (5*2)) | (3 << (6*2)) | (3 << (7*2))))
                    | (2 << (4*2))   // PA4 NSS  → AF
                    | (2 << (5*2))   // PA5 SCK  → AF
                    | (2 << (6*2))   // PA6 MISO → AF
                    | (2 << (7*2));  // PA7 MOSI → AF

    // Pull-up on NSS (keeps line high when master not connected)
    // PUPDR: 00=no pull, 01=pull-up, 10=pull-down
    GPIOA->PUPDR = (GPIOA->PUPDR & ~(3 << (4*2))) | (1 << (4*2));

    // High speed on MISO and SCK (reduces edge ringing at high frequencies)
    // OSPEEDR: 00=low, 01=medium, 10=high, 11=very high
    GPIOA->OSPEEDR |= (3 << (6*2))   // MISO very high speed
                    | (3 << (5*2));  // SCK very high speed

    // Set alternate function numbers (AFRL for pins 0-7)
    GPIOA->AFR[0] = (GPIOA->AFR[0]
                    & ~((0xF << (4*4)) | (0xF << (5*4))
                       | (0xF << (6*4)) | (0xF << (7*4))))
                    | (SPI_AF_NUM << (4*4))    // NSS
                    | (SPI_AF_NUM << (5*4))    // SCK
                    | (SPI_AF_NUM << (6*4))    // MISO
                    | (SPI_AF_NUM << (7*4));   // MOSI

    // Configure IRQ pin (PB2) for push mode - output, initially low (inactive)
    // Active high: low = no data, high = data ready
    GPIOB->MODER = (GPIOB->MODER & ~(3 << (2*2))) | (1 << (2*2));  // Output mode
    GPIOB->BSRR = IRQ_GPIO_PIN << 16;  // Reset = low (inactive)
}

/**
 * @brief Configure SPI1 peripheral registers
 */
static void spi_peripheral_init(void) {
    // Enable SPI1 clock (SPI1 is on APB2, not APB1!)
    RCC->APBENR2 |= RCC_APBENR2_SPI1EN;

    // Disable SPI during configuration
    SPI1->CR1 &= ~SPI_CR1_SPE;

    // ── CR1 Configuration ──
    // All zeros = slave mode, CPOL=0, CPHA=0 (SPI Mode 0), MSB first, hardware NSS
    SPI1->CR1 = 0;
    // CPHA=0: data sampled on first (leading) clock edge
    // CPOL=0: clock idles low
    // MSTR=0: slave mode
    // SSM=0: hardware NSS management
    // LSBFIRST=0: MSB transmitted first

    // ── CR2 Configuration ──
    // This is the critical register for STM32G0 SPI operation
    SPI1->CR2 = (7 << SPI_CR2_DS_Pos)    // DS=0b0111 → 8-bit data frames
              | SPI_CR2_FRXTH             // CRITICAL: FIFO RX threshold = 1 byte (not 2!)
              | SPI_CR2_RXNEIE            // RXNE interrupt enabled (for command byte)
              | SPI_CR2_TXDMAEN;          // TX DMA enabled from start
    // Note: RXDMAEN is deliberately OFF - will be enabled after RXNE ISR reads command

    // Enable SPI peripheral
    SPI1->CR1 |= SPI_CR1_SPE;
}

/**
 * @brief Configure DMAMUX to route SPI requests to DMA channels
 */
static void dmamux_init(void) {
    // CRITICAL: DMAMUX channels are 0-indexed, DMA channels are 1-indexed!
    // DMA1_Channel1 → DMAMUX1_Channel0
    // DMA1_Channel2 → DMAMUX1_Channel1

    // Route SPI1_RX to DMA1 Channel 1 (DMAMUX Channel 0)
    DMAMUX1_Channel0->CCR = DMAREQ_SPI_RX;  // 16

    // Route SPI1_TX to DMA1 Channel 2 (DMAMUX Channel 1)
    DMAMUX1_Channel1->CCR = DMAREQ_SPI_TX;  // 17
}

/**
 * @brief Configure EXTI for NSS rising edge detection
 */
static void exti_nss_init(void) {
    // Route PA4 to EXTI line 4
    // EXTICR[1] handles lines 4-7
    // Each line gets 8 bits: 0x00=PortA, 0x01=PortB, 0x02=PortC, etc.
    EXTI->EXTICR[1] = (EXTI->EXTICR[1] & ~(0xFF << 0)) | (NSS_EXTI_PORT << 0);

    // Enable rising edge detection (NSS deassert = transaction end)
    EXTI->RTSR1 |= (1 << NSS_EXTI_LINE);

    // Unmask interrupt for line 4
    EXTI->IMR1 |= (1 << NSS_EXTI_LINE);

    // Enable EXTI4_15 interrupt in NVIC (covers lines 4-15)
    NVIC_SetPriority(EXTI4_15_IRQn, 2);  // Lower priority than SPI/DMA
    NVIC_EnableIRQ(EXTI4_15_IRQn);
}

// ============================================================================
// DMA HELPER FUNCTIONS
// ============================================================================

/**
 * @brief Start RX DMA: SPI1_DR → buf (peripheral → memory)
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
    DMA1_Channel1->CPAR  = (uint32_t)&SPI1->DR;   // Source: SPI data register
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
 * @brief Start TX DMA: buf → SPI1_DR (memory → peripheral)
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
    DMA1_Channel2->CPAR  = (uint32_t)&SPI1->DR;   // Destination: SPI data register
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
    while (SPI1->SR & SPI_SR_FRLVL) {
        // 8-bit read to pop exactly one byte
        dummy = *(volatile uint8_t *)&SPI1->DR;
    }

    // Read SR to clear OVR flag
    dummy = SPI1->SR;
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
    SPI1->CR1 &= ~SPI_CR1_SPE;  // Disable SPI (flushes FIFOs on re-enable)
    SPI1->CR2 &= ~(SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN | SPI_CR2_RXNEIE);

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
    SPI1->CR2 = (7 << SPI_CR2_DS_Pos)    // 8-bit data frames
              | SPI_CR2_FRXTH            // 1-byte FIFO threshold
              | SPI_CR2_RXNEIE           // RXNE interrupt for command byte
              | SPI_CR2_TXDMAEN;         // TX DMA active from start
    // Note: RXDMAEN deliberately OFF - enabled after RXNE ISR

    // ── Step 7: Enable SPI ──
    SPI1->CR1 |= SPI_CR1_SPE;
}

/**
 * @brief Arm SPI slave for push mode transaction
 *
 * Similar to spi_slave_arm() but configured for push mode:
 * - TX DMA loaded with [TYPE:1][PAYLOAD:N]
 * - RX DMA armed to capture any incoming master data
 * - Both TX and RX DMA enabled from the start
 * - No RXNEIE (we're not waiting for a command byte)
 */
static void spi_slave_arm_push(void) {
    // ── Step 1: Stop everything ──
    SPI1->CR1 &= ~SPI_CR1_SPE;
    SPI1->CR2 &= ~(SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN | SPI_CR2_RXNEIE);

    // Disable both DMA channels
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
    DMA1_Channel2->CCR &= ~DMA_CCR_EN;

    // Clear all DMA flags
    DMA1->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CGIF2;

    // Clear SPI error flags
    spi_clear_errors();

    // ── Step 2: Clear RX buffer ──
    memset(ctx.rx_buf, 0x00, sizeof(ctx.rx_buf));

    // ── Step 3: Start TX DMA with the prepared push data ──
    // tx_buf was already loaded by spi_slave_prepare_push()
    spi_tx_dma_start(ctx.tx_buf, ctx.tx_dma_length);

    // ── Step 4: Start RX DMA to capture full transaction ──
    // RX captures everything master sends (could be dummy bytes or radio TX)
    spi_rx_dma_start(ctx.rx_buf, MAX_TRANSACTION_SIZE);

    // ── Step 5: Configure SPI CR2 for push mode ──
    // Both TX and RX DMA enabled from the start
    // No RXNEIE - we're not waiting for a command byte
    SPI1->CR2 = (7 << SPI_CR2_DS_Pos)    // 8-bit data frames
              | SPI_CR2_FRXTH            // 1-byte FIFO threshold
              | SPI_CR2_TXDMAEN          // TX DMA enabled
              | SPI_CR2_RXDMAEN;         // RX DMA enabled

    // ── Step 6: Enable SPI ──
    SPI1->CR1 |= SPI_CR1_SPE;

    // ── Step 7: Update state ──
    ctx.state = SPI_STATE_HAVE_DATA;
    ctx.payload_processed = false;
}

/**
 * @brief Prepare TX buffer for push and arm DMAs
 *
 * Loads the TX buffer with [TYPE:1][PAYLOAD:N] and configures
 * DMAs for the push transaction.
 *
 * @param push_type PUSH_TYPE_RADIO or PUSH_TYPE_GPS
 */
static void spi_slave_prepare_push(uint8_t push_type) {
    // Zero the TX buffer first
    memset(ctx.tx_buf, 0x00, sizeof(ctx.tx_buf));

    ctx.pending_push_type = push_type;

    switch (push_type) {
    case PUSH_TYPE_RADIO: {
        // Load type byte
        ctx.tx_buf[0] = PUSH_TYPE_RADIO;

        // Load payload from queue (peek, don't dequeue yet)
        // Use tail pointer for FIFO behavior (push oldest message first)
        uint8_t *msg_ptr;
        if (radio_message_queue_tail_pointer(ctx.radio_queue, &msg_ptr)) {
            memcpy(&ctx.tx_buf[PUSH_TYPE_BYTES], msg_ptr, PUSH_RADIO_PAYLOAD);
        }

        // Set TX length: type byte + radio payload
        ctx.tx_dma_length = PUSH_RADIO_TOTAL;  // 1 + 256 = 257
        break;
    }

    case PUSH_TYPE_GPS: {
        // Load type byte
        ctx.tx_buf[0] = PUSH_TYPE_GPS;

        // Load raw NMEA sentence from GPS queue (peek, don't dequeue yet)
        uint8_t *gps_ptr;
        if (gps_sample_queue_tail_pointer(ctx.gps_queue, &gps_ptr)) {
            memcpy(&ctx.tx_buf[PUSH_TYPE_BYTES], gps_ptr, PUSH_GPS_PAYLOAD);
        }

        // Set TX length: type byte + NMEA payload
        ctx.tx_dma_length = PUSH_GPS_TOTAL;  // 1 + 87 = 88
        break;
    }

    default:
        // Unknown push type - don't arm
        return;
    }

    // Arm DMAs for push transaction
    spi_slave_arm_push();

    // Assert IRQ to notify master
    spi_slave_assert_irq();
}

// ============================================================================
// INTERRUPT HANDLERS
// ============================================================================

/**
 * @brief SPI1 RXNE Interrupt Handler
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
void spi_slave_spi1_irq_handler(void) {
    // Verify this is an RXNE interrupt
    if ((SPI1->SR & SPI_SR_RXNE) && (SPI1->CR2 & SPI_CR2_RXNEIE)) {

        // ── Read command byte ──
        // CRITICAL: 8-bit read to pop exactly one byte from RX FIFO
        // This also clears the RXNE flag
        uint8_t cmd = *(volatile uint8_t *)&SPI1->DR;
        ctx.current_cmd = cmd;
        ctx.rx_buf[0] = cmd;

        // ── Disable RXNE interrupt ──
        // We got the command. Don't want RXNE firing for every subsequent byte.
        SPI1->CR2 &= ~SPI_CR2_RXNEIE;

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
        SPI1->CR2 |= SPI_CR2_RXDMAEN;

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
#ifdef DEBUG
                /* Log SPI radio TX to debug console */
                debug_uart_log_spi_radio_tx(&ctx.rx_buf[CMD_OVERHEAD], 256);
#endif
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
 * @brief NSS EXTI Handler (line 4, PA4)
 *
 * Fires when NSS rising edge detected.
 * This is the universal "transaction complete" signal.
 *
 * Handles:
 * - Pull mode: command-based transactions (SPI_STATE_ACTIVE)
 * - Push mode: IRQ-initiated transactions (SPI_STATE_HAVE_DATA)
 * - Simultaneous TX/RX in push mode (checks RX pattern for master TX)
 */
void spi_slave_nss_exti_handler(void) {

    // Check if this is NSS rising edge (line 4)
    if (EXTI->RPR1 & (1 << NSS_EXTI_LINE)) {
        // Clear pending flag (write 1 to clear)
        EXTI->RPR1 = (1 << NSS_EXTI_LINE);

        // ── Handle based on state ──
        switch (ctx.state) {

        case SPI_STATE_IDLE:
            // NSS rose before we even got a command byte.
            // Glitch or aborted transaction. Just re-arm.
            break;

        case SPI_STATE_HAVE_DATA:
            // ══════════════════════════════════════════════════════════════
            // PUSH MODE TRANSACTION COMPLETE
            // ══════════════════════════════════════════════════════════════
            // We asserted IRQ, master responded, transaction complete.

            // Deassert IRQ first
            spi_slave_deassert_irq();

            // Check how many bytes we actually transmitted
            {
                uint16_t tx_remaining = DMA1_Channel2->CNDTR;
                uint16_t tx_sent = ctx.tx_dma_length - tx_remaining;

                // If we sent at least the type byte + some payload, consider it success
                if (tx_sent >= PUSH_TYPE_BYTES) {
                    // Pop the message from queue since it was transmitted
                    if (ctx.pending_push_type == PUSH_TYPE_RADIO && ctx.radio_queue) {
                        radio_message_queue_pop(ctx.radio_queue);
                    } else if (ctx.pending_push_type == PUSH_TYPE_GPS && ctx.gps_queue) {
                        gps_sample_queue_pop(ctx.gps_queue);
                    }

                    ctx.push_transactions++;
                }

                // Check if master sent data while we were pushing (simultaneous TX/RX)
                uint16_t rx_remaining = DMA1_Channel1->CNDTR;
                uint16_t rx_received = MAX_TRANSACTION_SIZE - rx_remaining;

                // If we received a substantial amount and it looks like real data...
                if (rx_received >= PUSH_RADIO_PAYLOAD && spi_slave_rx_has_valid_data()) {
                    // Master sent a radio TX message simultaneously!
                    // Enqueue it to the radio queue
                    if (ctx.radio_queue) {
                        radio_message_enqueue(PUSH_RADIO_PAYLOAD, ctx.rx_buf, ctx.radio_queue);
                        ctx.master_tx_received++;
                    }
                }
            }

            ctx.pending_push_type = 0;
            ctx.transactions_completed++;
            break;

        case SPI_STATE_ACTIVE:
            // ══════════════════════════════════════════════════════════════
            // PULL MODE TRANSACTION COMPLETE
            // ══════════════════════════════════════════════════════════════
            // Master initiated with command byte, DMA handled the rest.

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
#ifdef DEBUG
                        /* Log SPI radio TX to debug console */
                        debug_uart_log_spi_radio_tx(&ctx.rx_buf[CMD_OVERHEAD], 256);
#endif
                        ctx.payload_processed = true;
                    }
                } else if (payload_bytes > 0) {
                    // Partial payload - master ended transaction early
                    // Could enqueue partial message or discard as error
                    // For now, discard
                }
            }

            // Check for overrun errors
            if (SPI1->SR & SPI_SR_OVR) {
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
    RCC->APBENR2 |= RCC_APBENR2_SPI1EN;    // SPI1 (on APB2, not APB1!)

    // ── Initialize peripherals ──
    spi_gpio_init();
    dmamux_init();
    spi_peripheral_init();
    exti_nss_init();

    // ── Enable interrupts in NVIC ──
    // SPI1 RXNE interrupt (highest priority - must respond fast)
    NVIC_SetPriority(SPI1_IRQn, 0);
    NVIC_EnableIRQ(SPI1_IRQn);

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
    ctx.master_tx_received = 0;
}

// ============================================================================
// PUSH MODE IMPLEMENTATION
// ============================================================================

/**
 * @brief Assert IRQ line to master (active high)
 *
 * Sets PB2 high to signal the master that data is ready.
 * The IRQ pin is active-high for rising edge detection on master.
 */
void spi_slave_assert_irq(void) {
    // BSRR lower 16 bits set the pin, upper 16 bits reset (clear) the pin
    // To drive high (active), we write to the set bits (lower half)
    GPIOB->BSRR = IRQ_GPIO_PIN;  // Set = high = active
    ctx.irq_asserted = true;
}

/**
 * @brief Deassert IRQ line to master (inactive low)
 *
 * Sets PB2 low to release the IRQ line.
 */
void spi_slave_deassert_irq(void) {
    // Write to reset bits (upper half) to drive low
    GPIOB->BSRR = IRQ_GPIO_PIN << 16;  // Reset = low = inactive
    ctx.irq_asserted = false;
}

/**
 * @brief Check if RX buffer contains valid data (not dummy bytes)
 *
 * Samples the first RX_PATTERN_SAMPLE_SIZE bytes and checks if they
 * appear to be real data vs dummy bytes (all 0x00 or 0xFF).
 *
 * @return true if RX buffer appears to contain real radio message data
 */
bool spi_slave_rx_has_valid_data(void) {
    uint8_t zeros = 0;
    uint8_t ones = 0;

    // Sample the first few bytes of the RX buffer
    for (int i = 0; i < RX_PATTERN_SAMPLE_SIZE; i++) {
        if (ctx.rx_buf[i] == 0x00) zeros++;
        if (ctx.rx_buf[i] == 0xFF) ones++;
    }

    // If most bytes are 0x00 or 0xFF, it's probably dummy data
    // Real radio messages typically have varied content
    return (zeros < RX_PATTERN_THRESHOLD && ones < RX_PATTERN_THRESHOLD);
}

/**
 * @brief Periodic tick function for push mode
 *
 * Call this from the main loop. When data is available and the slave
 * is idle, this prepares the TX buffer, arms DMAs, and asserts IRQ.
 *
 * Priority: GPS (smaller, more time-critical) > Radio (larger)
 */
void spi_slave_tick(void) {
    // Only process if we're idle (not mid-transaction)
    if (ctx.state != SPI_STATE_IDLE) {
        return;
    }

    // Already have IRQ asserted and waiting for master
    if (ctx.irq_asserted) {
        return;
    }

    // Check for pending GPS NMEA sentence to push (higher priority - smaller, time-sensitive)
    if (ctx.gps_queue && !gps_sample_queue_empty(ctx.gps_queue)) {
        spi_slave_prepare_push(PUSH_TYPE_GPS);
        return;
    }

    // Check for pending radio message to push
    if (ctx.radio_queue && !radio_message_queue_empty(ctx.radio_queue)) {
        spi_slave_prepare_push(PUSH_TYPE_RADIO);
        return;
    }
}
