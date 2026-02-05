#include "spi_slave.h"
#include "timing_utils.h"
#include "stm32g0xx_hal.h"
#include <string.h>

// ============================================================================
// EXTERNAL HANDLES & DEPENDENCIES
// ============================================================================

extern SPI_HandleTypeDef hspi1;  // Defined in main.c

// ============================================================================
// BUFFER ALLOCATIONS (DMA-safe, aligned)
// ============================================================================

// Ping-pong RX buffers for full-duplex operation
static uint8_t __attribute__((aligned(32))) rx_buffer_a[MAX_TRANSACTION_SIZE];
static uint8_t __attribute__((aligned(32))) rx_buffer_b[MAX_TRANSACTION_SIZE];

// TX buffer (pre-loaded with response data)
static uint8_t __attribute__((aligned(32))) tx_buffer[MAX_TRANSACTION_SIZE];

// Scratch buffer for TX when we don't care about TX content (RX-only transactions)
static uint8_t __attribute__((aligned(32))) tx_scratch[MAX_TRANSACTION_SIZE];

// ============================================================================
// STATIC HANDLER INSTANCE
// ============================================================================

static spi_handler_t handler;

// ============================================================================
// GPIO DEFINITIONS
// ============================================================================

// DATA_READY IRQ line (PB2, active low per spec)
#define IRQ_GPIO_PORT   GPIOB
#define IRQ_GPIO_PIN    GPIO_PIN_2

// CS line (PA4, for EXTI monitoring)
#define CS_GPIO_PORT    GPIOA
#define CS_GPIO_PIN     GPIO_PIN_4

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

/**
 * @brief Assert DATA_READY IRQ line (active low)
 */
void spi_slave_assert_irq(void) {
    handler.irq_assert_timestamp = get_time_us();
    HAL_GPIO_WritePin(IRQ_GPIO_PORT, IRQ_GPIO_PIN, GPIO_PIN_RESET);  // Active LOW
    handler.irq_asserted = true;
}

/**
 * @brief Deassert DATA_READY IRQ line
 */
void spi_slave_deassert_irq(void) {
    HAL_GPIO_WritePin(IRQ_GPIO_PORT, IRQ_GPIO_PIN, GPIO_PIN_SET);  // Inactive HIGH
    handler.irq_asserted = false;
}

/**
 * @brief Swap ping-pong RX buffers
 */
static void swap_rx_buffers(void) {
    uint8_t *temp = handler.rx_active_buffer;
    handler.rx_active_buffer = handler.rx_complete_buffer;
    handler.rx_complete_buffer = temp;
}

/**
 * @brief Check if there's pending data to send (push mode)
 *
 * @return true if GPS or radio data is queued
 */
static bool has_pending_data(void) {
    return !gps_sample_queue_empty(handler.gps_queue) ||
           !radio_message_queue_empty(handler.radio_queue);
}

/**
 * @brief Get count of buffered radio messages
 *
 * @return uint8_t Number of messages (0-255)
 */
static uint8_t get_radio_queue_count(void) {
    if (radio_message_queue_empty(handler.radio_queue)) {
        return 0;
    }

    // Calculate count from head and tail
    uint8_t head = handler.radio_queue->head;
    uint8_t tail = handler.radio_queue->tail;

    if (head >= tail) {
        return head - tail;
    } else {
        return (RADIO_MESSAGE_QUEUE_LEN - tail) + head;
    }
}

// ============================================================================
// STATE TRANSITION
// ============================================================================

/**
 * @brief Transition to new state with optional debug logging
 *
 * @param new_state State to transition to
 */
void spi_slave_transition(spi_handler_state_t new_state) {
    // TODO: Add optional debug GPIO toggle or UART logging here
    handler.state = new_state;
}

// ============================================================================
// DMA ARMING (CRITICAL FUNCTION)
// ============================================================================

/**
 * @brief Arm SPI DMA for next transaction
 *
 * REVISED APPROACH: Always arms 256-byte TransmitReceive transactions.
 * TX buffer is pre-loaded based on state:
 * - IDLE: 0xFF (ready to receive)
 * - HAVE_DATA: [TYPE | PAYLOAD] (ready to send)
 * - Other states: appropriate response data
 *
 * Collision detection happens by inspecting RX buffer after DMA complete.
 */
void spi_slave_arm_dma(void) {
    HAL_StatusTypeDef status;
    uint16_t size = MAX_TRANSACTION_SIZE;  // Always 256 bytes
    uint8_t *tx_buf = tx_scratch;  // Default to scratch (don't care)
    uint8_t *rx_buf = handler.rx_active_buffer;

    // Fill TX buffer based on state
    memset(tx_scratch, 0xFF, MAX_TRANSACTION_SIZE);

    switch (handler.state) {
        case SPI_STATE_WAIT_CONFIG:
            // Special case: config frame is only 8 bytes
            size = CONFIG_FRAME_SIZE;
            break;

        case SPI_STATE_IDLE:
            // Always ready to receive 256 bytes
            // TX = 0xFF (don't care)
            break;

        case SPI_STATE_HAVE_DATA:
            // Pre-load TX buffer with type + payload
            // TX buffer already prepared by prepare_push_tx_buffer()
            tx_buf = tx_buffer;
            break;

        case SPI_STATE_PULL_CMD:
            // Receiving dummy bytes (2 bytes) while preparing response
            size = PULL_DUMMY_BYTES;
            break;

        case SPI_STATE_PULL_RESPONSE:
            // Sending response data (already loaded in tx_buffer)
            tx_buf = tx_buffer;
            size = handler.transaction.transfer_size;
            break;

        default:
            // For other states, just arm with don't-care data
            break;
    }

    // Arm the DMA transfer
    status = HAL_SPI_TransmitReceive_DMA(&hspi1, tx_buf, rx_buf, size);

    if (status != HAL_OK) {
        // DMA error - set error state
        handler.board_state = BOARD_STATE_ERROR_DMA;
        handler.errors++;
    }
}

// ============================================================================
// PULL MODE COMMAND HANDLERS
// ============================================================================

/**
 * @brief Handle RADIO_RX_LIFO command (0x01)
 *
 * Loads TX buffer with newest radio message + status/dummy prefix
 */
static void handle_radio_rx_lifo(void) {
    // TX buffer layout: [STATUS/X] [DUMMY] [DATA 0-255]
    // For now, status bytes are 0xFF (don't care)
    memset(tx_buffer, 0xFF, PULL_DUMMY_BYTES);

    uint8_t *data_ptr = tx_buffer + PULL_DUMMY_BYTES;

    uint8_t *msg_ptr;
    if (radio_message_queue_head_pointer(handler.radio_queue, &msg_ptr)) {
        memcpy(data_ptr, msg_ptr, RADIO_MESSAGE_MAX_LEN);
        radio_message_queue_pop_lifo(handler.radio_queue);
    } else {
        // Queue empty, send zeros
        memset(data_ptr, 0, PULL_RADIO_PAYLOAD);
    }

    handler.transaction.transfer_size = PULL_DUMMY_BYTES + PULL_RADIO_PAYLOAD;
}

/**
 * @brief Handle RADIO_RX_FIFO command (0x02)
 *
 * Loads TX buffer with oldest radio message + status/dummy prefix
 */
static void handle_radio_rx_fifo(void) {
    memset(tx_buffer, 0xFF, PULL_DUMMY_BYTES);

    uint8_t *data_ptr = tx_buffer + PULL_DUMMY_BYTES;

    uint8_t *msg_ptr;
    if (radio_message_queue_tail_pointer(handler.radio_queue, &msg_ptr)) {
        memcpy(data_ptr, msg_ptr, RADIO_MESSAGE_MAX_LEN);
        radio_message_queue_pop(handler.radio_queue);
    } else {
        // Queue empty, send zeros
        memset(data_ptr, 0, PULL_RADIO_PAYLOAD);
    }

    handler.transaction.transfer_size = PULL_DUMMY_BYTES + PULL_RADIO_PAYLOAD;
}

/**
 * @brief Handle RADIO_RXBUF_LEN command (0x03)
 *
 * Returns count of buffered radio messages
 */
static void handle_radio_buf_len(void) {
    memset(tx_buffer, 0xFF, PULL_DUMMY_BYTES);

    uint8_t count = get_radio_queue_count();
    tx_buffer[PULL_DUMMY_BYTES] = count;

    handler.transaction.transfer_size = PULL_DUMMY_BYTES + PULL_BUFLEN_PAYLOAD;
}

/**
 * @brief Handle RADIO_TX command (0x04)
 *
 * Master is sending a radio message - we just receive 256 bytes
 * Response is handled after reception in DMA complete callback
 */
static void handle_radio_tx(void) {
    // We're receiving 256 bytes from master
    // TX buffer doesn't matter (send 0xFF)
    memset(tx_buffer, 0xFF, PULL_DUMMY_BYTES + PULL_RADIO_PAYLOAD);
    handler.transaction.transfer_size = PULL_DUMMY_BYTES + PULL_RADIO_PAYLOAD;
}

/**
 * @brief Handle GPS_RX command (0x05)
 *
 * Returns raw NMEA sentence (pull mode)
 */
static void handle_gps_rx(void) {
    memset(tx_buffer, 0xFF, PULL_DUMMY_BYTES);

    uint8_t *data_ptr = tx_buffer + PULL_DUMMY_BYTES;

    uint8_t *gps_ptr;
    if (gps_sample_queue_tail_pointer(handler.gps_queue, &gps_ptr)) {
        memcpy(data_ptr, gps_ptr, GPS_SAMPLE_SIZE);
        gps_sample_queue_pop(handler.gps_queue);
    } else {
        // Queue empty, send zeros
        memset(data_ptr, 0, PULL_GPS_PAYLOAD);
    }

    handler.transaction.transfer_size = PULL_DUMMY_BYTES + PULL_GPS_PAYLOAD;
}

/**
 * @brief Process received pull mode command and prepare response
 *
 * @param cmd Command byte received
 */
static void process_pull_command(uint8_t cmd) {
    handler.transaction.command = cmd;

    switch (cmd) {
        case CMD_RADIO_RX_LIFO:
            handle_radio_rx_lifo();
            break;

        case CMD_RADIO_RX_FIFO:
            handle_radio_rx_fifo();
            break;

        case CMD_RADIO_RXBUF_LEN:
            handle_radio_buf_len();
            break;

        case CMD_RADIO_TX:
            handle_radio_tx();
            break;

        case CMD_GPS_RX:
            handle_gps_rx();
            break;

        default:
            // Unknown command - send zeros
            memset(tx_buffer, 0, MAX_TRANSACTION_SIZE);
            handler.transaction.transfer_size = PULL_DUMMY_BYTES + PULL_RADIO_PAYLOAD;
            handler.errors++;
            break;
    }
}

// ============================================================================
// PUSH MODE HELPERS
// ============================================================================

/**
 * @brief Prepare push mode TX buffer with type + payload
 *
 * Loads tx_buffer with type byte followed by appropriate payload.
 * GPS has priority over radio.
 *
 * @return true if data was prepared, false if no data available
 */
static bool prepare_push_tx_buffer(void) {
    // Priority: GPS first, then radio

    if (!gps_sample_queue_empty(handler.gps_queue)) {
        // Send GPS data
        handler.transaction.push_type = PUSH_TYPE_GPS;

        uint8_t *gps_ptr;
        if (gps_sample_queue_tail_pointer(handler.gps_queue, &gps_ptr)) {
            // For push mode, we would send parsed GPS struct
            // For now, send raw NMEA (TODO: implement NMEA parser)
            memcpy(tx_buffer + PUSH_TYPE_BYTES, gps_ptr, GPS_SAMPLE_SIZE);
            handler.transaction.transfer_size = GPS_SAMPLE_SIZE;  // Payload size only
            return true;
        }
    }

    if (!radio_message_queue_empty(handler.radio_queue)) {
        // Send radio data (newest message, LIFO)
        handler.transaction.push_type = PUSH_TYPE_RADIO;

        uint8_t *radio_ptr;
        if (radio_message_queue_head_pointer(handler.radio_queue, &radio_ptr)) {
            memcpy(tx_buffer + PUSH_TYPE_BYTES, radio_ptr, RADIO_MESSAGE_MAX_LEN);
            handler.transaction.transfer_size = PUSH_RADIO_PAYLOAD;  // Payload size only
            return true;
        }
    }

    return false;  // No data available
}

// ============================================================================
// STARTUP CONFIGURATION
// ============================================================================

/**
 * @brief Process received configuration frame
 *
 * @param config_data Pointer to 8-byte config frame
 */
static void process_config_frame(const uint8_t *config_data) {
    const config_frame_t *cfg = (const config_frame_t *)config_data;

    // Store configuration
    handler.config = *cfg;

    // Set mode
    if (cfg->mode == SPI_MODE_PULL || cfg->mode == SPI_MODE_PUSH) {
        handler.mode = (spi_protocol_mode_t)cfg->mode;
    } else {
        // Invalid mode, default to Pull
        handler.mode = SPI_MODE_PULL;
        handler.errors++;
    }

    // Transition to IDLE, ready for normal operation
    spi_slave_transition(SPI_STATE_IDLE);
}

// ============================================================================
// MAIN DMA COMPLETE CALLBACK (STATE MACHINE DRIVER)
// ============================================================================

/**
 * @brief SPI TX/RX DMA complete callback
 *
 * Called from HAL_SPI_TxRxCpltCallback in stm32g0xx_it.c
 * This is the main state machine driver - runs in ISR context!
 */
void spi_slave_txrx_complete(void) {
    // Swap buffers immediately (complete buffer now has received data)
    swap_rx_buffers();

    // Process based on current state
    switch (handler.state) {
        case SPI_STATE_WAIT_CONFIG:
            // Received configuration frame
            process_config_frame(handler.rx_complete_buffer);
            // State already transitioned to IDLE in process_config_frame
            spi_slave_arm_dma();  // Arm for next transaction
            break;

        case SPI_STATE_IDLE:
            if (handler.mode == SPI_MODE_PULL) {
                // Received command byte
                uint8_t cmd = handler.rx_complete_buffer[0];
                process_pull_command(cmd);

                // Transition to receiving dummy bytes
                spi_slave_transition(SPI_STATE_PULL_CMD);
                spi_slave_arm_dma();
            } else {
                // Push mode: received unsolicited radio TX
                spi_slave_transition(SPI_STATE_RX_RADIO);

                // Forward to radio UART callback
                if (handler.radio_tx_callback != NULL) {
                    handler.radio_tx_callback(handler.rx_complete_buffer, PULL_RADIO_PAYLOAD);
                }

                spi_slave_transition(SPI_STATE_COMPLETE);
            }
            break;

        case SPI_STATE_PULL_CMD:
            // Dummy bytes received, now send response
            spi_slave_transition(SPI_STATE_PULL_RESPONSE);
            spi_slave_arm_dma();
            break;

        case SPI_STATE_PULL_RESPONSE:
            // Response sent, check if it was RADIO_TX command
            if (handler.transaction.command == CMD_RADIO_TX) {
                // Forward received data to radio UART
                if (handler.radio_tx_callback != NULL) {
                    handler.radio_tx_callback(
                        handler.rx_complete_buffer + PULL_DUMMY_BYTES,
                        PULL_RADIO_PAYLOAD
                    );
                }
            }

            spi_slave_transition(SPI_STATE_COMPLETE);
            break;

        case SPI_STATE_PUSH_TYPE:
            // Type byte sent, now send payload
            spi_slave_transition(SPI_STATE_PUSH_PAYLOAD);
            spi_slave_arm_dma();
            break;

        case SPI_STATE_PUSH_PAYLOAD:
            // Payload sent, pop the message from queue
            if (handler.transaction.push_type == PUSH_TYPE_GPS) {
                gps_sample_queue_pop(handler.gps_queue);
            } else if (handler.transaction.push_type == PUSH_TYPE_RADIO) {
                radio_message_queue_pop_lifo(handler.radio_queue);
            }

            spi_slave_transition(SPI_STATE_COMPLETE);
            break;

        case SPI_STATE_RX_RADIO:
            // Radio TX received from master, forward to UART
            if (handler.radio_tx_callback != NULL) {
                handler.radio_tx_callback(handler.rx_complete_buffer, PULL_RADIO_PAYLOAD);
            }

            spi_slave_transition(SPI_STATE_COMPLETE);
            break;

        case SPI_STATE_COLLISION:
            // Collision resolved, received radio TX
            if (handler.radio_tx_callback != NULL) {
                handler.radio_tx_callback(handler.rx_complete_buffer, PULL_RADIO_PAYLOAD);
            }

            spi_slave_transition(SPI_STATE_COMPLETE);
            break;

        default:
            // Should not get DMA complete in other states
            handler.errors++;
            break;
    }
}

// ============================================================================
// CS EXTI CALLBACKS (COLLISION DETECTION)
// ============================================================================

/**
 * @brief CS (NSS) falling edge callback
 *
 * Called when master asserts CS (starts transaction)
 * Implements collision detection for push mode
 */
void spi_slave_cs_falling_edge(void) {
    uint32_t now = get_time_us();

    // Check for collision in push mode
    if (handler.mode == SPI_MODE_PUSH && handler.irq_asserted) {
        uint32_t elapsed = now - handler.irq_assert_timestamp;

        if (elapsed < T_RACE_US) {
            // COLLISION DETECTED
            handler.collisions_detected++;

            // Deassert IRQ, back off
            spi_slave_deassert_irq();

            // Transition to collision state, receive 256 bytes
            spi_slave_transition(SPI_STATE_COLLISION);

            // Arm DMA to receive radio TX (256 bytes)
            HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(
                &hspi1,
                tx_scratch,
                handler.rx_active_buffer,
                PULL_RADIO_PAYLOAD
            );

            if (status != HAL_OK) {
                handler.board_state = BOARD_STATE_ERROR_DMA;
                handler.errors++;
            }
        } else {
            // Normal push mode read (master responding to our IRQ)
            spi_slave_transition(SPI_STATE_PUSH_TYPE);
            spi_slave_arm_dma();
        }
    } else if (handler.mode == SPI_MODE_PUSH && !handler.irq_asserted) {
        // Unsolicited CS in push mode = radio TX from master
        spi_slave_transition(SPI_STATE_RX_RADIO);

        // Arm DMA to receive 256 bytes
        HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(
            &hspi1,
            tx_scratch,
            handler.rx_active_buffer,
            PULL_RADIO_PAYLOAD
        );

        if (status != HAL_OK) {
            handler.board_state = BOARD_STATE_ERROR_DMA;
            handler.errors++;
        }
    }
    // Pull mode: DMA already armed in IDLE state, no action needed here
}

/**
 * @brief CS (NSS) rising edge callback
 *
 * Called when master deasserts CS (ends transaction)
 */
void spi_slave_cs_rising_edge(void) {
    // Transaction complete
    if (handler.state == SPI_STATE_COMPLETE) {
        // Cleanup and return to IDLE

        // Deassert IRQ if it was asserted
        if (handler.irq_asserted) {
            spi_slave_deassert_irq();
        }

        // Clear transaction context
        memset(&handler.transaction, 0, sizeof(handler.transaction));

        // Increment transaction counter
        handler.transactions_completed++;

        // Return to IDLE
        spi_slave_transition(SPI_STATE_IDLE);
        spi_slave_arm_dma();
    }
}

// ============================================================================
// PUBLIC API
// ============================================================================

/**
 * @brief Initialize SPI slave protocol handler
 */
void spi_slave_init(radio_message_queue_t *radio_queue, gps_sample_queue_t *gps_queue) {
    // Clear handler structure
    memset(&handler, 0, sizeof(handler));

    // Store queue pointers
    handler.radio_queue = radio_queue;
    handler.gps_queue = gps_queue;

    // Initialize buffers
    handler.rx_active_buffer = rx_buffer_a;
    handler.rx_complete_buffer = rx_buffer_b;

    // Initialize timing subsystem
    timing_init();

    // Initialize state
    handler.state = SPI_STATE_WAIT_CONFIG;
    handler.mode = SPI_MODE_PULL;  // Default, overridden by config frame
    handler.board_state = BOARD_STATE_OK;

    // Initialize IRQ line (deasserted = HIGH)
    HAL_GPIO_WritePin(IRQ_GPIO_PORT, IRQ_GPIO_PIN, GPIO_PIN_SET);
    handler.irq_asserted = false;

    // Arm DMA for configuration frame
    spi_slave_arm_dma();
}

/**
 * @brief Tick function - called from main loop
 *
 * Checks for pending data in push mode and asserts IRQ
 */
void spi_slave_tick(void) {
    // Only relevant in push mode
    if (handler.mode != SPI_MODE_PUSH) {
        return;
    }

    // Only transition if in IDLE state
    if (handler.state != SPI_STATE_IDLE) {
        return;
    }

    // Check if we have data to send
    if (has_pending_data() && !handler.irq_asserted) {
        // Prepare TX buffer
        if (prepare_push_tx_buffer()) {
            // Transition to HAVE_DATA and assert IRQ
            spi_slave_transition(SPI_STATE_HAVE_DATA);
            spi_slave_assert_irq();
        }
    }
}

/**
 * @brief Set radio TX callback
 */
void spi_slave_set_radio_callback(void (*callback)(uint8_t *data, uint16_t len)) {
    handler.radio_tx_callback = callback;
}

/**
 * @brief Set board error state
 */
void spi_slave_set_board_state(board_state_t state) {
    handler.board_state = state;
}

/**
 * @brief Get current state (for debugging)
 */
spi_handler_state_t spi_slave_get_state(void) {
    return handler.state;
}

/**
 * @brief Get current mode (for debugging)
 */
spi_protocol_mode_t spi_slave_get_mode(void) {
    return handler.mode;
}
