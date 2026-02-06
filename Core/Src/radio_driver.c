/**
 * @file radio_driver.c
 * @brief Radio transceiver driver using DMA + IDLE line detection
 *
 * Receives null-terminated messages from radio via UART5 using DMA in circular mode.
 * Interrupts only on:
 *   - IDLE line detection (end of transmission burst)
 *   - DMA Half-Transfer (buffer 50% full)
 *   - DMA Transfer-Complete (buffer wrap-around)
 *
 * TX: Messages are sent followed by null terminator (0x00).
 * RX: Bytes accumulate until 0x00 delimiter, then message is enqueued.
 */

#include "radio_driver.h"
#include "stm32g0xx_hal.h"
#include <string.h>

/* ============================================================================
 * Configuration
 * ============================================================================ */

#define RADIO_DMA_BUF_SIZE    512u   /**< DMA circular buffer size (power of 2) */

#if (RADIO_DMA_BUF_SIZE & (RADIO_DMA_BUF_SIZE - 1u))
#error "RADIO_DMA_BUF_SIZE must be power of two"
#endif

/* ============================================================================
 * External References
 * ============================================================================ */

extern UART_HandleTypeDef huart5;  /* Radio UART (defined in main.c) */

/* ============================================================================
 * Private State
 * ============================================================================ */

static radio_message_queue_t *rx_queue = NULL;

/** DMA circular receive buffer */
static uint8_t s_dma_buf[RADIO_DMA_BUF_SIZE];

/** Last processed position in DMA buffer */
static volatile uint16_t s_last_pos = 0;

/** Message accumulation buffer */
static uint8_t s_msg_buf[RADIO_MAX_MESSAGE_LEN];
static uint16_t s_msg_len = 0;

/** Driver initialized flag */
static bool s_initialized = false;

/* ============================================================================
 * Forward Declarations
 * ============================================================================ */

static void process_dma_data(uint16_t new_pos);
static void feed_byte(uint8_t b);

/* ============================================================================
 * Public API
 * ============================================================================ */

void radio_init(radio_message_queue_t *queue)
{
    rx_queue = queue;
    radio_message_queue_init(rx_queue);

    /* Reset state */
    s_last_pos = 0;
    s_msg_len = 0;
    memset(s_dma_buf, 0, sizeof(s_dma_buf));
    memset(s_msg_buf, 0, sizeof(s_msg_buf));

    /* Start DMA receive with IDLE line detection */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, s_dma_buf, RADIO_DMA_BUF_SIZE);

    /* Enable DMA Half-Transfer interrupt for periodic processing */
    __HAL_DMA_ENABLE_IT(huart5.hdmarx, DMA_IT_HT);

    s_initialized = true;
}

bool radio_send(const uint8_t *data, uint8_t len)
{
    if (data == NULL || len == 0 || len > 254) {
        return false;
    }

    /* Send data bytes */
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart5, (uint8_t *)data, len, 1000);
    if (status != HAL_OK) {
        return false;
    }

    /* Send null terminator */
    uint8_t null_byte = 0x00;
    status = HAL_UART_Transmit(&huart5, &null_byte, 1, 100);

    return (status == HAL_OK);
}

bool radio_available(void)
{
    if (rx_queue == NULL) {
        return false;
    }
    return !radio_message_queue_empty(rx_queue);
}

uint8_t radio_read(uint8_t *buffer)
{
    if (rx_queue == NULL || buffer == NULL) {
        return 0;
    }

    if (radio_message_queue_empty(rx_queue)) {
        return 0;
    }

    /* Dequeue the oldest message */
    radio_message_dequeue(rx_queue, buffer);

    /* Find the actual length (messages are null-padded) */
    uint8_t len = 0;
    while (len < RADIO_MAX_MESSAGE_LEN && buffer[len] != 0) {
        len++;
    }

    return len;
}

uint8_t radio_rx_count(void)
{
    if (rx_queue == NULL) {
        return 0;
    }

    if (radio_message_queue_empty(rx_queue)) {
        return 0;
    }

    return (rx_queue->head - rx_queue->tail + RADIO_MESSAGE_QUEUE_LEN) % RADIO_MESSAGE_QUEUE_LEN;
}

radio_message_queue_t *radio_get_rx_queue(void)
{
    return rx_queue;
}

/* ============================================================================
 * UART Callbacks (called from main.c)
 * ============================================================================ */

/**
 * @brief Called when IDLE line detected OR DMA transfer complete
 *
 * This is the main receive callback for DMA + IDLE mode.
 *
 * @param huart UART handle
 * @param Size Current position in DMA buffer
 */
void radio_rx_event_callback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance != USART5 || !s_initialized) {
        return;
    }

    process_dma_data(Size);
}

/**
 * @brief Called on UART error - restarts DMA
 */
void radio_uart_error_callback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART5 || !s_initialized) {
        return;
    }

    /* Reset and restart DMA */
    s_last_pos = 0;
    s_msg_len = 0;
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, s_dma_buf, RADIO_DMA_BUF_SIZE);
    __HAL_DMA_ENABLE_IT(huart5.hdmarx, DMA_IT_HT);
}

/* Legacy per-byte handler - no longer used but kept for compatibility */
void radio_rx_byte_handler(uint8_t byte)
{
    (void)byte;
    /* Not used in DMA mode */
}

/* ============================================================================
 * Private Functions
 * ============================================================================ */

/**
 * @brief Process new data in DMA buffer
 *
 * Handles circular buffer wrap-around and feeds bytes to message parser.
 *
 * @param new_pos Current DMA buffer position
 */
static void process_dma_data(uint16_t new_pos)
{
    uint16_t last = s_last_pos;
    uint16_t current = new_pos;

    if (current == last) {
        /* No new data */
        return;
    }

    if (current > last) {
        /* Simple case: no wrap-around */
        for (uint16_t i = last; i < current; i++) {
            feed_byte(s_dma_buf[i]);
        }
    } else {
        /* Wrap-around: process end of buffer, then start */
        for (uint16_t i = last; i < RADIO_DMA_BUF_SIZE; i++) {
            feed_byte(s_dma_buf[i]);
        }
        for (uint16_t i = 0; i < current; i++) {
            feed_byte(s_dma_buf[i]);
        }
    }

    s_last_pos = current;
}

/**
 * @brief Feed one byte to message accumulator
 *
 * Accumulates bytes until null terminator, then enqueues complete message.
 *
 * @param b Received byte
 */
static void feed_byte(uint8_t b)
{
    if (rx_queue == NULL) {
        return;
    }

    if (b == 0x00) {
        /* Null terminator - end of message */
        if (s_msg_len > 0) {
            /* Enqueue the accumulated message */
            radio_message_enqueue(s_msg_len, s_msg_buf, rx_queue);

            /* Reset for next message */
            s_msg_len = 0;
            memset(s_msg_buf, 0, sizeof(s_msg_buf));
        }
    } else {
        /* Accumulate byte */
        if (s_msg_len < RADIO_MAX_MESSAGE_LEN) {
            s_msg_buf[s_msg_len] = b;
            s_msg_len++;
        } else {
            /* Buffer overflow - discard and start over */
            s_msg_len = 0;
            memset(s_msg_buf, 0, sizeof(s_msg_buf));
        }
    }
}
