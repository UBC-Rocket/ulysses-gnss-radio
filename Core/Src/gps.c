/**
 * @file gps.c
 * @brief GPS Driver using DMA + IDLE line detection
 *
 * Receives NMEA sentences from GPS module via UART6 using DMA in circular mode.
 * Interrupts only on:
 *   - IDLE line detection (end of transmission burst)
 *   - DMA Half-Transfer (buffer 50% full)
 *   - DMA Transfer-Complete (buffer wrap-around)
 *
 * This is ~80x more efficient than per-byte interrupts for typical NMEA traffic.
 *
 * Author: Ernie Han (original), refactored for DMA by UBC Rocket
 */

#include "gps.h"
#include "gps_nema_queue.h"
#include <stdbool.h>
#include <string.h>

/* ============================================================================
 * Configuration
 * ============================================================================ */

#define GPS_DMA_BUF_SIZE    256u   /**< DMA circular buffer size (power of 2) */
#define GPS_NMEA_MAX        87u    /**< Max NMEA sentence length (82 chars + safety margin) */

#if (GPS_DMA_BUF_SIZE & (GPS_DMA_BUF_SIZE - 1u))
#error "GPS_DMA_BUF_SIZE must be power of two"
#endif

/* ============================================================================
 * Private State
 * ============================================================================ */

static UART_HandleTypeDef *s_gps = NULL;
static UART_HandleTypeDef *s_out = NULL;

/** DMA circular receive buffer */
static uint8_t s_dma_buf[GPS_DMA_BUF_SIZE];

/** Last processed position in DMA buffer */
static volatile uint16_t s_last_pos = 0;

/** NMEA sentence builder */
static uint8_t s_line[GPS_NMEA_MAX];
static uint16_t s_line_len = 0;
static bool s_in_sentence = false;

/** Shared GPS queue for SPI push mode */
static gps_sample_queue_t *s_gps_queue = NULL;

/** Debug output state */
static volatile bool s_out_tx_busy = false;
static uint8_t s_out_tx_buf[GPS_NMEA_MAX];
static uint16_t s_out_tx_len = 0;

/* ============================================================================
 * Forward Declarations
 * ============================================================================ */

static void process_dma_data(uint16_t new_pos);
static void feed_byte(uint8_t b);
static void nmea_reset(void);
static void out_try_start_tx(void);

/* ============================================================================
 * Public API
 * ============================================================================ */

void gps_set_queue(gps_sample_queue_t *queue)
{
    s_gps_queue = queue;
}

void gps_init(UART_HandleTypeDef *gps_uart, UART_HandleTypeDef *out_uart)
{
    s_gps = gps_uart;
    s_out = out_uart;

    /* Reset state */
    s_last_pos = 0;
    s_line_len = 0;
    s_in_sentence = false;
    s_out_tx_busy = false;
    s_out_tx_len = 0;

    memset(s_dma_buf, 0, sizeof(s_dma_buf));
    memset(s_line, 0, sizeof(s_line));

    /* Start DMA receive with IDLE line detection */
    if (s_gps) {
        /* Use ReceiveToIdle_DMA for automatic IDLE detection */
        HAL_UARTEx_ReceiveToIdle_DMA(s_gps, s_dma_buf, GPS_DMA_BUF_SIZE);

        /* Enable DMA Half-Transfer interrupt for periodic processing */
        /* (ReceiveToIdle_DMA enables IDLE and TC by default) */
        __HAL_DMA_ENABLE_IT(s_gps->hdmarx, DMA_IT_HT);
    }
}

void gps_process(void)
{
    /* 
     * In DMA mode, most processing happens in callbacks.
     * This function can be used for any deferred/non-ISR processing if needed.
     * Currently empty since callbacks handle everything.
     */
}

/* ============================================================================
 * UART Callbacks (called from main.c or stm32g0xx_it.c)
 * ============================================================================ */

/**
 * @brief Called when IDLE line detected OR DMA transfer complete
 *
 * This is the main receive callback for DMA + IDLE mode.
 * HAL_UARTEx_RxEventCallback fires on:
 *   - IDLE line detection (most common - end of GPS burst)
 *   - DMA Half-Transfer (buffer 50% full)
 *   - DMA Transfer-Complete (buffer 100% full / wrap)
 *
 * @param huart UART handle
 * @param Size Number of bytes received since last event
 */
void gps_rx_event_callback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (!s_gps || huart != s_gps) {
        return;
    }

    /* Size is the current position in the DMA buffer */
    process_dma_data(Size);
}

/**
 * @brief Called on UART error
 *
 * Restarts DMA reception on error.
 */
void gps_uart_error_callback(UART_HandleTypeDef *huart)
{
    if (!s_gps || huart != s_gps) {
        return;
    }

    /* Reset and restart DMA */
    s_last_pos = 0;
    HAL_UARTEx_ReceiveToIdle_DMA(s_gps, s_dma_buf, GPS_DMA_BUF_SIZE);
    __HAL_DMA_ENABLE_IT(s_gps->hdmarx, DMA_IT_HT);
}

/**
 * @brief Called when debug UART TX completes
 */
void gps_uart_tx_cplt_callback(UART_HandleTypeDef *huart)
{
    if (!s_out || huart != s_out) {
        return;
    }

    s_out_tx_busy = false;
    s_out_tx_len = 0;
}

/* Legacy callback - no longer used in DMA mode but kept for compatibility */
void gps_uart_rx_cplt_callback(UART_HandleTypeDef *huart)
{
    (void)huart;
    /* Not used in DMA mode */
}

/* ============================================================================
 * Private Functions
 * ============================================================================ */

/**
 * @brief Process new data in DMA buffer
 *
 * Handles circular buffer wrap-around and feeds bytes to NMEA parser.
 *
 * @param new_pos Current DMA buffer position (from callback)
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
        for (uint16_t i = last; i < GPS_DMA_BUF_SIZE; i++) {
            feed_byte(s_dma_buf[i]);
        }
        for (uint16_t i = 0; i < current; i++) {
            feed_byte(s_dma_buf[i]);
        }
    }

    s_last_pos = current;
}

/**
 * @brief Feed one byte to NMEA sentence builder
 *
 * Accumulates bytes into complete NMEA sentences and enqueues them.
 *
 * @param b Received byte
 */
static void feed_byte(uint8_t b)
{
    /* Wait for '$' to start a new sentence */
    if (!s_in_sentence) {
        if (b == '$') {
            s_in_sentence = true;
            s_line_len = 0;
            s_line[s_line_len++] = b;
        }
        return;
    }

    /* Check for buffer overflow */
    if (s_line_len >= (GPS_NMEA_MAX - 1u)) {
        nmea_reset();
        return;
    }

    /* Accumulate byte */
    s_line[s_line_len++] = b;

    /* Check for end of sentence */
    if (b == '\n') {
        /* Null-terminate for safety */
        if (s_line_len < GPS_NMEA_MAX) {
            s_line[s_line_len] = '\0';
        }

        /* Enqueue to SPI push queue */
        if (s_gps_queue != NULL) {
            gps_sample_enqueue(s_line, s_gps_queue);
        }

        /* Debug output to UART1 */
        if (s_out && !s_out_tx_busy) {
            memcpy(s_out_tx_buf, s_line, s_line_len);
            s_out_tx_len = s_line_len;
            out_try_start_tx();
        }

        nmea_reset();
    }
}

/**
 * @brief Reset NMEA sentence builder state
 */
static void nmea_reset(void)
{
    s_in_sentence = false;
    s_line_len = 0;
}

/**
 * @brief Start non-blocking debug output transmission
 */
static void out_try_start_tx(void)
{
    if (!s_out || s_out_tx_busy || s_out_tx_len == 0) {
        return;
    }

    s_out_tx_busy = true;

    if (HAL_UART_Transmit_IT(s_out, s_out_tx_buf, s_out_tx_len) != HAL_OK) {
        s_out_tx_busy = false;
    }
}
