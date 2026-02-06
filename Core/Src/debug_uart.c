/**
 * @file debug_uart.c
 * @brief Debug UART implementation - bidirectional injection and logging
 *
 * Provides message injection (RX) and system logging (TX) via UART1.
 * Only compiled when DEBUG macro is defined.
 */

#ifdef DEBUG

#include "debug_uart.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <ctype.h>

/* ============================================================================
 * External UART Handle
 * ============================================================================ */

extern UART_HandleTypeDef huart1;

/* ============================================================================
 * Module Variables
 * ============================================================================ */

/** DMA circular buffer for RX injection */
uint8_t debug_uart_rx_buffer[DEBUG_UART_RX_BUFFER_SIZE];

/** Log message queue */
static debug_log_queue_t log_queue;

/** Radio message queue pointer (for injection) */
static radio_message_queue_t *radio_queue = NULL;

/** GPS NMEA queue pointer (for injection) */
static gps_sample_queue_t *gps_queue = NULL;

/** Last processed RX buffer position (for DMA circular buffer tracking) */
static uint16_t last_rx_pos = 0;

/* ============================================================================
 * Internal Function Prototypes
 * ============================================================================ */

static void process_radio_injection(const uint8_t *payload, uint16_t len);
static void process_gps_injection(const uint8_t *payload, uint16_t len);
static void log_enqueue(const char *format, ...);
static bool log_queue_full(void);
static bool log_queue_empty(void);
static void format_hex_ascii(char *out, size_t out_size, const uint8_t *data, uint16_t len);

/* ============================================================================
 * Public API - Initialization
 * ============================================================================ */

void debug_uart_init(radio_message_queue_t *radio_q, gps_sample_queue_t *gps_q)
{
    // Store queue pointers
    radio_queue = radio_q;
    gps_queue = gps_q;

    // Initialize log queue
    log_queue.head = 0;
    log_queue.tail = 0;

    // Initialize RX position tracker
    last_rx_pos = 0;

    // Start DMA reception with IDLE interrupt
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, debug_uart_rx_buffer, DEBUG_UART_RX_BUFFER_SIZE);
}

/* ============================================================================
 * Public API - Injection (RX)
 * ============================================================================ */

void debug_uart_rx_event_callback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance != USART1) {
        return; // Safety check
    }

    // Calculate number of bytes received
    uint16_t bytes_received = Size;

    // Need at least 2 bytes (type + 1 byte payload)
    if (bytes_received < 2) {
        return; // Invalid message
    }

    // Read type byte
    uint8_t type_byte = debug_uart_rx_buffer[0];

    // Extract payload (everything after type byte)
    const uint8_t *payload = &debug_uart_rx_buffer[1];
    uint16_t payload_len = bytes_received - 1;

    // Route based on type byte
    if (type_byte == DEBUG_UART_TYPE_RADIO) {
        process_radio_injection(payload, payload_len);
    } else if (type_byte == DEBUG_UART_TYPE_GPS) {
        process_gps_injection(payload, payload_len);
    }
    // Ignore unknown type bytes
}

/* ============================================================================
 * Public API - Logging (TX)
 * ============================================================================ */

void debug_uart_log_radio(const uint8_t *msg, uint16_t len)
{
    if (msg == NULL || len == 0) {
        return;
    }

    char hex_ascii[128];
    format_hex_ascii(hex_ascii, sizeof(hex_ascii), msg, len);
    log_enqueue("[RADIO RX] %s\r\n", hex_ascii);
}

void debug_uart_log_gps_nmea(const char *nmea)
{
    if (nmea == NULL) {
        return;
    }

    log_enqueue("[GPS NMEA] %s\r\n", nmea);
}

void debug_uart_log_gps_fix(const gps_fix_t *fix)
{
    if (fix == NULL) {
        return;
    }

    log_enqueue("[GPS FIX] Lat: %.6f, Lon: %.6f, Alt: %.1fm, Speed: %.1fm/s, Sats: %u\r\n",
                fix->latitude,
                fix->longitude,
                fix->altitude_msl,
                fix->ground_speed,
                fix->num_satellites);
}

void debug_uart_log_spi_radio_tx(const uint8_t *msg, uint16_t len)
{
    if (msg == NULL || len == 0) {
        return;
    }

    char hex_ascii[128];
    format_hex_ascii(hex_ascii, sizeof(hex_ascii), msg, len);
    log_enqueue("[SPI TX] Radio msg from master: %s\r\n", hex_ascii);
}

void debug_uart_process_logs(void)
{
    // Process one message per call to avoid blocking too long
    if (log_queue_empty()) {
        return; // No messages to send
    }

    // Get message from tail
    const char *msg = log_queue.messages[log_queue.tail];
    uint16_t msg_len = strlen(msg);

    if (msg_len > 0) {
        // Transmit message (blocking with 100ms timeout)
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, msg_len, 100);
    }

    // Advance tail (circular)
    log_queue.tail = (log_queue.tail + 1) % DEBUG_UART_LOG_QUEUE_DEPTH;
}

/* ============================================================================
 * Internal Functions - Injection
 * ============================================================================ */

static void process_radio_injection(const uint8_t *payload, uint16_t len)
{
    if (radio_queue == NULL || payload == NULL || len == 0) {
        return;
    }

    // Find null terminator
    uint16_t msg_len = 0;
    for (uint16_t i = 0; i < len; i++) {
        if (payload[i] == 0x00) {
            msg_len = i;
            break;
        }
    }

    // If no null terminator found, use entire payload
    if (msg_len == 0) {
        msg_len = len;
    }

    // Validate message length (must fit in radio queue)
    if (msg_len > RADIO_MESSAGE_MAX_LEN) {
        return; // Too long
    }

    // Create temporary buffer with message
    uint8_t msg_buffer[RADIO_MESSAGE_MAX_LEN];
    memcpy(msg_buffer, payload, msg_len);

    // Enqueue to radio RX queue
    radio_message_enqueue(msg_len, msg_buffer, radio_queue);
}

static void process_gps_injection(const uint8_t *payload, uint16_t len)
{
    if (gps_queue == NULL || payload == NULL || len == 0) {
        return;
    }

    // Validate NMEA format (should start with '$')
    if (payload[0] != '$') {
        return; // Invalid NMEA
    }

    // Validate length (max 82 chars per NMEA spec)
    if (len > 82) {
        return; // Too long
    }

    // Create buffer and pad to GPS_SAMPLE_SIZE
    uint8_t gps_buffer[GPS_SAMPLE_SIZE];
    memset(gps_buffer, 0, GPS_SAMPLE_SIZE);
    memcpy(gps_buffer, payload, len);

    // Enqueue to GPS NMEA queue
    gps_sample_enqueue(gps_buffer, gps_queue);
}

/* ============================================================================
 * Internal Functions - Logging
 * ============================================================================ */

static void log_enqueue(const char *format, ...)
{
    if (log_queue_full()) {
        return; // Drop message if queue full
    }

    // Format message into queue buffer
    va_list args;
    va_start(args, format);
    vsnprintf(log_queue.messages[log_queue.head],
              DEBUG_UART_LOG_MSG_SIZE,
              format,
              args);
    va_end(args);

    // Advance head (circular)
    log_queue.head = (log_queue.head + 1) % DEBUG_UART_LOG_QUEUE_DEPTH;
}

static bool log_queue_full(void)
{
    return ((log_queue.head + 1) % DEBUG_UART_LOG_QUEUE_DEPTH) == log_queue.tail;
}

static bool log_queue_empty(void)
{
    return log_queue.head == log_queue.tail;
}

/* ============================================================================
 * Internal Functions - Utilities
 * ============================================================================ */

static void format_hex_ascii(char *out, size_t out_size, const uint8_t *data, uint16_t len)
{
    if (out == NULL || data == NULL || len == 0 || out_size < 10) {
        if (out != NULL && out_size > 0) {
            out[0] = '\0';
        }
        return;
    }

    size_t pos = 0;

    // Format hex bytes (limit to prevent overflow)
    uint16_t hex_limit = (len < 16) ? len : 16;
    for (uint16_t i = 0; i < hex_limit && pos < out_size - 20; i++) {
        pos += snprintf(&out[pos], out_size - pos, "%02X ", data[i]);
    }

    if (len > hex_limit) {
        pos += snprintf(&out[pos], out_size - pos, "... ");
    }

    // Add ASCII representation if printable
    pos += snprintf(&out[pos], out_size - pos, "(");
    bool has_printable = false;
    for (uint16_t i = 0; i < hex_limit && pos < out_size - 10; i++) {
        if (isprint(data[i])) {
            out[pos++] = (char)data[i];
            has_printable = true;
        }
    }

    if (!has_printable) {
        pos += snprintf(&out[pos], out_size - pos, "binary");
    }

    snprintf(&out[pos], out_size - pos, ")");
}

#endif // DEBUG
