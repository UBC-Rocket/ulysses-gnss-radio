/**
 * @file radio_driver.c
 * @brief Radio transceiver driver implementation
 *
 * Uses UART5 for radio communication. Messages are framed with null terminators.
 * RX: Bytes accumulate until 0x00, then message is enqueued.
 * TX: Data is sent followed by 0x00.
 */

#include "radio_driver.h"
#include "stm32g0xx_hal.h"
#include <string.h>

// ============================================================================
// EXTERNAL REFERENCES
// ============================================================================

extern UART_HandleTypeDef huart5;  // Radio UART (defined in main.c)

// ============================================================================
// PRIVATE STATE
// ============================================================================

static radio_message_queue_t *rx_queue = NULL;

// RX accumulation buffer
static uint8_t rx_temp_buffer[RADIO_MAX_MESSAGE_LEN];
static uint16_t rx_temp_index = 0;

// ============================================================================
// PUBLIC API IMPLEMENTATION
// ============================================================================

void radio_init(radio_message_queue_t *queue) {
    rx_queue = queue;
    radio_message_queue_init(rx_queue);
    
    // Reset RX state
    rx_temp_index = 0;
    memset(rx_temp_buffer, 0, sizeof(rx_temp_buffer));
    
    // Enable UART RX interrupt (RXNE)
    // The actual byte handling is done in radio_rx_byte_handler(),
    // called from the UART ISR in stm32g0xx_it.c
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_RXNE);
}

bool radio_send(const uint8_t *data, uint8_t len) {
    if (data == NULL || len == 0 || len > 254) {
        return false;
    }
    
    // Send data bytes
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart5, (uint8_t*)data, len, 1000);
    if (status != HAL_OK) {
        return false;
    }
    
    // Send null terminator
    uint8_t null_byte = 0x00;
    status = HAL_UART_Transmit(&huart5, &null_byte, 1, 100);
    
    return (status == HAL_OK);
}

bool radio_available(void) {
    if (rx_queue == NULL) {
        return false;
    }
    return !radio_message_queue_empty(rx_queue);
}

uint8_t radio_read(uint8_t *buffer) {
    if (rx_queue == NULL || buffer == NULL) {
        return 0;
    }
    
    if (radio_message_queue_empty(rx_queue)) {
        return 0;
    }
    
    // Dequeue the oldest message
    radio_message_dequeue(rx_queue, buffer);
    
    // Find the actual length (messages are null-padded)
    uint8_t len = 0;
    while (len < RADIO_MAX_MESSAGE_LEN && buffer[len] != 0) {
        len++;
    }
    
    return len;
}

uint8_t radio_rx_count(void) {
    if (rx_queue == NULL) {
        return 0;
    }
    
    if (radio_message_queue_empty(rx_queue)) {
        return 0;
    }
    
    return (rx_queue->head - rx_queue->tail + RADIO_MESSAGE_QUEUE_LEN) % RADIO_MESSAGE_QUEUE_LEN;
}

radio_message_queue_t* radio_get_rx_queue(void) {
    return rx_queue;
}

// ============================================================================
// ISR INTERFACE
// ============================================================================

void radio_rx_byte_handler(uint8_t byte) {
    if (rx_queue == NULL) {
        return;
    }
    
    if (byte == 0x00) {
        // Null terminator - end of message
        if (rx_temp_index > 0) {
            // Enqueue the accumulated message
            radio_message_enqueue(rx_temp_index, rx_temp_buffer, rx_queue);
            
            // Reset for next message
            rx_temp_index = 0;
            memset(rx_temp_buffer, 0, sizeof(rx_temp_buffer));
        }
    } else {
        // Accumulate byte
        if (rx_temp_index < RADIO_MAX_MESSAGE_LEN) {
            rx_temp_buffer[rx_temp_index] = byte;
            rx_temp_index++;
        } else {
            // Buffer overflow - discard and start over
            rx_temp_index = 0;
            memset(rx_temp_buffer, 0, sizeof(rx_temp_buffer));
        }
    }
}
