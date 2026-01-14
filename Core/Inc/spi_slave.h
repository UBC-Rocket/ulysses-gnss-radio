#ifndef SPI_SLAVE_H
#define SPI_SLAVE_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "radio_queue.h"
#include "gps_queue.h"

// Payload sizes for SPI transfers
#define GPS_PAYLOAD_SIZE    64
#define RADIO_PAYLOAD_SIZE  255

// Data type identifiers sent to FC
typedef enum {
    SPI_DATA_GPS = 0x01,
    SPI_DATA_RADIO = 0x02,
} spi_data_type_t;

// SPI handler state machine states (always armed, never idle)
typedef enum {
    SPI_STATE_STATUS_ARMED, // DMA armed with board state, INT low, waiting for FC poll
    SPI_STATE_TYPE_ARMED,   // DMA armed with type byte, INT high, waiting for FC read
    SPI_STATE_DATA_ARMED,   // DMA armed with payload, INT high, waiting for FC read
} spi_handler_state_t;

// Current transaction context
typedef struct {
    spi_data_type_t data_type;
    uint8_t *tx_buffer;
    uint16_t tx_size;
} spi_transaction_t;

// Main SPI handler structure
typedef struct {
    radio_message_queue_t *radio_queue;
    gps_sample_queue_t *gps_queue;

    spi_handler_state_t state;
    spi_transaction_t current_transaction;
    
    board_state_t board_state;
} spi_handler_t;

// Initialize the SPI handler with queue pointers (arms status DMA immediately)
void init_spi_handler(radio_message_queue_t *radio_queue, gps_sample_queue_t *gps_queue);

// Called from main loop - preempts status with data if available
void tick_spi_handler(void);

// Called from DMA complete interrupt
void spi_dma_complete(void);

// Set the board state (called when errors occur)
void spi_handler_set_board_state(board_state_t state);

#endif
