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
#define RX_BUFFER_SIZE      255  // Radio messages from FC

// Data type identifiers sent to FC
typedef enum {
    SPI_DATA_GPS = 0x01,
    SPI_DATA_RADIO = 0x02,
} spi_data_type_t;

// SPI handler state machine states
// INT line arbitrates: LOW = RX mode (FC can write), HIGH = TX mode (FC should read)
typedef enum {
    SPI_STATE_IDLE,     // RX DMA armed, INT low, waiting for FC write or tick()
    SPI_STATE_TX_TYPE,  // TX DMA armed with type byte, INT high, waiting for FC read
    SPI_STATE_TX_DATA,  // TX DMA armed with payload, INT high, waiting for FC read
} spi_handler_state_t;

// Callback type for forwarding received radio messages to UART TX
typedef void (*radio_tx_callback_t)(uint8_t *data, uint16_t len);

// Current TX transaction context
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
    
    // Ping-pong RX buffers
    uint8_t *rx_active_buffer;    // Currently receiving into
    uint8_t *rx_complete_buffer;  // Ready to process
    
    // Callback for forwarding radio messages to UART
    radio_tx_callback_t radio_tx_callback;
} spi_handler_t;

// Initialize the SPI handler with queue pointers (arms RX DMA, starts in IDLE)
void init_spi_handler(radio_message_queue_t *radio_queue, gps_sample_queue_t *gps_queue);

// Called from main loop - transitions IDLE -> TX_TYPE when data available
void tick_spi_handler(void);

// Called from HAL_SPI_TxCpltCallback when TX DMA completes
void spi_tx_complete(void);

// Called from HAL_SPI_RxCpltCallback when RX DMA completes (255 bytes received)
void spi_rx_complete(void);

// Set the board state (called when errors occur)
void spi_handler_set_board_state(board_state_t state);

// Set callback for forwarding received radio messages to UART TX
void spi_handler_set_radio_tx_callback(radio_tx_callback_t callback);

#endif
