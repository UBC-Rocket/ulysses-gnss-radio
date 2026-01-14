#include "spi_slave.h"
#include "stm32g0xx_hal.h"

// External SPI handle from main.c
extern SPI_HandleTypeDef hspi1;

// Data ready interrupt pin - directly drives FC interrupt line
// Using PB2 as the data ready signal
#define DATA_READY_PORT GPIOB
#define DATA_READY_PIN  GPIO_PIN_2

// Static handler instance
static spi_handler_t spi_handler;

// Buffer for type byte transmission
static uint8_t tx_type_buffer[1];

// Ping-pong RX buffers for receiving radio messages from FC
static uint8_t rx_buffer_a[RX_BUFFER_SIZE];
static uint8_t rx_buffer_b[RX_BUFFER_SIZE];

// --------------------------------------------------------------------------
// Internal helpers
// --------------------------------------------------------------------------

static void assert_data_ready(void) {
    HAL_GPIO_WritePin(DATA_READY_PORT, DATA_READY_PIN, GPIO_PIN_SET);
}

static void deassert_data_ready(void) {
    HAL_GPIO_WritePin(DATA_READY_PORT, DATA_READY_PIN, GPIO_PIN_RESET);
}

static void swap_ping_pong_buffers(void) {
    uint8_t *temp = spi_handler.rx_active_buffer;
    spi_handler.rx_active_buffer = spi_handler.rx_complete_buffer;
    spi_handler.rx_complete_buffer = temp;
}

static void arm_rx_dma(void) {
    HAL_SPI_Receive_DMA(&hspi1, spi_handler.rx_active_buffer, RX_BUFFER_SIZE);
}

static void arm_tx_type_dma(uint8_t type_byte) {
    tx_type_buffer[0] = type_byte;
    HAL_SPI_Transmit_DMA(&hspi1, tx_type_buffer, 1);
}

static void arm_tx_data_dma(uint8_t *data, uint16_t size) {
    HAL_SPI_Transmit_DMA(&hspi1, data, size);
}

// Process received radio message and forward via callback
static void process_rx_message(uint8_t *buffer) {
    if (spi_handler.radio_tx_callback != NULL) {
        // Forward the entire buffer to the radio TX callback
        // The callback is responsible for UART transmission to radio module
        spi_handler.radio_tx_callback(buffer, RX_BUFFER_SIZE);
    }
}

// Try to setup next TX transaction from queues
// Returns true if data was found and transaction was setup
static bool setup_next_tx_transaction(void) {
    // Priority: GPS first, then Radio
    
    // Check GPS queue
    if (!gps_sample_queue_empty(spi_handler.gps_queue)) {
        if (gps_sample_queue_tail_pointer(spi_handler.gps_queue,
                                          &spi_handler.current_transaction.tx_buffer)) {
            spi_handler.current_transaction.data_type = SPI_DATA_GPS;
            spi_handler.current_transaction.tx_size = GPS_PAYLOAD_SIZE;
            return true;
        }
    }
    
    // Check Radio queue
    if (!radio_message_queue_empty(spi_handler.radio_queue)) {
        if (radio_message_queue_tail_pointer(spi_handler.radio_queue,
                                             &spi_handler.current_transaction.tx_buffer)) {
            spi_handler.current_transaction.data_type = SPI_DATA_RADIO;
            spi_handler.current_transaction.tx_size = RADIO_PAYLOAD_SIZE;
            return true;
        }
    }
    
    return false;
}

static void pop_current_tx_message(void) {
    if (spi_handler.current_transaction.data_type == SPI_DATA_GPS) {
        gps_sample_queue_pop(spi_handler.gps_queue);
    } else if (spi_handler.current_transaction.data_type == SPI_DATA_RADIO) {
        radio_message_queue_pop(spi_handler.radio_queue);
    }
}

// --------------------------------------------------------------------------
// Public API
// --------------------------------------------------------------------------

void init_spi_handler(radio_message_queue_t *radio_queue, gps_sample_queue_t *gps_queue) {
    spi_handler.radio_queue = radio_queue;
    spi_handler.gps_queue = gps_queue;
    spi_handler.board_state = BOARD_STATE_IDLE;
    
    spi_handler.current_transaction.data_type = SPI_DATA_GPS;
    spi_handler.current_transaction.tx_buffer = NULL;
    spi_handler.current_transaction.tx_size = 0;
    
    // Initialize ping-pong buffers
    spi_handler.rx_active_buffer = rx_buffer_a;
    spi_handler.rx_complete_buffer = rx_buffer_b;
    
    // No radio TX callback initially
    spi_handler.radio_tx_callback = NULL;
    
    // Ensure data ready line is low (RX mode)
    deassert_data_ready();
    
    // Start in IDLE state with RX DMA armed
    spi_handler.state = SPI_STATE_IDLE;
    arm_rx_dma();
}

void spi_handler_set_board_state(board_state_t state) {
    spi_handler.board_state = state;
}

void spi_handler_set_radio_tx_callback(radio_tx_callback_t callback) {
    spi_handler.radio_tx_callback = callback;
}

void tick_spi_handler(void) {
    // Only transition from IDLE when data is available
    if (spi_handler.state != SPI_STATE_IDLE) {
        return;
    }
    
    // Check if data is available to send
    if (setup_next_tx_transaction()) {
        // Data available! Abort RX DMA and switch to TX mode
        HAL_SPI_Abort(&hspi1);
        
        // Assert interrupt to signal FC we have data
        assert_data_ready();
        
        // Arm TX DMA with type byte
        spi_handler.state = SPI_STATE_TX_TYPE;
        arm_tx_type_dma((uint8_t)spi_handler.current_transaction.data_type);
    }
    // else: stay in IDLE, RX DMA already armed
}

// Called from HAL_SPI_TxCpltCallback when TX DMA completes
void spi_tx_complete(void) {
    switch (spi_handler.state) {
        case SPI_STATE_TX_TYPE:
            // FC read the type byte, now arm payload
            spi_handler.state = SPI_STATE_TX_DATA;
            arm_tx_data_dma(spi_handler.current_transaction.tx_buffer,
                            spi_handler.current_transaction.tx_size);
            break;
            
        case SPI_STATE_TX_DATA:
            // FC finished reading payload - pop from queue
            pop_current_tx_message();
            
            // Check for more data to send
            if (setup_next_tx_transaction()) {
                // More data available - arm next type, keep INT high
                spi_handler.state = SPI_STATE_TX_TYPE;
                arm_tx_type_dma((uint8_t)spi_handler.current_transaction.data_type);
            } else {
                // No more data - deassert INT and return to IDLE (RX mode)
                deassert_data_ready();
                spi_handler.state = SPI_STATE_IDLE;
                arm_rx_dma();
            }
            break;
            
        case SPI_STATE_IDLE:
            // Shouldn't happen - TX complete while in IDLE
            break;
    }
}

// Called from HAL_SPI_RxCpltCallback when RX DMA completes (255 bytes received)
void spi_rx_complete(void) {
    // RX complete should only happen in IDLE state
    if (spi_handler.state != SPI_STATE_IDLE) {
        // Unexpected RX complete - ignore or handle error
        return;
    }
    
    // FC just wrote 255 bytes to us (radio message)
    // Swap ping-pong buffers so we can process the completed one
    swap_ping_pong_buffers();
    
    // Process the received message (forward to radio via callback)
    process_rx_message(spi_handler.rx_complete_buffer);
    
    // Re-arm RX DMA for next message
    arm_rx_dma();
}
