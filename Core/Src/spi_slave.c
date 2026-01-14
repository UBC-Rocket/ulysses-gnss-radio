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

// Buffer for type/status byte transmission
static uint8_t tx_type_buffer[1];

// --------------------------------------------------------------------------
// Internal helpers
// --------------------------------------------------------------------------

static void assert_data_ready(void) {
    HAL_GPIO_WritePin(DATA_READY_PORT, DATA_READY_PIN, GPIO_PIN_SET);
}

static void deassert_data_ready(void) {
    HAL_GPIO_WritePin(DATA_READY_PORT, DATA_READY_PIN, GPIO_PIN_RESET);
}

static void arm_status_dma(void) {
    tx_type_buffer[0] = (uint8_t)spi_handler.board_state;
    HAL_SPI_Transmit_DMA(&hspi1, tx_type_buffer, 1);
}

static void arm_type_dma(uint8_t type_byte) {
    tx_type_buffer[0] = type_byte;
    HAL_SPI_Transmit_DMA(&hspi1, tx_type_buffer, 1);
}

static void arm_data_dma(uint8_t *data, uint16_t size) {
    HAL_SPI_Transmit_DMA(&hspi1, data, size);
}

// Try to setup next data transaction from queues
// Returns true if data was found and transaction was setup
static bool setup_next_transaction(void) {
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
    
    // Ensure data ready line is low initially
    deassert_data_ready();
    
    // Arm DMA with status - always ready to respond
    spi_handler.state = SPI_STATE_STATUS_ARMED;
    arm_status_dma();
}

void spi_handler_set_board_state(board_state_t state) {
    spi_handler.board_state = state;
}

void tick_spi_handler(void) {
    // Only preempt when armed with status (no data was ready)
    if (spi_handler.state != SPI_STATE_STATUS_ARMED) {
        return;
    }
    
    // Check if data is now available
    if (setup_next_transaction()) {
        // Data available! Abort the status DMA and switch to type
        HAL_SPI_Abort(&hspi1);
        
        // Assert interrupt to signal FC
        assert_data_ready();
        
        // Arm DMA with type byte
        spi_handler.state = SPI_STATE_TYPE_ARMED;
        arm_type_dma((uint8_t)spi_handler.current_transaction.data_type);
    }
    // else: stay in STATUS_ARMED, DMA already armed with board state
}

void spi_dma_complete(void) {
    switch (spi_handler.state) {
        case SPI_STATE_STATUS_ARMED:
            // FC polled us while no data ready
            // Re-arm with current board state (may have changed)
            arm_status_dma();
            break;
            
        case SPI_STATE_TYPE_ARMED:
            // FC read the type byte, now arm for payload
            spi_handler.state = SPI_STATE_DATA_ARMED;
            arm_data_dma(spi_handler.current_transaction.tx_buffer,
                         spi_handler.current_transaction.tx_size);
            break;
            
        case SPI_STATE_DATA_ARMED:
            // FC finished reading payload - pop from queue
            if (spi_handler.current_transaction.data_type == SPI_DATA_GPS) {
                gps_sample_queue_pop(spi_handler.gps_queue);
            } else if (spi_handler.current_transaction.data_type == SPI_DATA_RADIO) {
                radio_message_queue_pop(spi_handler.radio_queue);
            }
            
            // Check for more data
            if (setup_next_transaction()) {
                // More data available - stay in type armed, keep INT high
                spi_handler.state = SPI_STATE_TYPE_ARMED;
                arm_type_dma((uint8_t)spi_handler.current_transaction.data_type);
            } else {
                // No more data - deassert INT and arm status
                deassert_data_ready();
                spi_handler.state = SPI_STATE_STATUS_ARMED;
                arm_status_dma();
            }
            break;
    }
}
