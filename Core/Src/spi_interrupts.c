#include "stm32g0xx_hal.h"
#include "spi_slave.h"

// ============================================================================
// SPI DMA COMPLETE CALLBACKS
// ============================================================================

/**
 * @brief SPI TX/RX complete callback (full-duplex DMA complete)
 *
 * Called when HAL_SPI_TransmitReceive_DMA completes.
 * This is our primary callback for the new protocol.
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI1) {
        spi_slave_txrx_complete();
    }
}

/**
 * @brief SPI TX complete callback (legacy, kept for compatibility)
 *
 * Not used in new protocol (we use TransmitReceive, not separate TX/RX)
 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    // Not used in new protocol
    (void)hspi;
}

/**
 * @brief SPI RX complete callback (legacy, kept for compatibility)
 *
 * Not used in new protocol (we use TransmitReceive, not separate TX/RX)
 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
    // Not used in new protocol
    (void)hspi;
}

// ============================================================================
// GPIO EXTI CALLBACKS (CS LINE MONITORING)
// ============================================================================

/**
 * @brief GPIO EXTI callback
 *
 * Called on CS (PA4) rising/falling edges for collision detection
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_4) {  // PA4 = CS/NSS
        // Check if CS is low (falling edge) or high (rising edge)
        GPIO_PinState cs_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);

        if (cs_state == GPIO_PIN_RESET) {
            // CS falling edge (master starting transaction)
            spi_slave_cs_falling_edge();
        } else {
            // CS rising edge (master ending transaction)
            spi_slave_cs_rising_edge();
        }
    }
}
