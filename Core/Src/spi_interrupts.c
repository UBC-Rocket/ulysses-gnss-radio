#include "stm32g0xx_hal.h"
#include "spi_slave.h"

// SPI DMA complete callbacks - these drive the state machine

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI1) {
        spi_tx_complete();
    }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI1) {
        spi_rx_complete();
    }
}
