#include "stm32g0xx_hal.h"
#include "spi_slave.h"

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI1) {
        spi_dma_complete();
    }
}
