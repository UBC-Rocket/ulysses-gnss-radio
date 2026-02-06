/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "spi_slave.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint8_t radio_rx_byte;
static uint8_t radio_temp_buffer[RADIO_MESSAGE_MAX_LEN];
static uint8_t radio_temp_index = 0;

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
// extern DMA_HandleTypeDef hdma_spi1_tx;  // Not used - SPI slave uses register-level SPI2
// extern DMA_HandleTypeDef hdma_spi1_rx;  // Not used - SPI slave uses register-level SPI2
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;
/* USER CODE BEGIN EV */
extern radio_message_queue_t radio_message_queue;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel 1 interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */
  // SPI2 RX DMA handler (for SPI slave)
  spi_slave_dma1_ch1_irq_handler();
  /* USER CODE END DMA1_Channel1_IRQn 0 */
  // HAL_DMA_IRQHandler(&hdma_spi1_tx);  // Disabled - not using HAL SPI1
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel 2 and channel 3 interrupts.
  */
void DMA1_Channel2_3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_3_IRQn 0 */
  // SPI2 TX DMA uses channel 2, but we don't need an interrupt handler for it
  // (we don't care about TX completion - NSS rising edge handles transaction end)
  /* USER CODE END DMA1_Channel2_3_IRQn 0 */
  // HAL_DMA_IRQHandler(&hdma_spi1_rx);  // Disabled - not using HAL SPI1
  /* USER CODE BEGIN DMA1_Channel2_3_IRQn 1 */

  /* USER CODE END DMA1_Channel2_3_IRQn 1 */
}

/**
  * @brief This function handles USART3, USART4, USART5, USART6, LPUART1 globlal Interrupts (combined with EXTI 28).
  */
void USART3_4_5_6_LPUART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_4_5_6_LPUART1_IRQn 0 */

  if (huart5.Instance->ISR & USART_ISR_RXNE_RXFNE)
  {
      radio_rx_byte = (uint8_t)(huart5.Instance->RDR);
      
      if (radio_rx_byte == 0x00)
      {
          if (radio_temp_index > 0)
          {
              radio_message_enqueue(radio_temp_index, 
                                  radio_temp_buffer,
                                  &radio_message_queue);
              radio_temp_index = 0;
          }
      }
      else
      {
          if (radio_temp_index < RADIO_MESSAGE_MAX_LEN)
          {
              radio_temp_buffer[radio_temp_index] = radio_rx_byte;
              radio_temp_index++;
          }
          else
          {
              radio_temp_index = 0;
          }
      }
  }
  /* USER CODE END USART3_4_5_6_LPUART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart5);
  HAL_UART_IRQHandler(&huart6);
}

/* USER CODE BEGIN 1 */

/**
  * @brief This function handles SPI2/SPI3 global interrupt.
  *
  * SPI2 is used for the SPI slave (GNSS/Radio board).
  * RXNE interrupt captures the command byte.
  * STM32G0B1 has combined SPI2_3_IRQn
  */
void SPI2_3_IRQHandler(void)
{
  spi_slave_spi2_irq_handler();
}

/**
  * @brief This function handles EXTI line 4 to 15 interrupts.
  *
  * PB12 (SPI2_NSS) is configured as EXTI12 for transaction end detection.
  */
void EXTI4_15_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_15_IRQn 0 */
  // SPI2 NSS rising edge handler (for SPI slave transaction end)
  spi_slave_nss_exti_handler();
  /* USER CODE END EXTI4_15_IRQn 0 */
  // HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);  // Disabled - not using HAL SPI1
  /* USER CODE BEGIN EXTI4_15_IRQn 1 */

  /* USER CODE END EXTI4_15_IRQn 1 */
}

/* USER CODE END 1 */
