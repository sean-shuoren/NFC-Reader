#include <spi.h>

/**
  * @brief SPI MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  *           - DMA configuration for transmission request by peripheral
  *           - NVIC configuration for DMA interrupt request enable
  * @param hspi: SPI handle pointer
  * @retval None
  */
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi) {
	GPIO_InitTypeDef  GPIO_InitStruct;

	if (hspi->Instance == SPIx) {
		/*##-1- Enable peripherals and GPIO Clocks #################################*/
		/* Enable GPIO TX/RX clock */
		SPIx_SCK_GPIO_CLK_ENABLE();
		SPIx_MISO_GPIO_CLK_ENABLE();
		SPIx_MOSI_GPIO_CLK_ENABLE();
		/* Enable SPI2 clock */
		SPIx_CLK_ENABLE();

		/*##-2- Configure peripheral GPIO ##########################################*/

		/* SPI SS GPIO pin configuration  */
		GPIO_InitStruct.Pin       = SPIx_SS_PIN;
		GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Pull      = GPIO_PULLUP;
		HAL_GPIO_Init(SPIx_SS_GPIO_PORT, &GPIO_InitStruct);

		/* SPI SCK GPIO pin configuration  */
		GPIO_InitStruct.Pin       = SPIx_SCK_PIN;
		GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull      = GPIO_NOPULL;
		GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(SPIx_SCK_GPIO_PORT, &GPIO_InitStruct);

		/* SPI MISO GPIO pin configuration  */
		GPIO_InitStruct.Pin 	  = SPIx_MISO_PIN;
		GPIO_InitStruct.Mode      = GPIO_MODE_INPUT;
		HAL_GPIO_Init(SPIx_MISO_GPIO_PORT, &GPIO_InitStruct);

		/* SPI MOSI GPIO pin configuration  */
		GPIO_InitStruct.Pin 	  = SPIx_MOSI_PIN;
		GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
		HAL_GPIO_Init(SPIx_MOSI_GPIO_PORT, &GPIO_InitStruct);

	    /*##-3- Configure the NVIC for SPI #########################################*/
	    /* NVIC for SPI */
	    HAL_NVIC_SetPriority(SPIx_IRQn, 1, 0);
	    HAL_NVIC_EnableIRQ(SPIx_IRQn);
	}
}

/**
  * @brief SPI MSP De-Initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO, DMA and NVIC configuration to their default state
  * @param hspi: SPI handle pointer
  * @retval None
  */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi) {
	if(hspi->Instance == SPIx) {
		/*##-1- Disable peripherals and GPIO Clocks ################################*/
		/* Configure SPI SCK as alternate function  */
		HAL_GPIO_DeInit(SPIx_SCK_GPIO_PORT, SPIx_SCK_PIN);
		/* Configure SPI MISO as alternate function  */
		HAL_GPIO_DeInit(SPIx_MISO_GPIO_PORT, SPIx_MISO_PIN);
		/* Configure SPI MOSI as alternate function  */
		HAL_GPIO_DeInit(SPIx_MOSI_GPIO_PORT, SPIx_MOSI_PIN);
	}
}


/**
  * @brief: Set up
  * @param
  * @retval
  */
void SPI_Config(void)
{
	/*  SPI initialization */
	SpiHandle.Instance               = SPIx;
	SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
	SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
	SpiHandle.Init.CLKPhase          = SPI_PHASE_2EDGE;
	SpiHandle.Init.CLKPolarity       = SPI_POLARITY_LOW;
	SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
	SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
	SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLE;
	SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
	SpiHandle.Init.CRCPolynomial     = 7;
	SpiHandle.Init.NSS               = SPI_NSS_SOFT;
	SpiHandle.Init.Mode 			 = SPI_MODE_MASTER;

	/* HAL_SPI_Init also calls HAL_SPI_MspInit() in stm32f1xx_hal_msp.c
	 * which also initialize GPIO for SS and EN signals */
	if(HAL_SPI_Init(&SpiHandle) != HAL_OK) {
		Error_Handler();
	}

	/* SPI block is enabled prior calling SPI transmit/receive functions, in order to get CLK signal properly pulled down.
	 Otherwise, SPI CLK signal is not clean on this board and leads to errors during transfer */
	__HAL_SPI_ENABLE(&SpiHandle);
}


