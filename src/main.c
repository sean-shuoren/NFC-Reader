#include "main.h"

/* Timer Clock Scale */
uint32_t uwPrescalerValue = 0;

int main(void) {
	/* STM32F103xB HAL library initialization:
	   - Configure the Flash prefetch
	   - Systick timer is configured by default as source of time base, but user
		 can eventually implement his proper time base source (a general purpose
		 timer for example or other time source), keeping in mind that Time base
		 duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
		 handled in milliseconds basis.
	   - Set NVIC Group Priority to 4
	   - Low Level Initialization
	 */
	HAL_Init();
	/* Configure the system clock to 64 MHz */
	SystemClock_Config();
	/* Configure LED2, LED2 and LED2 */
	BSP_LED_Init(LED2);

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


	/* Set TRF7970 IRQ */
	GPIO_InitStruct.Mode  = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Pin   = TRF79XXA_IRQ_PIN;
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	/* GPIO Init */
	HAL_GPIO_Init(TRF79XXA_IRQ_GPIO_PORT, &GPIO_InitStruct);


	/* Set Timer instance */
	/* Compute the prescaler value to have TIMx counter clock equal to 10000 Hz
	 * Initialize TIMx peripheral as follows:
	 *	   + Period = 10000 - 1
	 *	   + Prescaler = (SystemCoreClock/10000) - 1
	 *	   + ClockDivision = 0
	 *	   + Counter direction = Up
	 */
	uwPrescalerValue = (uint32_t)(SystemCoreClock / 1000) - 1;

	TimHandle.Instance 				 = TIMx;
	TimHandle.Init.Period            = 1000 - 1;
	TimHandle.Init.Prescaler         = uwPrescalerValue;
	TimHandle.Init.ClockDivision     = 0;
	TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
	TimHandle.Init.RepetitionCounter = 0;

	if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
		Error_Handler();

	Delay_TimHandle.Instance 			   = DELAY_TIM;
	Delay_TimHandle.Init.Period            = 1000 - 1;
	Delay_TimHandle.Init.Prescaler         = uwPrescalerValue;
	Delay_TimHandle.Init.ClockDivision     = 0;
	Delay_TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
	Delay_TimHandle.Init.RepetitionCounter = 0;

	if (HAL_TIM_Base_Init(&Delay_TimHandle) != HAL_OK)
		Error_Handler();

	/**************************
	* Controlling TRF7970ABP *
	**************************/

	// Set the SPI SS high
	HAL_GPIO_WritePin(SPIx_SS_GPIO_PORT, SPIx_SS_PIN,  GPIO_PIN_SET);
	// Set TRF Enable Pin high
	HAL_GPIO_WritePin(TRF_EN_GPIO_PORT, TRF_EN_PIN, GPIO_PIN_SET);
	// Wait until TRF system clock started
	delayMilliSeconds(2);

	// Set up TRF initial settings
	TRF79xxA_initialSettings();
	// Initialize all Global Variables for the ISO15693 layer
	TRF79xxA_ISO15693_init();

	// Poll for NFC tags
	while (1) {

		// Temperate variables
		uint8_t ui8TagFound = STATUS_FAIL;
		uint8_t ui8AddressedFlag = 0x00;

		// This function performs RF collision avoidance as outlined in TI application notes (sloa192, sloa227)
		// in order to determine if an external RF field is present.
		if (TRF79xxA_checkExternalRfField() == true)
			Error_Handler();

		// Configure the TRF79xxA for ISO15693 @ High Bit Rate, One Subcarrier, 1 out of 4
		TRF79xxA_setupInitiator(0x02);

		// Reset tag count
		g_ui8TagDetectedCount = 0;

		// Send a single slot inventory request to try and detect a single ISO15693 Tag
		// STATUS_FAIL == 0x01
		ui8TagFound = TRF79xxA_ISO15693_sendSingleSlotInventory();

		// RSSI value is the receive signal strength
		// Format: 01 - [3 bits auxiliary channel RSSI] - [3 bits main channel RSSI]
		uint8_t RSSI = TRF79xxA_readRegister(TRF79XXA_RSSI_LEVELS);

		// Inventory failed - search with full anticollision routine
		if (ui8TagFound == STATUS_FAIL) {
			// Clear the recursion counter
			g_ui8RecursionCount = 0;
			// Delay before issuing the anticollision commmand
			delayMilliSeconds(5);

			// Send 16 Slot Inventory request with no mask length and no AFI
			 ui8TagFound = TRF79xxA_ISO15693_runAnticollision(0x06, 0x00, 0x00);

			// Collision occurred, send addressed commands
			ui8AddressedFlag = 0x20;
		}

		if (ui8TagFound == STATUS_SUCCESS) {
			if (g_ui8TagDetectedCount == 1) {
				// Read an ISO15693 tag
				// TRF79xxA_ISO15693_ReadTag(0x02 | ui8AddressedFlag);

				// Read an ISO15693 tag which has extended protocol implemented
				TRF79xxA_ISO15693_ReadExtendedTag(0x0A | ui8AddressedFlag);

				// Example to read 25 blocks starting @ Block 0 from a tag which supports Read Multiple Block command
				// TRF79xxA_ISO15693_sendReadMultipleBlocks(0x22,0x00,25);
			}
		}

		// Turn off RF field once done reading the tag(s)
		TRF79xxA_turnRfOff();
	}

	// End of the transmit
	// Set the SPI SS High
	HAL_GPIO_WritePin(SPIx_SS_GPIO_PORT, SPIx_SS_PIN,  GPIO_PIN_SET);
	// Set TRF Enable Pin Low
	HAL_Delay(2);
	HAL_GPIO_WritePin(TRF_EN_GPIO_PORT, TRF_EN_PIN, GPIO_PIN_RESET);

	while(1);
}


/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 64000000
  *            HCLK(Hz)                       = 64000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            PLLMUL                         = 16
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
void SystemClock_Config(void) {
	RCC_ClkInitTypeDef clkinitstruct = {0};
	RCC_OscInitTypeDef oscinitstruct = {0};

	/* Configure PLL ------------------------------------------------------*/
	/* PLL configuration: PLLCLK = (HSI / 2) * PLLMUL = (8 / 2) * 16 = 64 MHz */
	/* PREDIV1 configuration: PREDIV1CLK = PLLCLK / HSEPredivValue = 64 / 1 = 64 MHz */
	/* Enable HSI and activate PLL with HSi_DIV2 as source */
	oscinitstruct.OscillatorType  = RCC_OSCILLATORTYPE_HSI;
	oscinitstruct.HSEState        = RCC_HSE_OFF;
	oscinitstruct.LSEState        = RCC_LSE_OFF;
	oscinitstruct.HSIState        = RCC_HSI_ON;
	oscinitstruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	oscinitstruct.HSEPredivValue    = RCC_HSE_PREDIV_DIV1;
	oscinitstruct.PLL.PLLState    = RCC_PLL_ON;
	oscinitstruct.PLL.PLLSource   = RCC_PLLSOURCE_HSI_DIV2;
	oscinitstruct.PLL.PLLMUL      = RCC_PLL_MUL16;
	if (HAL_RCC_OscConfig(&oscinitstruct)!= HAL_OK) {
		while(1);			/* Initialization Error */
	}

	/* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
	 clocks dividers */
	clkinitstruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	clkinitstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	clkinitstruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	clkinitstruct.APB2CLKDivider = RCC_HCLK_DIV1;
	clkinitstruct.APB1CLKDivider = RCC_HCLK_DIV2;
	if (HAL_RCC_ClockConfig(&clkinitstruct, FLASH_LATENCY_2)!= HAL_OK) {
		while(1);			/* Initialization Error*/
	}
}

/**
  * Toggle LED2 for error
  */
void Error_Handler(void) {
	while(1) {
		BSP_LED_Toggle(LED2);
		HAL_Delay(1000);
	}
}

