#include "main.h"

/* Timer Clock Scale */
//uint32_t uwPrescalerValue = 0;

int main(void)
{
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
	/* Interrupt Priority */
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);

	/* SPI initialization: MISO, MOSI, SCK, SS */
	SPI_Config();
	/* TRF Reader board EN GPIO pin configuration  */
	GPIO_Config_EN();
	/* Set TRF7970 IRQ */
	GPIO_Config_IRQ();
	/* Set Timer instance */
	Timer_Config_Wait();
	Timer_Config_Delay();
	/* Initialize UART */
	UART1_Init();


	/*************************** Controlling TRF7970ABP ***************************/

	/* Initialize lastUID */
	int i;
	for (i = 0; i < 8; ++i)
		lastReadUID[i] = 0xFF;
	/* Data structure for transmitting data on UART */
	struct dataPacket pkt;


	// Set the SPI SS high
	HAL_GPIO_WritePin(SPIx_SS_GPIO_PORT, SPIx_SS_PIN,  GPIO_PIN_SET);
	// Set TRF Enable Pin high
	HAL_GPIO_WritePin(TRF79XXA_EN_GPIO_PORT, TRF79XXA_EN_PIN, GPIO_PIN_SET);
	// Wait until TRF system clock started
	delayMilliSeconds(2);

	// Set up TRF initial settings
	TRF79xxA_initialSettings();
	// Initialize all Global Variables for the ISO15693 layer
	TRF79xxA_ISO15693_init();

	// Poll for NFC tags
	while (1)
	{
		// Temperate variables
		uint8_t ui8TagFound = STATUS_FAIL;
		uint8_t ui8AddressedFlag = 0x00;
		g_ui8TagDetectedCount = 0;

		// This function performs RF collision avoidance as outlined in TI application notes (sloa192, sloa227)
		// in order to determine if an external RF field is present.
#ifdef ANTI_COLLISION
		if (TRF79xxA_checkExternalRfField() == true)
		 	Error_Handler();
#endif

		// Configure the TRF79xxA for ISO15693 @ High Bit Rate, One Subcarrier, 1 out of 4
		TRF79xxA_setupInitiator(0x02);

		// Send a single slot inventory request to try and detect a single ISO15693 Tag
		ui8TagFound = TRF79xxA_ISO15693_sendSingleSlotInventory();		/* STATUS_FAIL == 0x01 */

		// Inventory failed - search with full anticollision routine
#ifdef ANTI_COLLISION
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
#endif

		if (ui8TagFound == STATUS_SUCCESS && g_ui8TagDetectedCount == 1)
		{
			// RSSI value is the receive signal strength
			uint8_t RSSI = TRF79xxA_readRegister(TRF79XXA_RSSI_LEVELS);		/* [01]-[3 bits auxiliary channel RSSI]-[3 bits main channel RSSI] */

			if (compareUID(g_pui8Iso15693UId, lastReadUID) == ID_DIFF)
			{
				// Read an ISO15693 tag which has extended protocol implemented
				if (TRF79xxA_ISO15693_ReadExtendedTag(0x0A | ui8AddressedFlag, pkt.tag_data) == STATUS_SUCCESS)
				{
					// Fill in data sections
					copyUID(pkt.uid, g_pui8Iso15693UId);
					pkt.tag_data_p[0] = pkt.tag_data[32];
					pkt.tag_data_p[1] = pkt.tag_data[33];
					pkt.tag_data_p[2] = pkt.tag_data[34];
					pkt.tag_data_p[3] = pkt.tag_data[35];

					// Transmit data packet to PC
					if(HAL_UART_Transmit(&UartHandle, &pkt, 44, 5000)!= HAL_OK) {
						Error_Handler();
					}
				}

				// Update last read UID
				copyUID(lastReadUID, g_pui8Iso15693UId);
			}
		}
		// Turn off RF field once done reading the tag(s)
		TRF79xxA_turnRfOff();
	}

	// Set the SPI SS High
	HAL_GPIO_WritePin(SPIx_SS_GPIO_PORT, SPIx_SS_PIN,  GPIO_PIN_SET);
	// Set TRF Enable Pin Low
	HAL_Delay(2);
	HAL_GPIO_WritePin(TRF79XXA_EN_GPIO_PORT, TRF79XXA_EN_PIN, GPIO_PIN_RESET);

} // main


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



/********************************************************/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
    Error_Handler();
}


uint8_t compareUID(uint8_t* id_a, uint8_t* id_b) {
	int i = 0;
	for (; i < 8; ++i) {
		if (id_a[i] != id_b[i]) {
			return ID_DIFF;
		}
	}
	return ID_SAME;
}

void copyUID(uint8_t* target, uint8_t* source) {
	int i = 0;
	for (; i < 8; ++i) {
		target[i] = source[i];
	}
}


