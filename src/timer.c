#include <timer.h>

/**
  * @brief TIM MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  * @param htim: TIM handle pointer
  * @retval None
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
	/* TIMx Peripheral clock enable
	 * Set the TIMx priority
	 * Enable the TIMx global Interrupt */

	if (htim->Instance == TIMx) {
		TIMx_CLK_ENABLE();
		HAL_NVIC_SetPriority(TIMx_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(TIMx_IRQn);
	}

	if (htim->Instance == DELAY_TIM) {
		DELAY_TIM_CLK_ENABLE();
		HAL_NVIC_SetPriority(DELAY_TIM_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(DELAY_TIM_IRQn);
	}

}

void Timer_Config_Wait(void)
{
	uint32_t uwPrescalerValue = (uint32_t)(SystemCoreClock / 1000) - 1;

	TimHandle.Instance 				 = TIMx;
	TimHandle.Init.Period            = 1000 - 1;
	TimHandle.Init.Prescaler         = uwPrescalerValue;
	TimHandle.Init.ClockDivision     = 0;
	TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
	TimHandle.Init.RepetitionCounter = 0;

	if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
		Error_Handler();
}

void Timer_Config_Delay(void)
{
	uint32_t uwPrescalerValue = (uint32_t)(SystemCoreClock / 1000) - 1;

	Delay_TimHandle.Instance 			   = DELAY_TIM;
	Delay_TimHandle.Init.Period            = 1000 - 1;
	Delay_TimHandle.Init.Prescaler         = uwPrescalerValue;
	Delay_TimHandle.Init.ClockDivision     = 0;
	Delay_TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
	Delay_TimHandle.Init.RepetitionCounter = 0;

	if (HAL_TIM_Base_Init(&Delay_TimHandle) != HAL_OK)
		Error_Handler();
}

/**
  * @brief Use DELAY_TIM to delay @period milliseconds
  * @param period: delay time duration
  * @retval None
  */
void waitMilliSeconds(uint32_t period)
{
	// Initialize timer with given period
	TimHandle.Init.Period = period - 1;
	if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
		Error_Handler();

	// TODO
	// For some reason the first time we start the timer
	// the period is not correct (always 1uS no matter @period)
	waitStatus = TIM_INIT;
	if (HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK)
		Error_Handler();
	while(waitStatus == TIM_INIT);

	// Start the timer again and delay for @period duration
	waitStatus = TIM_WAIT;
	if (HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK)
		Error_Handler();
	while(waitStatus == TIM_WAIT);

}

/**
  * @brief Use DELAY_TIM to delay @period milliseconds
  * @param period: delay time duration
  * @retval None
  */
void delayMilliSeconds(uint32_t period)
{
	// Initialize timer with given period
	Delay_TimHandle.Init.Period = period - 1;
	if (HAL_TIM_Base_Init(&Delay_TimHandle) != HAL_OK)
		Error_Handler();

	// TODO
	// For some reason the first time we start the timer
	// the period is not correct (always 1uS no matter @period)
	delayStatus = TIM_INIT;
	if (HAL_TIM_Base_Start_IT(&Delay_TimHandle) != HAL_OK)
		Error_Handler();
	while(delayStatus == TIM_INIT);

	// Start the timer again and delay for @period duration
	delayStatus = TIM_WAIT;
	if (HAL_TIM_Base_Start_IT(&Delay_TimHandle) != HAL_OK)
		Error_Handler();
	while(delayStatus == TIM_WAIT);

}
