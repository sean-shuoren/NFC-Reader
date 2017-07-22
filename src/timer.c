#include <timer.h>


/************ MspInit Functions *************/

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
	/* TIM Peripheral clock enable
	 * Set the TIM priority
	 * Enable the TIM global Interrupt */

	if (htim->Instance == WAIT_TIM) {
		WAIT_TIM_CLK_ENABLE();
		HAL_NVIC_SetPriority(TIMx_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(TIMx_IRQn);
	}

	if (htim->Instance == DELAY_TIM) {
		DELAY_TIM_CLK_ENABLE();
		HAL_NVIC_SetPriority(DELAY_TIM_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(DELAY_TIM_IRQn);
	}
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm)
{
	if(htim_pwm->Instance == TIM4) {
		PWM_TIM_CLK_ENABLE();
	}
}


/************ Configuration Functions *************/

void Timer_Config_Wait(void)
{
	uint32_t uwPrescalerValue = (uint32_t)(SystemCoreClock / 1000) - 1;

	TimHandle.Instance 				 = WAIT_TIM;
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


/*!< Specifies the pulse value to be loaded into the Capture Compare Register.
   This parameter can be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF */
void Timer_Config_PWM(uint32_t pulse)
{
	__HAL_RCC_GPIOB_CLK_ENABLE();

	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;

	// SystemCoreClock = 72000000;
	PWM_TimHandle.Instance 				= PWM_TIM;
	PWM_TimHandle.Init.Prescaler 		= (uint32_t)(SystemCoreClock / 100000) - 1;
	PWM_TimHandle.Init.CounterMode 		= TIM_COUNTERMODE_UP;
	PWM_TimHandle.Init.Period 			= 104 - 1;					// 960 Hz
	PWM_TimHandle.Init.ClockDivision 	= TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_PWM_Init(&PWM_TimHandle) != HAL_OK) {
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger 	= TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode 		= TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&PWM_TimHandle, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}

	sConfigOC.OCMode 					= TIM_OCMODE_PWM1;
	sConfigOC.Pulse 					= 104 * pulse / 100 ;
	sConfigOC.OCPolarity 				= TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode 				= TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&PWM_TimHandle, &sConfigOC, PWM_CHANNEL) != HAL_OK) {
		Error_Handler();
	}

	/* PWM Output GPIO pin
	 * TIM4 GPIO Configuration: PB6 ------> TIM4_CH1 */

	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin 	= GPIO_PIN_6;
	GPIO_InitStruct.Mode 	= GPIO_MODE_AF_PP;
	GPIO_InitStruct.Speed 	= GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/************ Start Timer and Wait for Interrupt Functions *************/

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
