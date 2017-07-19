#include "gpio.h"

void GPIO_Config_EN(void)
{
	/* TRF Reader board EN GPIO pin configuration  */
	GPIO_InitStruct.Pin      = TRF79XXA_EN_PIN;
	GPIO_InitStruct.Mode     = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull     = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed    = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(TRF79XXA_EN_GPIO_PORT, &GPIO_InitStruct);
}

void GPIO_Config_IRQ(void)
{
	/* TRF Reader board IRQ GPIO pin configuration  */
	GPIO_InitStruct.Mode 	 = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull 	 = GPIO_NOPULL;
	GPIO_InitStruct.Speed	 = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Pin  	 = TRF79XXA_IRQ_PIN;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* GPIO Init */
	HAL_GPIO_Init(TRF79XXA_IRQ_GPIO_PORT, &GPIO_InitStruct);
}


