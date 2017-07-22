/*
 * uart.c
 *
 *  Created on: 2017年7月6日
 *      Author: seanfoo
 */
#include "uart.h"


void UART1_Init(void)
{
	UartHandle.Instance        = USARTx;
	UartHandle.Init.BaudRate     = 115200;
	UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
	UartHandle.Init.StopBits     = UART_STOPBITS_1;
	UartHandle.Init.Parity       = UART_PARITY_NONE;
	UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE; //- Hardware flow control disabled (RTS and CTS signals)
	UartHandle.Init.Mode         = UART_MODE_TX_RX;

	if(HAL_UART_Init(&UartHandle) != HAL_OK) {
		Error_Handler(); // Initialization Error
	}
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  // Enable peripherals and GPIO Clocks
  // Enable GPIO TX/RX clock
  USARTx_TX_GPIO_CLK_ENABLE();
  USARTx_RX_GPIO_CLK_ENABLE();

  // Enable USARTx clock
  USARTx_CLK_ENABLE();

  // UART TX GPIO pin configuration
  GPIO_InitStruct.Pin       = USARTx_TX_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;

  HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStruct);

  // UART RX GPIO pin configuration
  GPIO_InitStruct.Pin = USARTx_RX_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStruct);

  // Configure the NVIC for UART
  HAL_NVIC_SetPriority(USARTx_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(USARTx_IRQn);
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
  // Reset peripherals
  USARTx_FORCE_RESET();
  USARTx_RELEASE_RESET();

  // Configure USART1 Tx as alternate function
  HAL_GPIO_DeInit(USARTx_TX_GPIO_PORT, USARTx_TX_PIN);
  // Configure USART1 Rx as alternate function
  HAL_GPIO_DeInit(USARTx_RX_GPIO_PORT, USARTx_RX_PIN);

  HAL_NVIC_DisableIRQ(USARTx_IRQn);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
    Error_Handler();
}
