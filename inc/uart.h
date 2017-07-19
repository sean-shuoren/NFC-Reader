/*
 * uart.h
 *
 *  Created on: 2017年7月6日
 *      Author: seanfoo
 */

#ifndef UART_H_
#define UART_H_

#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_nucleo.h"

UART_HandleTypeDef UartHandle;

// Definition for USARTx clock resources
#define USARTx                           USART1
#define USARTx_CLK_ENABLE()              __HAL_RCC_USART1_CLK_ENABLE();
#define USARTx_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

#define USARTx_FORCE_RESET()             __HAL_RCC_USART1_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __HAL_RCC_USART1_RELEASE_RESET()

// Definition for USARTx Pins
#define USARTx_TX_PIN                    GPIO_PIN_9
#define USARTx_TX_GPIO_PORT              GPIOA
#define USARTx_RX_PIN                    GPIO_PIN_10
#define USARTx_RX_GPIO_PORT              GPIOA

// Definition for USARTx's NVIC
#define USARTx_IRQn                      USART1_IRQn
#define USARTx_IRQHandler                USART1_IRQHandler


void UART1_Init(void);
void HAL_UART_MspInit(UART_HandleTypeDef *huart);
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart);


#endif /* UART_H_ */
