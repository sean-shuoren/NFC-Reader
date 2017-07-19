/*
 * gpio.h
 *
 *  Created on: 2017年7月20日
 *      Author: seanfoo
 */

#ifndef GPIO_H_
#define GPIO_H_

#include "stm32f1xx_nucleo.h"

#define TRF79XXA_EN_PIN             GPIO_PIN_11	// TRF Reader board EN
#define TRF79XXA_EN_GPIO_PORT       GPIOB

#define TRF79XXA_IRQ_PIN			GPIO_PIN_10
#define TRF79XXA_IRQ_GPIO_PORT 		GPIOB


GPIO_InitTypeDef  GPIO_InitStruct;

void GPIO_Config_EN(void);
void GPIO_Config_IRQ(void);


#endif /* GPIO_H_ */
