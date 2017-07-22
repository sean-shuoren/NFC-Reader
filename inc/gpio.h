/*
 * gpio.h
 *
 *  Created on: 2017年7月20日
 *      Author: seanfoo
 */

#ifndef GPIO_H_
#define GPIO_H_

#include "stm32f1xx_nucleo.h"

#define TRF79XXA_IRQ_PIN			GPIO_PIN_10
#define TRF79XXA_IRQ_GPIO_PORT 		GPIOB

#define TRF79XXA_EN_PIN             GPIO_PIN_11
#define TRF79XXA_EN_GPIO_PORT       GPIOB

#define MOTOR_GROUND_PIN			GPIO_PIN_7
#define MOTOR_GROUND_GPIO_PORT      GPIOA
#define MOTOR_CONTROL_PIN			GPIO_PIN_6
#define MOTOR_CONTROL_GPIO_PORT     GPIOA



void GPIO_Config_EN(void);
void GPIO_Config_IRQ(void);

void GPIO_Config_Motor_Ground(void);
void GPIO_Config_Motor_Control(void);


#endif /* GPIO_H_ */
