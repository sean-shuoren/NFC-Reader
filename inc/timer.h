#ifndef TIMER_H_
#define TIMER_H_

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32f1xx_nucleo.h"

/* Compute the prescaler value to have TIMx counter clock equal to 10000 Hz
 * Initialize TIMx peripheral as follows:
 *	   + Period = 10000 - 1
 *	   + Prescaler = (SystemCoreClock/10000) - 1
 *	   + ClockDivision = 0
 *	   + Counter direction = Up
 */

/* Definition for TIMx clock resources */
#define WAIT_TIM                    TIM3
#define WAIT_TIM_CLK_ENABLE()           __HAL_RCC_TIM3_CLK_ENABLE()

#define DELAY_TIM					TIM2
#define DELAY_TIM_CLK_ENABLE()      __HAL_RCC_TIM2_CLK_ENABLE()

#define PWM_TIM						TIM4
#define PWM_TIM_CLK_ENABLE()		__HAL_RCC_TIM4_CLK_ENABLE()
#define PWM_CHANNEL					TIM_CHANNEL_1

/* Definition for TIMx's NVIC */
#define TIMx_IRQn                   TIM3_IRQn
#define TIMx_IRQHandler             TIM3_IRQHandler
#define DELAY_TIM_IRQn              TIM2_IRQn
#define DELAY_TIM_IRQHandler        TIM2_IRQHandler

/* TIM handle declaration */
TIM_HandleTypeDef    TimHandle;
TIM_HandleTypeDef    Delay_TimHandle;
TIM_HandleTypeDef    PWM_TimHandle;

/* Flag for Delay */
typedef enum
{
	TIM_OVER,
	TIM_INIT,
	TIM_WAIT
} TimStatus;

volatile TimStatus delayStatus;
volatile TimStatus waitStatus;

/************ Utility function ***********/
void Timer_Config_Delay(void);
void Timer_Config_Wait(void);
void Timer_Config_PWM(uint32_t pulse);

void delayMilliSeconds(uint32_t);
void waitMilliSeconds(uint32_t);

extern void Error_Handler(void);

#endif
