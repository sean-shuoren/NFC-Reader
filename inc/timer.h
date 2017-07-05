#ifndef TIMER_H_
#define TIMER_H_

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32f1xx_nucleo.h"

/* Set Timer instance */

/* Definition for TIMx clock resources */
#define TIMx                        TIM3
#define TIMx_CLK_ENABLE()           __HAL_RCC_TIM3_CLK_ENABLE()
#define DELAY_TIM					TIM2
#define DELAY_TIM_CLK_ENABLE()      __HAL_RCC_TIM2_CLK_ENABLE()

/* Definition for TIMx's NVIC */
#define TIMx_IRQn                   TIM3_IRQn
#define TIMx_IRQHandler             TIM3_IRQHandler
#define DELAY_TIM_IRQn              TIM2_IRQn
#define DELAY_TIM_IRQHandler        TIM2_IRQHandler

/* TIM handle declaration */
TIM_HandleTypeDef    TimHandle;
TIM_HandleTypeDef    Delay_TimHandle;

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
void delayMilliSeconds(uint32_t);
void waitMilliSeconds(uint32_t);

extern void Error_Handler(void);

#endif
