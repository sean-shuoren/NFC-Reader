#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f1xx_nucleo.h"
#include "stm32f1xx.h"
#include "spi.h"
#include "timer.h"
#include "trf7970a.h"


/* Private macro -------------------------------------------------------------*/
#define MASTER_BOARD


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

#endif
