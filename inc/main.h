#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f1xx_nucleo.h"
#include "stm32f1xx.h"
#include "spi.h"
#include "timer.h"
#include "gpio.h"
#include "trf7970a.h"


/* Private macro -------------------------------------------------------------*/
#define MASTER_BOARD
//#define ANTI_COLLISION

enum {
	ID_SAME,
	ID_DIFF
};
uint8_t lastReadUID [8];
uint8_t compareUID(uint8_t *, uint8_t *);
void copyUID(uint8_t* target, uint8_t* source);

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);


/* Data Transfered to GUI */
struct dataPacket {
	uint8_t uid [8];
	uint8_t tag_data_p [4];
	uint8_t tag_data [36];
};


#endif
