#ifndef SPI_H_
#define SPI_H_

#include "stm32f1xx_nucleo.h"
#include <stdbool.h>

/* BIT define for convenience */
#define BIT0	0x01
#define BIT1	0x02
#define BIT2	0x04
#define BIT3	0x08
#define BIT4	0x10
#define BIT5	0x20
#define BIT6	0x40
#define BIT7	0x80


/* SPI transfer state*/
enum {
	TRANSFER_WAIT,
	TRANSFER_COMPLETE,
	TRANSFER_ERROR
};

enum {
	STATUS_SUCCESS,
	STATUS_FAIL
};

/* SPI handler declaration */
SPI_HandleTypeDef SpiHandle;

/* transfer state */
static volatile uint32_t wTransferState = TRANSFER_WAIT;

/* Definition for SPIx clock resources */
#define SPIx                             SPI2
#define SPIx_CLK_ENABLE()                __HAL_RCC_SPI2_CLK_ENABLE()
#define DMAx_CLK_ENABLE()                __HAL_RCC_DMA1_CLK_ENABLE()
#define SPIx_SCK_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPIx_MISO_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPIx_MOSI_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()

/* Definition for SPIx Pins */
//#define TRF_EN_PIN                       GPIO_PIN_11	// TRF Reader board EN
//#define TRF_EN_GPIO_PORT                 GPIOB

#define SPIx_SCK_PIN                     GPIO_PIN_13
#define SPIx_SCK_GPIO_PORT               GPIOB
#define SPIx_MISO_PIN                    GPIO_PIN_14
#define SPIx_MISO_GPIO_PORT              GPIOB
#define SPIx_MOSI_PIN                    GPIO_PIN_15
#define SPIx_MOSI_GPIO_PORT              GPIOB
#define SPIx_SS_PIN                      GPIO_PIN_12	// SPI Select
#define SPIx_SS_GPIO_PORT                GPIOB

/* Definition for SPIx's NVIC */
#define SPIx_IRQn                        SPI2_IRQn
#define SPIx_IRQHandler                  SPI2_IRQHandler


/* SPI functions */
void SPI_Config(void);


#endif /* SPI_H_ */
