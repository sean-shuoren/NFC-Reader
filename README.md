# NFC Reader
Using STM32-F103RB to control TRF7970A-BP NFC Reader to read and write M24LR64E-R Dynamic NFC Tag.

## Spec Sheets
- [STM32-F103RB HAL library](http://www.st.com/content/ccc/resource/technical/document/user_manual/72/52/cc/53/05/e3/4c/98/DM00154093.pdf/files/DM00154093.pdf/jcr:content/translations/en.DM00154093.pdf)
- [TRF7970A NFC Reader Chip](http://www.ti.com/lit/ds/symlink/trf7970a.pdf) ([BoosterPack](http://www.dlpdesign.com/dlp-7970abp-ds-v11.pdf))
- [M24LR64E-R Dynamic NFC Tag](http://www.st.com/content/ccc/resource/technical/document/datasheet/45/6b/a2/02/63/ad/45/43/DM00047008.pdf/files/DM00047008.pdf/jcr:content/translations/en.DM00047008.pdf)

Please refer to [here](http://www.openstm32.org/Creating+a+new+project) for setting up a debug environment with static STM32 HAL library using Eclipse.

## Overview

### iso15693.h/.c
The methods to interact with TRF7970A NFC reader board according to ISO15693 NFC standard is defined.

`ISO15693_init` should be called initially to initialize data structures. When we need to read a NFC tag, `ISO15693_sendSingleSlotInventory` should be called to detect whether a NFC tag is in the range. If there is only one tag in the range, `g_ui8TagDetectedCount` will be set to one, and `STATUS_SUCCESS` will be returned. Otherwise, even with multiple tags detected, `STATUS_FAIL` will be returned. After the inventory, the 8 byte NFC tag's unique ID will be stored in `g_pui8Iso15693UId`.

Originally if multiple tags are detected, we should run through a anti-collision process, but since we won't encounter this situation (tags are transmitted one after another on the conveyer belt), we neglect this process.

To read the content of the tag, `ISO15693_ReadExtendedTag` should be called. In this function, `ISO15693_sendGetSystemInfoExtended` is called first to get the total number of blocks in the NFC tag. Then, `ISO15693_sendReadSingleBlockExtended` is called multiple times to read the NFC tag blocks. Since there is total of 2047 blocks in M24LR64E-R NFC tag, reading all the blocks is time-consuming and unnecessary here, you can specify the blocks that should be read according to your designed protocol.

To write a block of the tag, `ISO15693_sendWriteSingleBlockExtended` should be called. The block we are writing to is specified by `ui16BlockNumber` and the data be written into the block is specified using `pui8BlockData`, a uint8_t array of size 4.

There are other functionalities that can be used to interact with M24LR64E-R NFC tag, especially some security functionalities have not been implemented. Please refer to the data sheet of the NFC tag for those functionalities.

### trf7970a.h/.c
In the header file, registers and commands of the TRF7970A reader board are defined here.
`g_pui8TrfBuffer` buffer is used to hold the data that is transmitted back and forth between STM32 and NFC reader.

`TRF79xxA_writeRaw`, `TRF79xxA_sendDirectCommand`, `TRF79xxA_readRegister`,  `TRF79xxA_writeRegister`, `TRF79xxA_readContinuous`, `TRF79xxA_writeContinuous` are basic functions to read and write the registers and FIFO in TRF7970A reader chip.

When commands are send to the reader board and program is waiting for the results, `TRF79xxA_waitRxData` is called to let the program waiting for certain time for the transmit and receive process to complete. If either of the process is complete, the reader chip will send a interrupt through IRQ pin (section gpio.h/.c). `TRF79xxA_readIrqStatus` and `TRF79xxA_processIRQ` will be called in the subsequent routine to perform certain actions, such as read from the TRF7970A reader board.

### spi.h/.c
STM32 communicates with the NFC reader board using SPI protocol. Pin MISO, MOSI, SCK, SSL of the SPI protocol are defined here. By default, the `SPI2` module on STM32 maps those pins to PIN 14, 15, 13, 12 on port B on the STM32 board, respectively.

`SPI_Config` is a function definition that should be called in the main function to initiate the SPI module on STM32.

SPI communication can be started by calling `HAL_SPI_Transmit` and `HAL_SPI_Receive` functions in the HAL library

### gpio.h/.c
The EN signal which should be set high to enable the communication to NFC reader board. Calling `GPIO_Config_EN` function initiates this pin. The mapping of this pin on STM32 can also be changed by redefining the macros in the header file.

The IRQ GPIO handles the interrupt request that send from the NFC reader. Calling `GPIO_Config_IRQ` function initiates this pin. The mapping of this pin on STM32 can also be changed by redefining the macros in the header file.

`MOTOR_CONTROL_PIN` and `MOTOR_GROUND_PIN` are also defined here. We need three signals to control the motor that drives the conveyer belt. One ground pin that should always be set low. A control pin that when set high, enables the motor to rotate. Another PWM signal that controls the speed of the motor is defined in `timer.h/.c`. Calling `GPIO_Config_Motor_Ground` and `GPIO_Config_Motor_Control` function initiates this pin. The mapping of this pin on STM32 can also be changed by redefining the macros in the header file.

### timer.h/.c
There are three timers defined here. `DELAY_TIM` is solely for delay purpose. Since the `HAL_Delay` function in the library follows a busy loop pattern that tends to mess up when entangled with interrupts, a separate delay timer is necessary. Calling `Timer_Config_Delay` function initiates this timer. Calling `delayMilliSeconds` causes the program to pause for certain milliseconds.

`WAIT_TIM` is for the purpose of handling the communication with the NFC reader board. The board needs some time to process the command and provide the data we need before sending a interrupt using the IRQ GPIO pin. Calling `Timer_Config_Wait` function initiates this timer. Calling `waitMilliSeconds` causes the program to wait for certain milliseconds for the response of the Reader board.

`PWM_TIM` is for the purpose of the PWM clock that used to control the speed of the motor. The period of the PWM is preset to 960 Hz. And the duty cycle can be controlled by `pulse` parameter in `Timer_Config_PWM` which should be a value between 0 and 100 to define the duty cycle. Calling `HAL_TIM_PWM_Start` in the main function starts the PWM.

### uart.h/.c
STM32 transmit the data that reads from the tag to the host PC using a UART to USB device. Pin TX, RX of the UART protocol are defined here. By default, the `USART1` module on STM32 maps those pins to PIN 9, 10 on port A on the STM32 board, respectively.

`UART1_Init` should be called in the main function to initiate the UART communication module on STM32. Calling `HAL_UART_Transmit` makes the STM32 to transmit data on TX and RX pins.
