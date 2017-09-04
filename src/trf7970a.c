#include "trf7970a.h"
#include "timer.h"
#include "gpio.h"


//===============================================================
//
//! TRF79xxA_setupInitiator - Write the initial settings for
//! a set of TRF79xxA registers based on which protocol is to be
//! enabled.
//!
//! \param ui8IsoControl is the value to write to the ISO Control
//! register of the TRF79xxA
//!
//! This function is used to configure a series of TRF79xxA
//! registers based on which RFID technology will be enabled in
//! the ISO control register.
//!
//! This function will only enables one RFID technology at a time.
//!
//! \return None.
//
//===============================================================
void TRF79xxA_setupInitiator(uint8_t ui8IsoControl)
{
	/* We are using ISO 15693 here */
	if (ui8IsoControl == 0x02) {
		TRF79xxA_reset();			// Reset the TRF7970A to ensure a clean state of operation before changing modes

		TRF79xxA_turnRfOn();		// Turn on the RF field

		g_eTrfGeneralSettings.ui8IsoControl = ui8IsoControl;	// Update the ISO Control Register variable

		// Register 0x01 - ISO Control Register
		TRF79xxA_writeRegister(TRF79XXA_ISO_CONTROL,ui8IsoControl);

		// Register 0x09 - System Clock Output, Modulation Scheme
		TRF79xxA_writeRegister(TRF79XXA_MODULATOR_CONTROL, 0x01); 		// Sys Clock Output = 13.56MHz, OOK = 100%

		// Resgister 0x07 - No Response Wait Time
		TRF79xxA_writeRegister(TRF79XXA_RX_NO_RESPONSE_WAIT_TIME, 0x15);

		// Register 0x14 - Adjustable FIFO Level
		TRF79xxA_writeRegister(TRF79XXA_FIFO_IRQ_LEVELS, 0x0C);

		// Register 0x18 - NFC Target Detection Level
		// This register must be written to 0x00 per TRF7970A Errata
		TRF79xxA_writeRegister(TRF79XXA_NFC_TARGET_LEVEL, 0x00);
	}

}

//===============================================================
//
//! TRF79xxA_initialSettings - Initialize TRF79xxA
//!
//! This function configures the TRF79xxA upon power up.
//! Steps include:
//!  - Setup SPI communication
//!  - Send Soft Init and Idle Direct Commands
//!  - Reset the FIFO
//!  - Configure TRF79xxA modulator and regulator registers
//!  - TRF7970A only: Write to regiser 0x18 per errata
//!
//! \return None.
//
//===============================================================
void TRF79xxA_initialSettings(void)
{
	g_ui8CollisionPosition = 0;
	g_ui8FifoOffset = 0;
	g_ui8FifoRxLength = 0;

	g_sTrfStatus = TRF_IDLE;
	g_eTrfGeneralSettings.eTrfPowerSetting = TRF79xxA_3V_FULL_POWER;
	g_eTrfGeneralSettings.bRfFieldOn = false;
	g_eTrfGeneralSettings.ui8IsoControl = 0x02;	// TRF79xxA Default

	g_ui8IrqFlag = 0x00;
	g_ui8TimeoutFlag = 0x00;

	delayMilliSeconds(2);	/* Delay to allow SPI to finish starting up */

	// Send out SOFT_INIT + IDLE initial sequence to soft reset TRF7970A
	TRF79xxA_sendDirectCommand(TRF79XXA_SOFT_INIT_CMD);
	TRF79xxA_sendDirectCommand(TRF79XXA_IDLE_CMD);

	delayMilliSeconds(2);	/* Delay to ensure soft reset has processed */

	g_eTrfGeneralSettings.ui8IsoControl = 0x21;

	TRF79xxA_resetFIFO();			// Reset the FIFO

	TRF79xxA_writeRegister(TRF79XXA_MODULATOR_CONTROL, 0x01); 		// ASK 100%, no SYS_CLK output
	TRF79xxA_writeRegister(TRF79XXA_REGULATOR_CONTROL, 0x07);
	TRF79xxA_writeRegister(TRF79XXA_NFC_TARGET_LEVEL, 0x00); 		// For TRF7970A Errata

	// TRF79XXA_REGULATOR_CONTROL

}

//===============================================================
//
//! TRF79xxA_checkExternalRfField - Checks if an external RF field
//! is present.
//!
//! This function performs RF collision avoidance as outlined in
//! TI application notes (sloa192, sloa227) in order to determine
//! if an external RF field is present.
//!
//! \return bExtFieldOn retuns the external RF field status
//
//===============================================================
bool TRF79xxA_checkExternalRfField(void)
{
	bool bExtFieldOn;
	uint8_t ui8Value;

	TRF79xxA_writeRegister(TRF79XXA_CHIP_STATUS_CONTROL, 0x02);
	TRF79xxA_sendDirectCommand(TRF79XXA_TEST_EXTERNAL_RF_CMD);
	delayMilliSeconds(2); 		//	Delay for 50uS

	ui8Value = TRF79xxA_readRegister(TRF79XXA_RSSI_LEVELS);

	// Check for RF field bit and update variable
	if ((ui8Value & 0x07) > 1) {
		bExtFieldOn = true;
	}
	else {
		bExtFieldOn = false;
	}

	return bExtFieldOn;
}



//===============================================================
//
//! TRF79xxA_turnRfOff - Turn off the transmission of the TRF79xxA
//! RF Field
//!
//! This function stops the TRF79xxA transmitting an RF field
//!
//! \return None.
//
//===============================================================
void TRF79xxA_turnRfOff(void)
{
	uint8_t	ui8Value;

	ui8Value = g_eTrfGeneralSettings.eTrfPowerSetting;

	TRF79xxA_writeRegister(TRF79XXA_CHIP_STATUS_CONTROL,ui8Value);

	g_eTrfGeneralSettings.bRfFieldOn = false;	// Update RF Field variable
}

//===============================================================
//
//! TRF79xxA_turnRfOn - Turns on the transmission of the TRF79xxA
//! RF Field
//!
//! This function enables the TRF79xxA transmit an RF field
//!
//! \return None.
//
//===============================================================
void TRF79xxA_turnRfOn(void)
{
	uint8_t	ui8Value;

	ui8Value = g_eTrfGeneralSettings.eTrfPowerSetting | 0x20;	// Turn RF field On

	TRF79xxA_writeRegister(TRF79XXA_CHIP_STATUS_CONTROL,ui8Value);

	g_eTrfGeneralSettings.bRfFieldOn = true;	// Update RF Field variable
}

//===============================================================
//
//! TRF79xxA_disableSlotCounter - Disables interrupts from 15693
//! slot counter function.
//!
//! This function will configure the TRF79xxA to disable the
//! firing of the IRQ interrupt when an ISO15693 slot marker
//! hits the No Response timeout threshold.
//!
//! \return None.
//
//===============================================================
void TRF79xxA_disableSlotCounter(void)
{
	uint8_t ui8Value;

	ui8Value = TRF79xxA_readRegister(TRF79XXA_IRQ_MASK);
	ui8Value &= 0xFE;		// Clear BIT0 in register 0x0D
	TRF79xxA_writeRegister(TRF79XXA_IRQ_MASK,ui8Value);
}

//===============================================================
//
//! TRF79xxA_enableSlotCounter - Enables interrupts from 15693
//! slot counter function.
//!
//! This function will configure the TRF79xxA to enable the
//! firing of the IRQ interrupt when an ISO15693 slot marker
//! hits the No Response timeout threshold.
//!
//! \return None.
//
//===============================================================
void TRF79xxA_enableSlotCounter(void)
{
	uint8_t ui8Value;

	ui8Value = TRF79xxA_readRegister(TRF79XXA_IRQ_MASK);
	ui8Value |= BIT0;		// Set BIT0 in register 0x0D
	TRF79xxA_writeRegister(TRF79XXA_IRQ_MASK,ui8Value);
}

//===============================================================
//
//! TRF79xxA_getTrfStatus - Returns current TRF79xxA driver
//! status
//!
//! \return g_sTrfStatus returns the current TRF79xxA drive
//! status
//
//===============================================================
tTrfStatus TRF79xxA_getTrfStatus(void)
{
	return g_sTrfStatus;
}

//===============================================================
//
//! TRF79xxA_setTrfStatus - Set the TRF79xxA driver status
//!
//! \param sTrfStatus is the new TRF79xxA driver status
//!
//! This function sets the TRF79xxA driver status manually.
//! This can be used to modify the TRF driver status without an
//! IRQ event. Use with caution.
//!
//! \return None.
//
//===============================================================
void TRF79xxA_setTrfStatus(tTrfStatus sTrfStatus)
{
	g_sTrfStatus = sTrfStatus;
}

//===============================================================
//
//! TRF79xxA_setTrfPowerSetting - Set the TRF79xxA power setting
//!
//! \param sTrfPowerSetting is the new TRF79xxA Power Setting
//!
//! This function allows for configuration of the TRF79xxA power
//! setting.
//!
//! Options are:
//!  - TRF79xxA_3V_FULL_POWER: 3V TRF79xxA input w/ full power RF output
//!  - TRF79xxA_5V_FULL_POWER: 5V TRF79xxA input w/ full power RF output
//!  - TRF79xxA_3V_HALF_POWER: 3V TRF79xxA input w/ half power RF output
//!  - TRF79xxA_5V_HALF_POWER: 5V TRF79xxA input w/ half power RF output
//!
//! \return None.
//
//===============================================================
void TRF79xxA_setTrfPowerSetting(tTrfPowerOptions sTrfPowerSetting)
{
	g_eTrfGeneralSettings.eTrfPowerSetting = sTrfPowerSetting;
}

//===============================================================
//
//! TRF79xxA_getCollisionPosition - Return the current Collision
//! Position value
//!
//! This function will return the current Collision Position
//! value.
//! This is only used for the ISO14443 Type A anti-collision
//! process.
//!
//! \return g_ui8CollisionPosition returns the current Collision
//! Position value
//
//===============================================================
uint8_t TRF79xxA_getCollisionPosition(void)
{
	return g_ui8CollisionPosition;
}

//===============================================================
//
//! TRF79xxA_setCollisionPosition - Set the Collision Position
//! variable
//!
//! \param ui8ColPos is the new Collision Position value
//!
//! This function sets the Collision Position variable.
//! This is only used for the ISO14443 Type A anti-collision
//! process.
//!
//! \return None.
//
//===============================================================
void TRF79xxA_setCollisionPosition(uint8_t ui8ColPos)
{
	g_ui8CollisionPosition = ui8ColPos;
}

//===============================================================
//
//! TRF79xxA_getRxBytesReceived - Returns the Number of RX Bytes
//! received by the TRF79xxA FIFO
//!
//! This function returns the number of bytes received by the
//! TRF79xxA during the last packet reception.
//!
//! This is used to check for how much data has been received
//! and to ensure packets with expected lengths were fully
//! received.
//!
//! \return g_ui8FifoRxLength returns the current FIFO RX Length
//
//===============================================================
uint8_t TRF79xxA_getRxBytesReceived(void)
{
	return g_ui8FifoRxLength;
}

//===============================================================
//
//! TRF79xxA_getTrfBuffer - Returns a point to the start of the
//! TRF Buffer.
//!
//! This function will return a pointer for the start address of
//! the TRF79xxA Data Buffer.
//!
//! This is used to access the data received from successful RF
//! commands in files which do not have naturally have access to
//! the g_pui8TrfBuffer.
//!
//! \return &g_pui8TrfBuffer[0] returns the start address of
//! g_pui8TrfBuffer.
//
//===============================================================
uint8_t * TRF79xxA_getTrfBuffer(void)
{
	return &g_pui8TrfBuffer[0];
}

//===============================================================
//
//! TRF79xxA_getIsoControlValue - Fetch the latest Iso Control
//! Register value
//!
//! This function returns the current TRF79xxA ISO Control
//! Register setting .
//!
//! The ISO Control Register value is updated whenever a read or
//! write to the ISO Control Register occurs.
//!
//! \return g_ui8IsoControlValue returns the current ISO Control
//! Register value
//
//===============================================================
uint8_t TRF79xxA_getIsoControlValue(void)
{
	return g_eTrfGeneralSettings.ui8IsoControl;
}



//===============================================================
//
//! TRF79xxA_reset - Resets TRF79xxA
//!
//! This function will reset the TRF79xxA through the Software
//! Init direct command followed by reinitializing basic settings
//! and clearing affected global variables.
//!
//! \return None.
//
//===============================================================
void TRF79xxA_reset(void)
{
	TRF79xxA_sendDirectCommand(TRF79XXA_SOFT_INIT_CMD);
	TRF79xxA_sendDirectCommand(TRF79XXA_IDLE_CMD);

//	delayMilliSeconds(2);

	TRF79xxA_resetFIFO();			// Reset the FIFO

	TRF79xxA_writeRegister(TRF79XXA_MODULATOR_CONTROL, 0x01); 		// ASK 100%, no SYS_CLK output
	TRF79xxA_writeRegister(TRF79XXA_REGULATOR_CONTROL, 0x01);
	TRF79xxA_writeRegister(TRF79XXA_NFC_TARGET_LEVEL, 0x00); 		// For TRF7970A Errata

	g_eTrfGeneralSettings.ui8IsoControl = 0x21;
	g_eTrfGeneralSettings.bRfFieldOn = false;
	g_ui8FifoOffset = 0;
	g_ui8FifoRxLength = 0;
}

//===============================================================
//
//! TRF79xxA_resetFIFO - Resets TRF79xxA FIFO
//!
//! This function used to reset the TRF79xxA FIFO.
//!
//! \return None.
//
//===============================================================
void TRF79xxA_resetFIFO(void) {
	uint8_t	ui8Command = TRF79XXA_RESET_FIFO_CMD;
	TRF79xxA_sendDirectCommand(ui8Command);
}



//===============================================================
//
//! TRF79xxA_resetIrqStatus - Resets the IRQ Status Register of
//! the TRF79xxA
//!
//! This function resets/clears the TRF79xxA IRQ Status Register
//!
//! \return None.
//
//===============================================================
void TRF79xxA_resetIrqStatus(void)
{
	uint8_t puiIrqStatus[2];

	puiIrqStatus[0] = TRF79XXA_IRQ_STATUS;
	puiIrqStatus[1] = TRF79XXA_IRQ_MASK;

	TRF79xxA_readContinuous(puiIrqStatus, 2);		// read second reg. as dummy read
}

//===============================================================
//
//! TRF79xxA_readIrqStatus - Read out the IRQ Status Register
//!
//! This function reads the IRQ Status register of the TRF79xxA
//! and store the result into the location pointed to by the
//! input.
//!
//! \return pui8Value returns the value of the IRQ Status
//! Register
//
//===============================================================
uint8_t TRF79xxA_readIrqStatus(void)
{
	uint8_t zeroValue = 0x00;
	uint8_t pui8ValueSend = TRF79XXA_IRQ_STATUS;
	pui8ValueSend = (0x7f & pui8ValueSend);		// address
	pui8ValueSend = (0x40 | pui8ValueSend);		// non continuous read

	uint8_t pui8ValueRecv = 0x00;

	HAL_GPIO_WritePin(SPIx_SS_GPIO_PORT, SPIx_SS_PIN,  GPIO_PIN_RESET);

	// SPI send Read IRQ Status
	if (HAL_SPI_Transmit(&SpiHandle, &pui8ValueSend, sizeof(uint8_t), 1000) != HAL_OK) {
		Error_Handler();
	}
	// Receive IRQ status
	if (HAL_SPI_Receive(&SpiHandle, &pui8ValueRecv, sizeof(uint8_t), 1000) != HAL_OK) {
		Error_Handler();
	}
	// Dummy read
	if (HAL_SPI_Transmit(&SpiHandle, &zeroValue, sizeof(uint8_t), 1000) != HAL_OK) {
		Error_Handler();
	}

	HAL_GPIO_WritePin(SPIx_SS_GPIO_PORT, SPIx_SS_PIN,  GPIO_PIN_SET);

	return pui8ValueRecv;
}

//===============================================================
//
//! TRF79xxA_processIRQ - Services TRF79xxA IRQ interrupts
//!
//! \param pui8IrqStatus is the received IRQ Status flags
//!
//! The Interrupt Service Routine determines how the IRQ should
//! be handled. The TRF79xxA IRQ status register is read to
//! determine the cause of the IRQ. Conditions are checked and
//! appropriate actions taken.
//!
//! \return None.
//
//===============================================================
void TRF79xxA_processIRQ(uint8_t * pui8IrqStatus)
{
	uint8_t	ui8DummyRead;
	uint8_t	ui8Length;

	if(*pui8IrqStatus == (TRF79XXA_IRQ_STATUS_TX_COMPLETE | TRF79XXA_IRQ_STATUS_FIFO_HIGH_OR_LOW))			// BIT5 and BIT7
	{								// TX active and 32 bytes left in FIFO
		g_sTrfStatus = TX_COMPLETE;
	}
	else if(*pui8IrqStatus == TRF79XXA_IRQ_STATUS_TX_COMPLETE)
	{								// TX complete
		g_sTrfStatus = TX_COMPLETE;
		TRF79xxA_resetFIFO();				// reset the FIFO after TX
	}
	else if((*pui8IrqStatus & BIT1) == TRF79XXA_IRQ_STATUS_COLLISION_ERROR)
	{								// Collision error
		if ((g_eTrfGeneralSettings.ui8IsoControl == 0x08) || (g_eTrfGeneralSettings.ui8IsoControl == 0x88))
		{
			g_sTrfStatus = COLLISION_ERROR;

			g_ui8CollisionPosition = TRF79xxA_readRegister(TRF79XXA_COLLISION_POSITION);

			if (g_ui8CollisionPosition > 0x20)
			{
				ui8Length = g_ui8CollisionPosition - 0x20;		// number of valid bytes in FIFO

				if((ui8Length & 0x0F) != 0x00)
				{
					ui8Length = ui8Length + 0x10;	// add 1 byte if broken byte recieved
				}
				ui8Length = ui8Length >> 4;

				if(ui8Length != 0x00)
				{
					g_pui8TrfBuffer[g_ui8FifoOffset] = TRF79XXA_FIFO;		// write the recieved bytes to the correct place of the buffer

					TRF79xxA_readContinuous(&g_pui8TrfBuffer[g_ui8FifoOffset], ui8Length);
					g_ui8FifoOffset = g_ui8FifoOffset + ui8Length;
				}
			}
			else
			{
				g_ui8FifoRxLength = TRF79xxA_readRegister(TRF79XXA_FIFO_STATUS);
				g_ui8FifoRxLength &= 0x7F;

				g_pui8TrfBuffer[g_ui8FifoOffset] = TRF79XXA_FIFO;				// write the recieved bytes to the correct place of the buffer

				TRF79xxA_readContinuous(&g_pui8TrfBuffer[g_ui8FifoOffset], g_ui8FifoRxLength);
				g_ui8FifoOffset = g_ui8FifoOffset + g_ui8FifoRxLength;
			}
		}
		else if((g_eTrfGeneralSettings.ui8IsoControl & 0xF8) == 0x00)		// Covers all ISO15693 Data Rates for RFID mode with RX CRC on
		{
			g_sTrfStatus = COLLISION_ERROR;
		}
		else
		{
			g_sTrfStatus = PROTOCOL_ERROR;
		}

		TRF79xxA_sendDirectCommand(TRF79XXA_STOP_DECODERS_CMD);
		TRF79xxA_resetFIFO();		// reset the FIFO after TX
		TRF79xxA_resetIrqStatus();
	}
	else if(*pui8IrqStatus == TRF79XXA_IRQ_STATUS_RX_COMPLETE)
	{	// RX flag means that EOF has been recieved
		// and the number of unread bytes is in FIFOstatus regiter

		g_ui8FifoRxLength = TRF79xxA_readRegister(TRF79XXA_FIFO_STATUS);
		g_ui8FifoRxLength &= 0x7F;
		g_pui8TrfBuffer[g_ui8FifoOffset] = TRF79XXA_FIFO;				// write the received bytes to the correct place of the buffer

		TRF79xxA_readContinuous(&g_pui8TrfBuffer[g_ui8FifoOffset], g_ui8FifoRxLength);

		g_ui8FifoOffset = g_ui8FifoOffset + g_ui8FifoRxLength;

		TRF79xxA_resetFIFO();			// reset the FIFO after last byte has been read out

		if (g_sTrfStatus == RX_WAIT_EXTENSION)
		{
			g_ui8FifoRxLength = g_ui8FifoOffset;
		}

		g_sTrfStatus = RX_COMPLETE;
	}
	else if(*pui8IrqStatus == (TRF79XXA_IRQ_STATUS_RX_COMPLETE | TRF79XXA_IRQ_STATUS_FIFO_HIGH_OR_LOW))
	{									// RX active and 96 bytes allready in FIFO
		g_sTrfStatus = RX_WAIT;

		// Read FIFO Status to determine how many bytes are in the FIFO
		g_ui8FifoRxLength = TRF79xxA_readRegister(TRF79XXA_FIFO_STATUS);
		g_ui8FifoRxLength &= 0x7F;

		if (NFC_FIFO_SIZE > (g_ui8FifoOffset+g_ui8FifoRxLength))
		{
			// Read from the FIFO to empty it
			g_pui8TrfBuffer[g_ui8FifoOffset] = TRF79XXA_FIFO;
			TRF79xxA_readContinuous(&g_pui8TrfBuffer[g_ui8FifoOffset], g_ui8FifoRxLength);	// read all received bytes from FIFO
			g_ui8FifoOffset = g_ui8FifoOffset + g_ui8FifoRxLength;					// Adjust buffer index
		}
		else
		{
			g_sTrfStatus = PROTOCOL_ERROR;
			return;
		}

		// Read FIFO Status again to determine if more bytes have been received
		g_ui8FifoRxLength = TRF79xxA_readRegister(TRF79XXA_FIFO_STATUS);
		g_ui8FifoRxLength &= 0x7F;

		if (g_ui8FifoRxLength > 0)
		{
			g_sTrfStatus = RX_WAIT_EXTENSION;
		}
		else
		{
			g_ui8FifoRxLength = g_ui8FifoOffset;
			g_sTrfStatus = RX_COMPLETE;
			return;
		}
	}
	else if (*pui8IrqStatus == (TRF79XXA_IRQ_STATUS_RX_COMPLETE | TRF79XXA_IRQ_STATUS_NO_RESPONSE))
	{
		// RX has begun but as not completed, space exists in FIFO still, just wait longer to receive full reply.
		g_sTrfStatus = RX_WAIT_EXTENSION;
	}
	else if((*pui8IrqStatus & BIT4) == TRF79XXA_IRQ_STATUS_CRC_ERROR)		// CRC error
	{
		if((*pui8IrqStatus & BIT6) == TRF79XXA_IRQ_STATUS_RX_COMPLETE)		// 4 Bit receive
		{
			ui8DummyRead = TRF79XXA_FIFO;		// write the recieved bytes to the correct place of the buffer

			TRF79xxA_readContinuous(&ui8DummyRead, 1);
		}

		TRF79xxA_reset();

		g_sTrfStatus = PROTOCOL_ERROR;
	}
	else if((*pui8IrqStatus & BIT2) == TRF79XXA_IRQ_STATUS_FRAMING_ERROR)	// byte framing error
	{
		if((*pui8IrqStatus & BIT5) == TRF79XXA_IRQ_STATUS_FIFO_HIGH_OR_LOW)
		{
			g_sTrfStatus = RX_WAIT;
		}
		else if ((*pui8IrqStatus & BIT6) == TRF79XXA_IRQ_STATUS_RX_COMPLETE)
		{
			if((g_eTrfGeneralSettings.ui8IsoControl & 0xF8) == 0x00)		// Covers all ISO15693 Data Rates for RFID mode with RX CRC on
			{
				g_sTrfStatus = COLLISION_ERROR;
			}
			else
			{
				g_sTrfStatus = PROTOCOL_ERROR;

				TRF79xxA_reset();
			}
		}
		else
		{
			g_sTrfStatus = PROTOCOL_ERROR;

			TRF79xxA_reset();
		}
	}
	else if(*pui8IrqStatus == TRF79XXA_IRQ_STATUS_IDLE)
	{						// No response interrupt
		g_sTrfStatus = NO_RESPONSE_RECEIVED;
	}
	else if(*pui8IrqStatus == TRF79XXA_IRQ_STATUS_NO_RESPONSE)
	{						// No response interrupt
		g_sTrfStatus = NO_RESPONSE_RECEIVED_15693;
		g_ui8FifoOffset = 0;
	}
	else
	{						// Interrupt register not properly set
		g_sTrfStatus = PROTOCOL_ERROR;

		TRF79xxA_sendDirectCommand(TRF79XXA_STOP_DECODERS_CMD);	// reset the FIFO after TX
		TRF79xxA_reset();
		TRF79xxA_resetIrqStatus();
	}
}


//===============================================================
//
//! TRF79xxA_waitTxIRQ - Timeout sequence for just TX
//!
//! \param ui8TxTimeout is the TX timeout in milliseconds
//!
//! This function ensures data has been transmitted correctly
//! only.
//!
//! \return None.
//
//===============================================================
void TRF79xxA_waitTxIRQ(uint8_t ui8TxTimeout)
{
	g_sTrfStatus = RX_WAIT;
	g_ui8TimeoutFlag = 0x00;

	// Wait for end of TX
	while((g_sTrfStatus != TX_COMPLETE) && (g_sTrfStatus != TX_ERROR))
	{
		// Clear the IRQ Flag
		g_ui8IrqFlag = 0x00;

		// Setup for the Timer
		waitMilliSeconds(ui8TxTimeout);
		// Wait for an interrupt
		while((g_ui8IrqFlag == 0x00) && (g_ui8TimeoutFlag == 0x00));

		if (g_sTrfStatus != TX_COMPLETE) {
			if (g_sTrfStatus == TX_WAIT) {
				// Wait longer since we received an 0xA0
				TRF79xxA_waitTxIRQ(ui8TxTimeout);
			}
			else {
				// Failed to send packet properly - Exit TX Timeout
				g_sTrfStatus = TX_ERROR;
			}
		}
		else
			break;
	}
}

//===============================================================
//
//! TRF79xxA_waitRxIRQ - Timeout sequence for just RX
//!
//! \param ui8RxTimeout is the RX timeout in milliseconds
//!
//! This function determines if any data has been received prior
//! to the RX timeout only.
//!
//! When the RX timeout occurs before data is received, then mark
//! the TRF79xxA status as a No Response Received status.
//!
//! \return None.
//
//===============================================================
void TRF79xxA_waitRxIRQ(uint8_t ui8RxTimeout)
{
	// Reset the FIFO Offset prior to receiving data
	g_ui8FifoOffset = 0;

	g_sTrfStatus = RX_WAIT;
	g_ui8TimeoutFlag = 0x00;

	// Wait for end of RX or timeout
	while(g_sTrfStatus == RX_WAIT) {
		// Clear the IRQ Flag
		g_ui8IrqFlag = 0x00;

		// Setup for the Timer
		waitMilliSeconds(ui8RxTimeout);
		// Wait for an interrupt
		while((g_ui8IrqFlag == 0x00) && (g_ui8TimeoutFlag == 0x00));

		while (g_sTrfStatus == RX_WAIT_EXTENSION)
		{
			g_ui8IrqFlag = 0x00;
			if ((g_eTrfGeneralSettings.ui8IsoControl & 0x1F) > 0x07)
				TimHandle.Init.Period = 7 - 1;
			else
				TimHandle.Init.Period = 50 - 1;
			HAL_TIM_Base_Start_IT(&TimHandle);

			// Wait for an interrupt
			while((g_ui8IrqFlag == 0x00) && (g_ui8TimeoutFlag == 0x00));
		}

		if (g_sTrfStatus == RX_WAIT) {
			g_sTrfStatus = NO_RESPONSE_RECEIVED;
		}
	}
}

//===============================================================
//
//! TRF79xxA_waitRxData -
//!
//! \return g_sTrfStatus returns the current TRF79xxA drive
//! status
//
//===============================================================
tTrfStatus TRF79xxA_waitRxData(uint8_t ui8TxTimeout, uint8_t ui8RxTimeout)
{
	switch (g_sTrfStatus)
	{
	case TRF_IDLE:
	case TX_WAIT:
		TRF79xxA_waitTxIRQ(ui8TxTimeout);		// TX timeout
		TRF79xxA_waitRxIRQ(ui8RxTimeout);		// RX timeout
		break;
	case TX_COMPLETE:
		TRF79xxA_waitRxIRQ(ui8RxTimeout);		// RX timeout
		break;
	case NO_RESPONSE_RECEIVED_15693:
		// Do Nothing
		break;
	case COLLISION_ERROR:
		if ((g_eTrfGeneralSettings.ui8IsoControl == 0x02) || (g_eTrfGeneralSettings.ui8IsoControl == 0x08) || (g_eTrfGeneralSettings.ui8IsoControl == 0x88))
		{
			// Do Nothing
		}
		else
		{
			return g_sTrfStatus;
		}
		break;
	case RX_COMPLETE:
	case RX_WAIT:
	case RX_WAIT_EXTENSION:
		// Do Nothing
		break;
	default:
		g_sTrfStatus = TX_ERROR;
		break;
	}

	return g_sTrfStatus;
}


//===============================================================
//
//! TRF79xxA_readContinuous - Read multiple TRF79xxA registers
//!
//! \param pui8Payload is the address of the first register as
//! well as the pointer for buffer where the results will be
//! \param ui8Length is the number of registers to read
//!
//! This function reads a specified number of TRF79xxA registers
//! from a specified address.
//!
//! \return None.
//
//===============================================================
void TRF79xxA_readContinuous(uint8_t * pui8Payload, uint8_t ui8Length)
{
	// Set Address/Command Word Bit Distribution to command
	*pui8Payload = (0x7f & *pui8Payload);			// address
	*pui8Payload = (0x60 | *pui8Payload);			// continuous read

	// SLAVE_SELECT_LOW Start SPI Mode
	HAL_GPIO_WritePin(SPIx_SS_GPIO_PORT, SPIx_SS_PIN,  GPIO_PIN_RESET);

	// SPI send address byte
	if (HAL_SPI_Transmit(&SpiHandle, pui8Payload, 1, 1000) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_SPI_Receive(&SpiHandle, pui8Payload, ui8Length, 1000) != HAL_OK) {
		Error_Handler();
	}

	// Stop SPI Mode
	HAL_GPIO_WritePin(SPIx_SS_GPIO_PORT, SPIx_SS_PIN,  GPIO_PIN_SET);
}

//===============================================================
//
//! TRF79xxA_writeContinuous - Write to consecutive TRF79xxA
//! registers.
//!
//! \param pui8Payload is the address of the first register
//! followed by the contents to write for each register
//! \param ui8Length is the number of registers to write + 1
//! Minimum value of ui8Length allowed = 2 (a write to 1 register)
//!
//! This function writes data to a specific number of TRF79xxA
//! registers from a specific address.
//!
//! \return None.
//
//===============================================================
void TRF79xxA_writeContinuous(uint8_t * pui8Payload, uint8_t ui8Length)
{
	// Check
	if (ui8Length <= 1)
		return;
	if (*pui8Payload == TRF79XXA_CHIP_STATUS_CONTROL)	// If the write starts at the Chip Status Control Register
	{
		if (((*pui8Payload+1) & BIT5) == BIT5)	// Check for RF field bit and update variable
		{
			g_eTrfGeneralSettings.bRfFieldOn = true;
		}
		else {
			g_eTrfGeneralSettings.bRfFieldOn = false;
		}
		if (ui8Length > 2)		// Check if the write length includes the ISO Control Register being written to (0x01)
		{
			g_eTrfGeneralSettings.ui8IsoControl = (*pui8Payload+2);	// If so, update the Iso Control Register variable
		}
	}
	else if (*pui8Payload == TRF79XXA_ISO_CONTROL)	// If the write starts at the ISO Control Register
	{
		g_eTrfGeneralSettings.ui8IsoControl = *pui8Payload+1;	// Update the ISO Control Register variable
	}

	// Set Address/Command Word Bit Distribution to command
	*pui8Payload = (0x7f & *pui8Payload);			// address
	*pui8Payload = (0xbf & *pui8Payload);			// continuous write

	// SLAVE_SELECT_LOW Start SPI Mode
	HAL_GPIO_WritePin(SPIx_SS_GPIO_PORT, SPIx_SS_PIN,  GPIO_PIN_RESET);

	// Call continuous write function
	if (HAL_SPI_Transmit(&SpiHandle, pui8Payload, ui8Length, 1000) != HAL_OK) {
		Error_Handler();
	}

	// Stop SPI Mode
	HAL_GPIO_WritePin(SPIx_SS_GPIO_PORT, SPIx_SS_PIN,  GPIO_PIN_SET);
}

//===============================================================
//
//! TRF79xxA_readRegister - Read out a single TRF79xxA register
//!
//! This function reads a specific TRF79xxA register.
//!
//! \return pui8Value returns the value of the TRF79xxA Register
//
//===============================================================
uint8_t TRF79xxA_readRegister(uint8_t ui8TrfRegister)
{
	uint8_t recvValue = 0x00;

	// Set Address/Command Word Bit Distribution to command
	ui8TrfRegister = (0x7f & ui8TrfRegister);		// address
	ui8TrfRegister = (0x40 | ui8TrfRegister);		// non continuous read

	// SLAVE_SELECT_LOW Start SPI Mode
	HAL_GPIO_WritePin(SPIx_SS_GPIO_PORT, SPIx_SS_PIN,  GPIO_PIN_RESET);

	// SPI send address byte
	if (HAL_SPI_Transmit(&SpiHandle, &ui8TrfRegister, sizeof(uint8_t), 1000) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_SPI_Receive(&SpiHandle, &recvValue, sizeof(uint8_t), 1000) != HAL_OK) {
		Error_Handler();
	}

	// Stop SPI Mode
	HAL_GPIO_WritePin(SPIx_SS_GPIO_PORT, SPIx_SS_PIN,  GPIO_PIN_SET);

	return recvValue;
}

//===============================================================
//
//! TRF79xxA_writeRegister - Write single to a TRF79xxA Register
//!
//! \param ui8TrfRegister is the register address for the write
//! \param ui8Value is the value to write to the specified
//! register
//!
//! This function writes a new value into a single TRF79xxA
//! register.
//!
//! \return None.
//
//===============================================================
void TRF79xxA_writeRegister(uint8_t ui8TrfRegister, uint8_t ui8Value)
{
	if (ui8TrfRegister == TRF79XXA_ISO_CONTROL) {
		// Attempt to enable Card Emulation/Peer to Peer which is not supported by firmware
		// Exit function to avoid issues with that
		if ((ui8Value & BIT5) == BIT5)
			return;

		g_eTrfGeneralSettings.ui8IsoControl = ui8Value;	// Update the ISO Control Register variable
	}

	if (ui8TrfRegister == TRF79XXA_CHIP_STATUS_CONTROL) {
		if ((ui8Value & BIT5) == BIT5)	// Check for RF field bit and update variable
			g_eTrfGeneralSettings.bRfFieldOn = true;
		else
			g_eTrfGeneralSettings.bRfFieldOn = false;
	}

	// Set Address/Command Word Bit Distribution to command
	ui8TrfRegister = (0x1f & ui8TrfRegister);		// address non continuous write

	// SLAVE_SELECT_LOW Start SPI Mode
	HAL_GPIO_WritePin(SPIx_SS_GPIO_PORT, SPIx_SS_PIN,  GPIO_PIN_RESET);

	// Call continuous write function
	if (HAL_SPI_Transmit(&SpiHandle, &ui8TrfRegister, sizeof(uint8_t), 1000) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_SPI_Transmit(&SpiHandle, &ui8Value, sizeof(uint8_t), 1000) != HAL_OK) {
		Error_Handler();
	}

	// Stop SPI Mode
	HAL_GPIO_WritePin(SPIx_SS_GPIO_PORT, SPIx_SS_PIN,  GPIO_PIN_SET);
}

//===============================================================
//
//! TRF79xxA_sendDirectCommand - Send a Direct Command to TRF79xxA.
//!
//! \param ui8Value is the direct command to be issued
//!
//! This function is used to transmit a Direct Command to TRF79xxA.
//!
//! \return None.
//
//===============================================================
void TRF79xxA_sendDirectCommand(uint8_t ui8Value)
{
	// SLAVE_SELECT_LOW Start SPI Mode
	HAL_GPIO_WritePin(SPIx_SS_GPIO_PORT, SPIx_SS_PIN,  GPIO_PIN_RESET);

	// Set Address/Command Word Bit Distribution to command
	ui8Value = (0x80 | ui8Value);					// command
	ui8Value = (0x9f & ui8Value);					// command code

	// SPI send byte
	if (HAL_SPI_Transmit(&SpiHandle, &ui8Value, sizeof(uint8_t), 1000) != HAL_OK) {
		Error_Handler();
	}

	// Stop SPI Mode
	HAL_GPIO_WritePin(SPIx_SS_GPIO_PORT, SPIx_SS_PIN,  GPIO_PIN_SET);
}

//===============================================================
//
//! TRF79xxA_writeRaw - Write data to TRF79xxA
//!
//! \param pui8Payload is the buffer with data packet contents
//! \param ui8Length is the size of the data packet
//!
//! This function writes provided data directly to the TRF79xxA.
//!	Data size (ui8Length) should be smaller than TRF79xxA_MAX_FIFO_SIZE (127).
//!
//! \return None.
//
//===============================================================
void TRF79xxA_writeRaw(uint8_t * pui8Payload, uint8_t ui8Length)
{
	g_sTrfStatus = TRF_IDLE;

	// First send includes 5 bytes for command overhead
	if (ui8Length < TRF79xxA_MAX_FIFO_SIZE + 5) {
		// SLAVE_SELECT_LOW Start SPI Mode
		HAL_GPIO_WritePin(SPIx_SS_GPIO_PORT, SPIx_SS_PIN,  GPIO_PIN_RESET);

		int i = 0;
		for (; i < ui8Length; ++i)
			if (HAL_SPI_Transmit(&SpiHandle, pui8Payload + i, 1, 1000) != HAL_OK)
				Error_Handler();

		// Stop SPI Mode
		HAL_GPIO_WritePin(SPIx_SS_GPIO_PORT, SPIx_SS_PIN,  GPIO_PIN_SET);
	}
	else
		Error_Handler();
}




//===============================================================
//
//! TRF79xxA_timerHandler - Handler for assigned MCU Timer
//! Interrupt of TX/RX Timeout functions.
//!
//! This function processes the Timer Interrupt for the TX/RX
//! timeout functions. If a timeout occurs, the IRQ Status
//! register is read out to determine the current TRF79xxA state.
//!
//! \return None.
//!
//! \parent procedure: HAL_TIM_IRQHandler
//
//===============================================================
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// Stop the Timer
	HAL_TIM_Base_Stop_IT(htim);

	// If this is Timer for delay purpose
	if (htim->Instance == DELAY_TIM) {
		delayStatus = TIM_OVER;
		return;
	}

	// If this is Timer for TRF79xxA communication purpose
	if (htim->Instance == WAIT_TIM) {

		// Read IRQ on the second timer
		if (waitStatus == TIM_WAIT) {
			uint8_t ui8IrqStatus = TRF79xxA_readIrqStatus();

			if(ui8IrqStatus == TRF79XXA_IRQ_STATUS_TX_COMPLETE)
				g_sTrfStatus = TX_COMPLETE;
			else if(ui8IrqStatus == TRF79XXA_IRQ_STATUS_IDLE)
				g_sTrfStatus = NO_RESPONSE_RECEIVED;
			else
				g_sTrfStatus = RX_WAIT;
		}

		waitStatus = TIM_OVER;
		g_ui8TimeoutFlag = 0x01;

	}
}

//===============================================================
//
//! TRF79xxA_irqHandler- Interrupt handler for IRQ interrupts
//!
//! Handles receiving IRQ's, getting IRQ status, and maintaining
//! timers/global variables
//!
//! \return None.
//
//===============================================================
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == TRF79XXA_IRQ_PIN)
	{
		// TRF79xxA_irqHandler
		uint8_t ui8IrqStatus;

		// stop timer mode
		HAL_TIM_Base_Stop_IT(&TimHandle);

		g_ui8IrqFlag = 0x01;

		ui8IrqStatus = TRF79xxA_readIrqStatus();

		// TX active and only 3 bytes left in FIFO
		if(ui8IrqStatus == 0xA0) {
			g_sTrfStatus = TX_WAIT;
		}
		else {
			TRF79xxA_processIRQ(&ui8IrqStatus);
		}

		if (waitStatus == TIM_WAIT)
			waitStatus = TIM_OVER;
	}
}

