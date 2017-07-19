#include "trf7970a.h"
#include "timer.h"


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

		// TODO: IRQ_CLR;
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

		// TODO: IRQ_CLR;
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
//!
//! \return None.
//
//===============================================================
void TRF79xxA_writeRaw(uint8_t * pui8Payload, uint8_t ui8Length)
{
	uint8_t ui8TxBytesRemaining;
	uint8_t ui8TxIndex;
	uint8_t ui8FifoTxLength;
	uint8_t ui8TxBytesAvailable;
//	bool bContinuedSend;

	g_sTrfStatus = TRF_IDLE;

	// First send includes 5 bytes for command overhead
	if (TRF79xxA_MAX_FIFO_SIZE + 5 > ui8Length) {
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
//	else {
//		ui8TxBytesRemaining = ui8Length;
//		ui8TxIndex = 0;
//		ui8TxBytesAvailable = TRF79xxA_MAX_FIFO_SIZE + 5; // First send includes 5 bytes for command overhead
//														// (Reset FIFO, Transmit with or without CRC, Continuous Write, Length High and Length Low)
//		// bContinuedSend = false;							// First send is not continued
//
//		while(ui8TxBytesRemaining > 0) {
//			if (ui8TxBytesRemaining > TRF79xxA_MAX_FIFO_SIZE) {
//				if (HAL_SPI_Transmit(&SpiHandle, &pui8Payload[ui8TxIndex], ui8TxBytesAvailable, 1000) != HAL_OK)
//					Error_Handler();
//
//				ui8TxBytesRemaining = ui8TxBytesRemaining - ui8TxBytesAvailable;
//				ui8TxIndex = ui8TxIndex + ui8TxBytesAvailable;
//				// bContinuedSend = true;
//			}
//			else {
//				// Last send
//				if (HAL_SPI_Transmit(&SpiHandle, &pui8Payload[ui8TxIndex], ui8TxBytesRemaining, 1000) != HAL_OK)
//					Error_Handler();
//				// bContinuedSend = false;
//				ui8TxBytesRemaining = 0;
//			}
//
//			g_ui8TimeoutFlag = 0x00;
//			// Clear the IRQ Flag
//			g_ui8IrqFlag = 0x00;
//
//			// Setup for the Timer
//			waitMilliSeconds(5);
//			// Wait for an interrupt
//			while((g_ui8IrqFlag == 0x00) && (g_ui8TimeoutFlag == 0x00));
//
//			if (g_sTrfStatus == TX_WAIT) {
//				ui8FifoTxLength = TRF79xxA_readRegister(TRF79XXA_FIFO_STATUS);
//				ui8FifoTxLength &= 0x7F;
//				ui8TxBytesAvailable = TRF79xxA_MAX_FIFO_SIZE-ui8FifoTxLength;
//			}
//			else if (g_sTrfStatus == TX_COMPLETE)
//			{
//				if (ui8TxBytesRemaining == 0) {
//					// Packet is sent
//					break;
//				}
//				else {
//					ui8FifoTxLength = TRF79xxA_readRegister(TRF79XXA_FIFO_STATUS);
//					ui8FifoTxLength &= 0x7F;
//					ui8TxBytesAvailable = TRF79xxA_MAX_FIFO_SIZE - ui8FifoTxLength;
//					// bContinuedSend = true;
//				}
//			}
//			else {
//				// Error occurred, break
//				g_sTrfStatus = TX_ERROR;
//				break;
//			}
//		}
//	}
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
	if (htim->Instance == TIMx) {

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
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == TRF79XXA_IRQ_PIN) {
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

/************************************ ISO ISO15693 Specific *****************************************/

//*****************************************************************************
//
//! ISO15693_init - Initialize all Global Variables for the ISO15693 layer.
//!
//! This function will initialize all the global variables for the ISO15693
//! layer with the appropriate starting values or empty values/buffers.
//!
//! \return None
//
//*****************************************************************************
void TRF79xxA_ISO15693_init(void)
{
	uint8_t ui8LoopCount;

	g_ui8TagDetectedCount = 0;
	g_ui8RecursionCount = 0;

	for (ui8LoopCount = 0; ui8LoopCount < 8; ui8LoopCount++) {
		g_pui8Iso15693UId[ui8LoopCount] = 0xFF;  // Initialize with invalid UID
	}

	for (ui8LoopCount = 0; ui8LoopCount < 8; ui8LoopCount++) {
		g_pui8AnticollisionMaskBuffer[ui8LoopCount] = 0x00;
	}
}

//*****************************************************************************
//
//! ISO15693_sendSingleSlotInventory - Issue a single slot Inventory command
//! for ISO15693 tags.
//!
//! This function issues a single slot Inventory command for tag detection
//! of ISO15693 tags. If a tag is found, the UID is stored in the
//! g_pui8Iso15693UId buffer.
//!
//! If UART is enabled, the tag ID is sent out to a host via UART.
//!
//! \return ui8Status returns either STATUS_SUCCESS or STATUS_FAIL to indicate
//! if the Inventory command resulted in a successful tag detection or not.
//
//*****************************************************************************
uint8_t TRF79xxA_ISO15693_sendSingleSlotInventory()
{
	uint8_t ui8Offset = 0;
	uint8_t ui8LoopCount = 0;
	uint8_t ui8Status = STATUS_FAIL;

	g_pui8TrfBuffer[ui8Offset++] = 0x8F;		// Reset FIFO
	g_pui8TrfBuffer[ui8Offset++] = 0x91;		// Send with CRC
	g_pui8TrfBuffer[ui8Offset++] = 0x3D;		// Write Continuous
	g_pui8TrfBuffer[ui8Offset++] = 0x00;		// Length of packet in bytes - upper and middle nibbles of transmit byte length
	g_pui8TrfBuffer[ui8Offset++] = 0x30;		// Length of packet in bytes - lower and broken nibbles of transmit byte length
	g_pui8TrfBuffer[ui8Offset++] = 0x26;		// ISO15693 flags
	g_pui8TrfBuffer[ui8Offset++] = 0x01;		// Inventory command code
	g_pui8TrfBuffer[ui8Offset++] = 0x00;		// Mask Length = 0 (Also not sending AFI)

	// Issue the ISO15693 Inventory Command
	TRF79xxA_writeRaw(&g_pui8TrfBuffer[0], ui8Offset);

	g_sTrfStatus = TRF79xxA_waitRxData(5,15);			// 5 millisecond TX timeout, 15 millisecond RX timeout

	if (g_sTrfStatus == RX_COMPLETE) {
		if (g_pui8TrfBuffer[0] == 0x00) {

			ui8Status = STATUS_SUCCESS;

			// UID Starts at the 3rd received bit (1st is flags and 2nd is DSFID)
			for (ui8LoopCount = 2; ui8LoopCount < 10; ui8LoopCount++) {
				// Store UID into a Buffer
				g_pui8Iso15693UId[ui8LoopCount-2] = g_pui8TrfBuffer[ui8LoopCount];
			}

			g_ui8TagDetectedCount = 1;
		}
	}
	else
		ui8Status = STATUS_FAIL;

	return ui8Status;
}

//*****************************************************************************
//
//! ISO15693_runAnticollision - Issue an Inventory command for either
//! 1 or 16 slot anticollision of ISO15693 tags.
//!
//! \param ui8ReqFlag are the request flags for ISO15693 commands.
//! \param ui8MaskLength is the number of significant bits in the mask value.
//! \param ui8Afi is the AFI to be issued with command (if AFI flag is
//! included in ui8ReqFlag).
//!
//! Function issues a sixteen slot Inventory command for the detection of
//! ISO15693 tags. If a tag is found, the UID is stored in the g_ui8Iso15693UId
//! buffer. The process will run until all ISO15693 detected have responded
//! with their UID's.
//!
//! The function uses a recursive call for the anticollision process. In order
//! to avoid stack overflows, a recursion counter is used to limit the maximum
//! number of recursions.
//!
//! The number of UID's which can be stored is set by g_ui8MaximumTagCount
//! and the declaration of the g_ui8Iso15693UId buffer. Once the buffer for
//! UID's is filled then no other UID's responses will be stored.
//!
//! If UART is enabled, the UID of each ISO15693 tag detected is sent out to a
//! host via UART.
//!
//! \return ui8Status returns STATUS_SUCCESS if the anticollision function
//! resulted in a successful tag detection. Otherwise, returns STATUS_FAIL.
//
//*****************************************************************************
uint8_t TRF79xxA_ISO15693_runAnticollision(uint8_t ui8ReqFlags, uint8_t ui8MaskLength, uint8_t ui8Afi)
{
	uint8_t ui8Offset = 0;
	uint8_t ui8LoopCount1;
	uint8_t ui8LoopCount2;
	uint16_t ui16TransmitByteLength;
	uint16_t ui16SlotNumber = 0x0000;
	uint8_t ui8MaskValue;
	uint8_t ui8MaskByteCount;
	uint8_t ui8Status = STATUS_FAIL;

	// Clear Bit 5 to ensure 16 slot inventory is used.
	ui8ReqFlags &= ~BIT5;

	// Set ui8MaskByteCount based on the inputted Mask Length
	//     ui8MaskByteCount will be 1 for length = 4 or 8,
	//     ui8MaskByteCount will be 2 for length = 12 or 16,
	//     and so on
	ui8MaskByteCount = (((ui8MaskLength >> 2) + 1) >> 1);

	// Calculate how long the output byte will be
	if (ui8ReqFlags & 0x10) {
		// Check if AFI will be included or not
		// Set packet size = Mask Value + Mask Length + AFI Byte + ISO15693 Command Code + ISO15693 Request Flags
		ui16TransmitByteLength = ui8MaskByteCount + 4;
	}
	else {
		// Set packet size = Mask Value + Mask Length + ISO15693 Command Code + ISO15693 Request Flags
		ui16TransmitByteLength = ui8MaskByteCount + 3;
	}

	// Format Anti-collision command packet
	g_pui8TrfBuffer[ui8Offset++] = 0x8F;					// Reset FIFO
	g_pui8TrfBuffer[ui8Offset++] = 0x91;					// Send with CRC
	g_pui8TrfBuffer[ui8Offset++] = 0x3D;					// Write Continuous
	g_pui8TrfBuffer[ui8Offset++] = (uint8_t) (ui16TransmitByteLength >> 8);		// Length of packet in bytes - upper and middle nibbles of transmit byte length
	g_pui8TrfBuffer[ui8Offset++] = (uint8_t) (ui16TransmitByteLength << 4);		// Length of packet in bytes - lower and broken nibbles of transmit byte length
	g_pui8TrfBuffer[ui8Offset++] = ui8ReqFlags;				// ISO15693 Request Flags
	g_pui8TrfBuffer[ui8Offset++] = 0x01;					// Inventory Request Command Code

	if (ui8ReqFlags & 0x10) {
		// Check if AFI will be included or not
		g_pui8TrfBuffer[ui8Offset++] = ui8Afi;			// Optional AFI Byte
		g_pui8TrfBuffer[ui8Offset++] = ui8MaskLength;	// Mask Length
		if (ui8MaskLength > 0) {
			for (ui8LoopCount1 = 0; ui8LoopCount1 < ui8MaskByteCount; ui8LoopCount1++) {
				// Fill in inputted Mask Values
				g_pui8TrfBuffer[ui8Offset++] = g_pui8AnticollisionMaskBuffer[(ui8MaskByteCount-ui8LoopCount1)];
			}
		}
	}
	else {
		g_pui8TrfBuffer[ui8Offset++] = ui8MaskLength;	// Mask Length
		if (ui8MaskLength > 0) {
			for (ui8LoopCount1 = 0; ui8LoopCount1 < ui8MaskByteCount; ui8LoopCount1++) {
				// Fill in inputted Mask Values
				g_pui8TrfBuffer[ui8Offset++] = g_pui8AnticollisionMaskBuffer[((ui8MaskByteCount-1)-ui8LoopCount1)];
			}
		}
	}

	TRF79xxA_enableSlotCounter();

	TRF79xxA_resetIrqStatus();

	TRF79xxA_writeRaw(&g_pui8TrfBuffer[0], ui8Offset);		// Issue the ISO15693 Inventory Command

	g_sTrfStatus = TRF79xxA_getTrfStatus();

	if (g_sTrfStatus == TRF_IDLE || g_sTrfStatus == TX_WAIT) {
		TRF79xxA_waitTxIRQ(5);				// 5 millisecond TX timeout
	}

	for (ui8LoopCount2 = 1; ui8LoopCount2 <= 16; ui8LoopCount2++)
	{
		TRF79xxA_waitRxIRQ(15);		// 15 millisecond RX timeout

		g_sTrfStatus = TRF79xxA_getTrfStatus();	// Get the TRF797x Status

		switch (g_sTrfStatus)
		{
		case RX_COMPLETE:						// If data has been received, then UID is in the buffer
			if (g_pui8TrfBuffer[0] == 0x00)		// Confirm "no error" in response flags byte
			{
				ui8Status = STATUS_SUCCESS;

				// UID Starts at the 3rd received bit (1st is flags and 2nd is DSFID)
				for (ui8LoopCount1 = 2; ui8LoopCount1 < 10; ui8LoopCount1++)
				{
					g_pui8Iso15693UId[ui8LoopCount1-2] = g_pui8TrfBuffer[ui8LoopCount1];	// Store UID to a Buffer
				}

				g_ui8TagDetectedCount++;
#ifdef ENABLE_HOST
				// Print out UID to UART Host
				UART_putNewLine();
				UART_sendCString("ISO15693 UID: ");
				UART_putChar('[');
				for (ui8LoopCount1 = 0; ui8LoopCount1 < 8; ui8LoopCount1++)
				{
					UART_putByte(g_pui8Iso15693UId[7-ui8LoopCount1]);		// Send UID to host
				}
				UART_putChar(']');
				UART_putNewLine();
#endif
			}
			break;

		case COLLISION_ERROR:
			// A collision has occurred for this slot
			// Mark a collision occurred in the correct Slot Number bit.
			ui16SlotNumber |= (0x01 << (ui8LoopCount2-1));
			// Allow time for tag to finish responding before issuing EOF
			delayMilliSeconds(5);
			break;

		case NO_RESPONSE_RECEIVED:
			// No Response was received, break out of function as there is no tag for this slot
			break;

		case NO_RESPONSE_RECEIVED_15693:
			// No Response was received, break out of function as there is no tag for this slot
			break;

		case PROTOCOL_ERROR:
			// Protocol Error occurred, exit out of anticollision
#ifdef ENABLE_HOST
			UART_sendCString("ISO15693 Anticollision Error");
			UART_putNewLine();
#endif
			TRF79xxA_setupInitiator(0x02);
			return ui8Status = STATUS_FAIL;

		default:
			break;
		}

		// FIFO has to be reset before receiving the next response
		TRF79xxA_resetFIFO();

		if (ui8LoopCount2 < 16)	{
			// If 16 slots used, and the last slot as not been reached, then send EOF (i.e. next slot indicator)
			TRF79xxA_sendDirectCommand(TRF79XXA_STOP_DECODERS_CMD);
			TRF79xxA_sendDirectCommand(TRF79XXA_RUN_DECODERS_CMD);
			TRF79xxA_sendDirectCommand(TRF79XXA_TRANSMIT_NEXT_SLOT_CMD);
		}
		else if (ui8LoopCount2 == 16) {
			// Once at the end of slot 16, then stop the slot counter
			TRF79xxA_sendDirectCommand(TRF79XXA_STOP_DECODERS_CMD);
			TRF79xxA_disableSlotCounter();
		}
	}

	TRF79xxA_disableSlotCounter();

	// The mask length is a multiple of 4 bits
	ui8MaskLength = ui8MaskLength + 4;

	// Set ui8MaskByteCount based on the inputted Mask Length
	//     ui8MaskByteCount is 1 for length = 4 or 8,
	//     ui8MaskByteCount is 2 for length = 12 or 16,
	//     and so on
	ui8MaskByteCount = (((ui8MaskLength >> 2) + 1) >> 1);

	// If the slot number pointer is not 0, the slot count is 16 (to indicate anticollision is needed),
	// the mask length doesn't exceed 60 bits, and the slot number is not 16 then proceed to recursive function call
	while ((ui16SlotNumber != 0x00) && (ui8MaskLength < 61)) {

		ui8MaskValue = 0x00;
		ui8LoopCount1 = 0;

		while (ui8LoopCount1 < 16) {
			if ((ui16SlotNumber & (0x01 << ui8LoopCount1)) != 0x00) {
				ui8MaskValue = ui8LoopCount1;

				// Clear that slot bit from the array
				ui16SlotNumber &= ~(0x01 << ui8LoopCount1);

				break;
			}
			ui8LoopCount1++;
		}

		if ((ui8MaskLength & 0x04) == 0x00) {
			// Shift slot pointer if mask length doesn't have Bit 2 (0x04) set (since it is a multiple of 4 bits)
			ui8MaskValue = ui8MaskValue << 4;
		}
		else {
			// Otherwise re-copy the mask values
			for (ui8LoopCount1 = 7; ui8LoopCount1 > 0; ui8LoopCount1--) {
				g_pui8AnticollisionMaskBuffer[ui8LoopCount1] = g_pui8AnticollisionMaskBuffer[ui8LoopCount1 - 1];
			}
			// And set the mask value for the first byte in the array = 0
			g_pui8AnticollisionMaskBuffer[0] &= 0x00;
		}

		g_pui8AnticollisionMaskBuffer[0] |= ui8MaskValue;								// Now update the mask value of the first byte based on the slot number pointer

		delayMilliSeconds(2);

		if (g_ui8RecursionCount < ISO15693_MAX_RECURSION_COUNT) {
			g_ui8RecursionCount++;
			// Issue a recursive call with new Mask
			ui8Status = TRF79xxA_ISO15693_runAnticollision(ui8ReqFlags, ui8MaskLength, ui8Afi);
		}
		else {
			return ui8Status = STATUS_FAIL;
		}

		// Restore the Global AnticollisionMaskBuffer with the values from the current anticollision function.
		if ((ui8MaskLength & 0x04) == 0x00) {
			// If mask length doesn't have Bit 2 (0x04) set (since it is a multiple of 4 bits)
			// then clear the upper nibble which is where the new mask value was placed
			g_pui8AnticollisionMaskBuffer[0] &= 0x0F;
		}
		else {	// Otherwise re-shift the mask values
			for (ui8LoopCount1 = 0; ui8LoopCount1 < 7; ui8LoopCount1++) {
				g_pui8AnticollisionMaskBuffer[ui8LoopCount1] = g_pui8AnticollisionMaskBuffer[ui8LoopCount1 + 1];
			}
			// And set the mask value for the first byte in the array = 0
			g_pui8AnticollisionMaskBuffer[7] = 0x00;
		}
	}

	// Clear any IRQs
	TRF79xxA_resetIrqStatus();

	// Reduce the recursion count as stack space will be freed on function exit
	if (g_ui8RecursionCount > 0) {
		g_ui8RecursionCount--;
	}

	return ui8Status;
}

//*****************************************************************************
//
//! NFC_appIso15693ReadTag - Read all blocks of a ISO15693 tag.
//!
//! \param ui8ReqFlag are the request flags for ISO15693 commands.
//!
//! This function issues Get System Information command to determine how many
//! blocks of data are stored within the ISO15693 tag.
//!
//! Afterwards, all blocks are read out using a Read Single block, unless an
//! error occurs during the read process at which point the function will stop
//! reading data and exit.
//!
//! \return None.
//
//*****************************************************************************
void TRF79xxA_ISO15693_ReadTag(uint8_t ui8ReqFlag)
{
	uint16_t ui16ReadBlocks = 0x00;
	uint16_t ui16LoopCount = 0x00;

	ui16ReadBlocks = TRF79xxA_ISO15693_sendGetSystemInfo(ui8ReqFlag); 	// Get Tag Information with Request Flag = 0x02

	if (ui16ReadBlocks != 0x00) {
		// Read all available blocks on the ISO15693 Tag
		for (ui16LoopCount = 0; ui16LoopCount < ui16ReadBlocks+1; ui16LoopCount++) {
			if (TRF79xxA_ISO15693_sendReadSingleBlock(ui8ReqFlag, ui16LoopCount) == STATUS_FAIL) {
				// Keep reading blocks unless a No Response is receive		{
				break;
			}
		}
	}
}

//*****************************************************************************
//
//! NFC_appIso15693ReadExtendedTag - Read all blocks of an ISO15693 tag that
//! requires the Protocol Extension flag to be used.
//!
//! \param ui8ReqFlag are the request flags for ISO15693 commands.
//!
//! This function issues Get System Information command with the Protocol
//! Extension bit set in the request flags to determine how many blocks of
//! data is stored within the ISO15693 tag.
//!
//! Then all blocks are read out using a Read Single block for Extended
//! ISO15693 tags, unless an error occurs during the read process at which
//! point the function will stop reading data and exit.
//!
//! \return None.
//
//*****************************************************************************
uint8_t TRF79xxA_ISO15693_ReadExtendedTag(uint8_t ui8ReqFlag, uint8_t* dataBuf) {
	uint16_t ui16ReadBlocks = 0x00;
	uint16_t i = 0x00;

	ui8ReqFlag |= 0x08; 	// Add in Protocol Extension Flag if it was omitted from the inputted request flags

	ui16ReadBlocks = TRF79xxA_ISO15693_sendGetSystemInfoExtended(ui8ReqFlag);	// Issue a Get System Info with Protocol Extension

	if (ui16ReadBlocks != 0x00)
	{
		// Read 8 blocks on the ISO15693 Tag
		for (i = 0; i < 9; i++) {
			// Keep reading blocks until a No Response is received
			if (TRF79xxA_ISO15693_sendReadSingleBlockExtended(ui8ReqFlag, i, dataBuf + 4*i) == STATUS_FAIL) {
				return STATUS_FAIL;
			}
		}
		return STATUS_SUCCESS;
	}
	return STATUS_FAIL;
}

//*****************************************************************************
//
//! ISO15693_sendReadMultipleBlocks - Issue the Read Multiple Block command
//! for ISO15693 tags.
//!
//! \param ui8ReqFlag are the request flags for ISO15693 commands.
//! \param ui8FirstBlock is the starting block number to read data from.
//! \param ui8NumberOfBlocks is the amount of blocks to read data from.
//!
//! This function issues a Read Multiple Block with the specified request
//! flags, the starting block number, and the number blocks to read data from.
//!
//! If UART is enabled, the data read from the ISO15693 tag is output via UART.
//!
//! \return ui8Status returns either STATUS_SUCCESS or STATUS_FAIL
//! to indicate if the Read Multiple Block was successful or not.
//
//*****************************************************************************
uint8_t TRF79xxA_ISO15693_sendReadMultipleBlocks(uint8_t ui8ReqFlag, uint8_t ui8FirstBlock, uint8_t ui8NumberOfBlocks)
{
	uint8_t ui8Offset = 0;
	uint8_t ui8Status = STATUS_FAIL;
#ifdef ENABLE_HOST
	uint8_t ui8LoopCount1 = 0;
	uint8_t ui8LoopCount2 = 0;
	uint8_t ui8RxLength = 0;
	uint8_t ui8BlockSize = 0;
#endif

	g_pui8TrfBuffer[ui8Offset++] = 0x8F;		// Reset FIFO
	g_pui8TrfBuffer[ui8Offset++] = 0x91;		// Send with CRC
	g_pui8TrfBuffer[ui8Offset++] = 0x3D;		// Write Continuous
	g_pui8TrfBuffer[ui8Offset++] = 0x00;		// Length of packet in bytes - upper and middle nibbles of transmit byte length
	if (ui8ReqFlag & 0x20)
	{
		g_pui8TrfBuffer[ui8Offset++] = 0xC0;		// Length of packet in bytes - lower and broken nibbles of transmit byte length
	}
	else
	{
		g_pui8TrfBuffer[ui8Offset++] = 0x40;		// Length of packet in bytes - lower and broken nibbles of transmit byte length
	}

	g_pui8TrfBuffer[ui8Offset++] = ui8ReqFlag;	// ISO15693 flags
	g_pui8TrfBuffer[ui8Offset++] = 0x23;			// Read Multiple Block command code

	if (ui8ReqFlag & 0x20)
	{
		g_pui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[0];	// Tag UID
		g_pui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[1];	// Tag UID
		g_pui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[2];	// Tag UID
		g_pui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[3];	// Tag UID
		g_pui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[4];	// Tag UID
		g_pui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[5];	// Tag UID
		g_pui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[6];	// Tag UID
		g_pui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[7];	// Tag UID
	}

	g_pui8TrfBuffer[ui8Offset++] = ui8FirstBlock;			// Number of the first block to read from

	if (ui8NumberOfBlocks > 0)
	{
		g_pui8TrfBuffer[ui8Offset++] = ui8NumberOfBlocks-1;	// Index for number of blocks to be read - this value is one less than
	}
	else
	{
		// Invalid count provided
		return ui8Status = STATUS_FAIL;
	}

	TRF79xxA_writeRaw(&g_pui8TrfBuffer[0], ui8Offset);		// Issue the Get System Information command

	g_sTrfStatus = TRF79xxA_waitRxData(10,30+ui8NumberOfBlocks);	// 10 millisecond TX timeout, 30 millisecond RX timeout - adding number of blocks to extend timeout for larger read requests

	if (g_sTrfStatus == RX_COMPLETE)		// If data has been received
	{
		if (g_pui8TrfBuffer[0] == 0x00)		// Confirm "no error" in response flags byte
		{
#ifdef ENABLE_HOST
			ui8RxLength = TRF79xxA_getRxBytesReceived();

			if (ui8ReqFlag & BIT6) // Handle case for Option Flag causing one extra byte to be transmitted.
			{
				ui8Offset = 2;
			}
			else
			{
				ui8Offset = 1;
			}

			ui8LoopCount1 = ui8RxLength-ui8Offset;

			while (ui8LoopCount1 > 0)
			{
				if (ui8LoopCount1 > ui8NumberOfBlocks)
				{
					ui8LoopCount1 = ui8LoopCount1 - ui8NumberOfBlocks;
				}
				else
				{
					ui8LoopCount1 = 0;
				}
				ui8BlockSize++;
			}

			for (ui8LoopCount2 = 0; ui8LoopCount2 < ui8NumberOfBlocks; ui8LoopCount2++)
			{
				UART_sendCString("Block ");
				UART_putByte(ui8FirstBlock+ui8LoopCount2);		// Output block number
				UART_sendCString(" Data: ");
				UART_putChar('[');

				// Output received data to UART
				for (ui8LoopCount1 = 0; ui8LoopCount1 < ui8BlockSize; ui8LoopCount1++)
				{
					UART_putByte(g_pui8TrfBuffer[ui8Offset++]);		// Send out data read from tag to host
				}

				UART_putChar(']');
				UART_putNewLine();
			}
#endif
			// Response received
			ui8Status = STATUS_SUCCESS;
		}
		else		// An error has been sent back in the response byte
		{
#ifdef ENABLE_HOST
			// 	Indicates when an error occurs or block addresses are unreachable - useful for debugging
			UART_sendCString("Block ");
			UART_putByte(ui8FirstBlock);			// Output block number
			UART_sendCString(" Error");
			UART_putNewLine();
			UART_sendCString("ISO15693 Error Code: ");
			UART_putByte(g_pui8TrfBuffer[1]);		// Output ISO15693 error code
			UART_putNewLine();
#endif
			// Response with error
			ui8Status = STATUS_FAIL;
		}
	}
	else
	{
		// No response
		ui8Status = STATUS_FAIL;
	}

	return ui8Status;
}



//*****************************************************************************
//
//! ISO15693_sendGetSystemInfo - Issue the Get System Information command
//! for ISO15693 tags.
//!
//! \param ui8ReqFlag are the request flags for ISO15693 commands.
//!
//! This function issues a Get System Information command for ISO15693 tags.
//! This can be used to determine how many blocks of data can be read from
//! the tag.
//!
//! If UART is enabled, the contents of the Get System Information response
//! is output via UART.
//!
//! \return ui16NumberOfBlocks returns the number of blocks contained
//! in the ISO15693 tag.
//
//*****************************************************************************
uint16_t TRF79xxA_ISO15693_sendGetSystemInfo(uint8_t ui8ReqFlag)
{
	uint8_t ui8Offset = 0;
	uint16_t ui16NumberOfBlocks = 0x00;
	uint8_t ui8RxLength = 0;
#ifdef ENABLE_HOST
	uint8_t ui8LoopCount = 1;
#endif

	g_pui8TrfBuffer[ui8Offset++] = 0x8F;		// Reset FIFO
	g_pui8TrfBuffer[ui8Offset++] = 0x91;		// Send with CRC
	g_pui8TrfBuffer[ui8Offset++] = 0x3D;		// Write Continuous
	g_pui8TrfBuffer[ui8Offset++] = 0x00;		// Length of packet in bytes - upper and middle nibbles of transmit byte length
	if (ui8ReqFlag & 0x20)
		g_pui8TrfBuffer[ui8Offset++] = 0xA0;		// Length of packet in bytes - lower and broken nibbles of transmit byte length
	else
		g_pui8TrfBuffer[ui8Offset++] = 0x20;		// Length of packet in bytes - lower and broken nibbles of transmit byte length

	g_pui8TrfBuffer[ui8Offset++] = ui8ReqFlag | 0x08;		// ISO15693 flags: 0x02 with extension flag
	g_pui8TrfBuffer[ui8Offset++] = 0x2B;					// Get System Information command code

	if (ui8ReqFlag & 0x20)
	{
		g_pui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[0];	// Tag UID
		g_pui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[1];	// Tag UID
		g_pui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[2];	// Tag UID
		g_pui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[3];	// Tag UID
		g_pui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[4];	// Tag UID
		g_pui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[5];	// Tag UID
		g_pui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[6];	// Tag UID
		g_pui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[7];	// Tag UID
	}

	TRF79xxA_writeRaw(&g_pui8TrfBuffer[0], ui8Offset);		// Issue the Get System Information command

	g_sTrfStatus = TRF79xxA_waitRxData(10,30);	// 10 millisecond TX timeout, 30 millisecond RX timeout

	if (g_sTrfStatus == RX_COMPLETE)		// If data has been received
	{
		if (g_pui8TrfBuffer[0] == 0x00)		// Confirm "no error" in response flags byte
		{
			ui8RxLength = TRF79xxA_getRxBytesReceived();

#ifdef ENABLE_HOST
			// Output received data to UART
			UART_sendCString("Get Sys Info Data: ");
			UART_putChar('[');

			for (ui8LoopCount = 1; ui8LoopCount < ui8RxLength; ui8LoopCount++)
			{
				UART_putByte(g_pui8TrfBuffer[ui8LoopCount]);		// Send Get System Info data to host
			}

			UART_putChar(']');
			UART_putNewLine();
#endif

			// Check to see that no error flags were sent and that there is a block number data available
			if (g_pui8TrfBuffer[0] == 0x00 && ((g_pui8TrfBuffer[1] & 0x07) == 0x07))
			{
				ui16NumberOfBlocks = g_pui8TrfBuffer[12];
			}
			else if (g_pui8TrfBuffer[0] == 0x00 && (((g_pui8TrfBuffer[1] & 0x07) == 0x06) || ((g_pui8TrfBuffer[1] & 0x07) == 0x05)))
			{
				ui16NumberOfBlocks = g_pui8TrfBuffer[11];
			}
			else if (g_pui8TrfBuffer[0] == 0x00 && ((g_pui8TrfBuffer[1] & 0x07) == 0x04))
			{
				ui16NumberOfBlocks = g_pui8TrfBuffer[10];
			}
		}
	}
	else if ((g_sTrfStatus == NO_RESPONSE_RECEIVED) || (g_sTrfStatus == NO_RESPONSE_RECEIVED_15693))
	{
		// Case for TI HF-I Pro and Standard tags
		ui16NumberOfBlocks = 0x0A;
	}

	return ui16NumberOfBlocks;
}

//*****************************************************************************
//
//! ISO15693_sendGetSystemInfoExtended - Issue the Get System Information
//! command for ISO15693 tags with the protocol extention flag set.
//!
//! \param ui8ReqFlag are the request flags for ISO15693 commands.
//!
//! This function issues a Get System Information command for ISO15693 tags
//! with the Protocol Extension bit set in the request flags. This can be
//! used to determine how many blocks of data can be read from the tag.
//!
//! If UART is enabled, the contents of the Get System Information
//! response is output via UART.
//!
//! \return ui16NumberOfBlocks returns the number of blocks contained in
//! the ISO15693 tag.
//
//*****************************************************************************
uint16_t TRF79xxA_ISO15693_sendGetSystemInfoExtended(uint8_t ui8ReqFlag)
{
	uint8_t ui8Offset = 0;
	uint16_t ui16NumberOfBlocks = 0x00;
#ifdef ENABLE_HOST
	uint8_t ui8LoopCount = 1;
	uint8_t ui8RxLength = 0;
#endif

	g_pui8TrfBuffer[ui8Offset++] = 0x8F;		// Reset FIFO
	g_pui8TrfBuffer[ui8Offset++] = 0x91;		// Send with CRC
	g_pui8TrfBuffer[ui8Offset++] = 0x3D;		// Write Continuous
	g_pui8TrfBuffer[ui8Offset++] = 0x00;		// Length of packet in bytes - upper and middle nibbles of transmit byte length
	if (ui8ReqFlag & 0x20)
	{
		g_pui8TrfBuffer[ui8Offset++] = 0xA0;		// Length of packet in bytes - lower and broken nibbles of transmit byte length
	}
	else
	{
		g_pui8TrfBuffer[ui8Offset++] = 0x20;		// Length of packet in bytes - lower and broken nibbles of transmit byte length
	}
	g_pui8TrfBuffer[ui8Offset++] = ui8ReqFlag | 0x08;		// ISO15693 flags + protocol extension bit
	g_pui8TrfBuffer[ui8Offset++] = 0x2B;			// Get System Information command code

	if (ui8ReqFlag & 0x20)
	{
		g_pui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[0];	// Tag UID
		g_pui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[1];	// Tag UID
		g_pui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[2];	// Tag UID
		g_pui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[3];	// Tag UID
		g_pui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[4];	// Tag UID
		g_pui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[5];	// Tag UID
		g_pui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[6];	// Tag UID
		g_pui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[7];	// Tag UID
	}

	TRF79xxA_writeRaw(&g_pui8TrfBuffer[0], ui8Offset);		// Issue the Get System Information command

	g_sTrfStatus = TRF79xxA_waitRxData(10,30);	// 10 millisecond TX timeout, 30 millisecond RX timeout

	if (g_sTrfStatus == RX_COMPLETE)		// If data has been received
	{
		if (g_pui8TrfBuffer[0] == 0x00)		// Confirm "no error" in response flags byte
		{
#ifdef ENABLE_HOST
			UART_sendCString("Get Sys Info Data: ");
			UART_putChar('[');

			ui8RxLength = TRF79xxA_getRxBytesReceived();

			// Output received data to UART
			for (ui8LoopCount = 1; ui8LoopCount < ui8RxLength; ui8LoopCount++)
			{
				UART_putByte(g_pui8TrfBuffer[ui8LoopCount]);		// Send Get System Info data to host
			}

			UART_putChar(']');
			UART_putNewLine();
#endif
			// Check to see that no error flags were sent and that there is a block number data available

			if (g_pui8TrfBuffer[0] == 0x00 && ((g_pui8TrfBuffer[1] & 0x07) == 0x07))
			{
				ui16NumberOfBlocks = ((g_pui8TrfBuffer[13] << 8) | (g_pui8TrfBuffer[12])) ;
			}
			else if (g_pui8TrfBuffer[0] == 0x00 && (((g_pui8TrfBuffer[1] & 0x07) == 0x06) || ((g_pui8TrfBuffer[1] & 0x07) == 0x05)))
			{
				ui16NumberOfBlocks = ((g_pui8TrfBuffer[12] << 8) | (g_pui8TrfBuffer[11])) ;
			}
			else if (g_pui8TrfBuffer[0] == 0x00 && ((g_pui8TrfBuffer[1] & 0x07) == 0x04))
			{
				ui16NumberOfBlocks = ((g_pui8TrfBuffer[11] << 8) | (g_pui8TrfBuffer[10])) ;
			}
		}
	}

	return ui16NumberOfBlocks;
}

//*****************************************************************************
//
//! ISO15693_sendReadSingleBlock - Issue the Read Single Block command
//! for ISO15693 tags.
//!
//! \param ui8ReqFlag are the request flags for ISO15693 commands.
//! \param ui8BlockNumber is the block number to read data from.
//!
//! This function issues a Read Single Block with the specified request flags
//! and block number to read data from.
//!
//! If UART is enabled, the data read from the ISO15693 tag is output via UART.
//!
//! \return ui8Status returns either STATUS_SUCCESS or STATUS_FAIL
//! to indicate if the Read Single Block was successful or not.
//
//*****************************************************************************
uint8_t TRF79xxA_ISO15693_sendReadSingleBlock(uint8_t ui8ReqFlag, uint8_t ui8BlockNumber)
{
	uint8_t ui8Offset = 0;
	uint8_t ui8Status = STATUS_FAIL;
#ifdef ENABLE_HOST
	uint8_t ui8LoopCount = 1;
	uint8_t ui8RxLength = 0;
#endif

	/* Prepare Request Command */
	g_pui8TrfBuffer[ui8Offset++] = 0x8F;		// Reset FIFO
	g_pui8TrfBuffer[ui8Offset++] = 0x91;		// Send with CRC
	g_pui8TrfBuffer[ui8Offset++] = 0x3D;		// Write Continuous
	g_pui8TrfBuffer[ui8Offset++] = 0x00;		// Length of packet in bytes - upper and middle nibbles of transmit byte length
	if (ui8ReqFlag & 0x20)
		g_pui8TrfBuffer[ui8Offset++] = 0xB0;		// Length of packet in bytes - lower and broken nibbles of transmit byte length
	else
		g_pui8TrfBuffer[ui8Offset++] = 0x30;		// Length of packet in bytes - lower and broken nibbles of transmit byte length

	g_pui8TrfBuffer[ui8Offset++] = ui8ReqFlag;		// ISO15693 flags
	g_pui8TrfBuffer[ui8Offset++] = 0x20;			// Read Single Block command code

	if (ui8ReqFlag & 0x20) {
		g_pui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[0];	// Tag UID
		g_pui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[1];	// Tag UID
		g_pui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[2];	// Tag UID
		g_pui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[3];	// Tag UID
		g_pui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[4];	// Tag UID
		g_pui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[5];	// Tag UID
		g_pui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[6];	// Tag UID
		g_pui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[7];	// Tag UID
	}

	g_pui8TrfBuffer[ui8Offset++] = ui8BlockNumber;		// Block # (variable, for HF-I Plus device can go to 0x3F, Pro and Standard handled with "error" response flags)

	TRF79xxA_writeRaw(&g_pui8TrfBuffer[0], ui8Offset);		// Issue the Get System Information command

	g_sTrfStatus = TRF79xxA_waitRxData(5,15);		// 5 millisecond TX timeout, 15 millisecond RX timeout

	if (g_sTrfStatus == RX_COMPLETE)		// If data has been received
	{
		if (g_pui8TrfBuffer[0] == 0x00)		// Confirm "no error" in response flags byte
		{
#ifdef ENABLE_HOST
			UART_sendCString("Block ");
			UART_putByte(ui8BlockNumber);		// Output block number
			UART_sendCString(" Data: ");
			UART_putChar('[');

			ui8RxLength = TRF79xxA_getRxBytesReceived();

			if (ui8ReqFlag & BIT6) // Handle case for Option Flag causing one extra byte to be transmitted.
			{
				ui8Offset = 2;
			}
			else
			{
				ui8Offset = 1;
			}

			// Output received data to UART
			for (ui8LoopCount = ui8Offset; ui8LoopCount < ui8RxLength; ui8LoopCount++)
			{
				UART_putByte(g_pui8TrfBuffer[ui8LoopCount]);		// Send out data read from tag to host
			}

			UART_putChar(']');
			UART_putNewLine();
#endif
			// Response received
			ui8Status = STATUS_SUCCESS;
		}
		else		// An error has been sent back in the response byte
		{
#ifdef ENABLE_HOST
			// 	Indicates when an error occurs or block addresses are unreachable - useful for debugging
			UART_sendCString("Block ");
			UART_putByte(ui8BlockNumber);			// Output block number
			UART_sendCString(" Error");
			UART_putNewLine();
			UART_sendCString("ISO15693 Error Code: ");
			UART_putByte(g_pui8TrfBuffer[1]);		// Output ISO15693 error code
			UART_putNewLine();
#endif
			// Response with error
			ui8Status = STATUS_FAIL;
		}
	}
	else
	{
		// No response
		ui8Status = STATUS_FAIL;
	}

	return ui8Status;
}

//*****************************************************************************
//
//! ISO15693_sendReadSingleBlockExtended - Issue the Read Single Block
//! command for ISO15693 tags with the protocol extention flag set
//!
//! \param ui8ReqFlag are the request flags for ISO15693 commands.
//! \param ui16BlockNumber is the block number to read data from.
//!
//! This function issues a Read Single Block with the block number and the
//! specified request flags, including the Protocol Extension bit, to read
//! data from ISO15693 tags which require the use of extended protocol
//! commands.
//!
//! If UART is enabled, the data read from the ISO15693 tag is output via UART.
//!
//! \return ui8Status returns either STATUS_SUCCESS or STATUS_FAIL
//! to indicate if the Read Single Block was successful or not.
//
//*****************************************************************************
uint8_t TRF79xxA_ISO15693_sendReadSingleBlockExtended(uint8_t ui8ReqFlag, uint16_t ui16BlockNumber, uint8_t* data_buff)
{
	uint8_t ui8Offset = 0;
	uint8_t ui8Status = STATUS_FAIL;
#ifdef ENABLE_HOST
	uint8_t ui8LoopCount = 1;
	uint8_t ui8RxLength = 0;
#endif

	g_pui8TrfBuffer[ui8Offset++] = 0x8F;		// Reset FIFO
	g_pui8TrfBuffer[ui8Offset++] = 0x91;		// Send with CRC
	g_pui8TrfBuffer[ui8Offset++] = 0x3D;		// Write Continuous
	g_pui8TrfBuffer[ui8Offset++] = 0x00;		// Length of packet in bytes - upper and middle nibbles of transmit byte length
	if (ui8ReqFlag & 0x20)
	{
		g_pui8TrfBuffer[ui8Offset++] = 0xC0;		// Length of packet in bytes - lower and broken nibbles of transmit byte length
	}
	else
	{
		g_pui8TrfBuffer[ui8Offset++] = 0x40;		// Length of packet in bytes - lower and broken nibbles of transmit byte length
	}
	g_pui8TrfBuffer[ui8Offset++] = ui8ReqFlag | 0x08;	// ISO15693 flags with protocol extension bit set
	g_pui8TrfBuffer[ui8Offset++] = 0x20;				// Read Single Block command code

	if (ui8ReqFlag & 0x20)
	{
		g_pui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[0];	// Tag UID
		g_pui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[1];	// Tag UID
		g_pui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[2];	// Tag UID
		g_pui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[3];	// Tag UID
		g_pui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[4];	// Tag UID
		g_pui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[5];	// Tag UID
		g_pui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[6];	// Tag UID
		g_pui8TrfBuffer[ui8Offset++] = g_pui8Iso15693UId[7];	// Tag UID
	}

	g_pui8TrfBuffer[ui8Offset++] = (uint8_t) (ui16BlockNumber & 0xFF);				// Block # (variable, for this device it can go to 0xFF)
	g_pui8TrfBuffer[ui8Offset++] = (uint8_t) ((ui16BlockNumber >> 8) & 0xFF);		// Block # (variable, for this device it can go to 0x07)

	TRF79xxA_writeRaw(&g_pui8TrfBuffer[0], ui8Offset);		// Issue the Read Single Block command

	g_sTrfStatus = TRF79xxA_waitRxData(5,15);		// 5 millisecond TX timeout, 15 millisecond RX timeout

	if (g_sTrfStatus == RX_COMPLETE)		// If data has been received
	{
		if (g_pui8TrfBuffer[0] == 0x00)		// Confirm "no error" in response flags byte
		{
			// Response received
			ui8Status = STATUS_SUCCESS;

			// Results are in g_pui8TrfBuffer[1] to g_pui8TrfBuffer[4]
			*data_buff 		= g_pui8TrfBuffer[1];
			*(data_buff+1) 	= g_pui8TrfBuffer[2];
			*(data_buff+2) 	= g_pui8TrfBuffer[3];
			*(data_buff+3) 	= g_pui8TrfBuffer[4];
		}
		else
		{
			// Received an error from the tag
			ui8Status = STATUS_FAIL;
		}
	}
	else
	{
		// Did not receive a proper response from tag
		ui8Status = STATUS_FAIL;
	}

	return ui8Status;
}

