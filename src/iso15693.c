#include "time.h"
#include "trf7970a.h"
#include "iso15693.h"

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
uint8_t CustomReadTag(uint8_t ui8ReqFlag, uint8_t* dataBuf) {
	uint16_t ui16ReadBlocks = 0x00;
	uint16_t i = 0x00;

	ui8ReqFlag |= 0x08; 	// Add in Protocol Extension Flag if it was omitted from the inputted request flags

	ui16ReadBlocks = ISO15693_GetSystemInfo(ui8ReqFlag);	// Issue a Get System Info with Protocol Extension

	if (ui16ReadBlocks != 0x00)
	{
		// Read 8 blocks on the ISO15693 Tag
		for (i = 0; i < 9; i++) {
			// Keep reading blocks until a No Response is received
			if (ISO15693_ReadSingleBlock(ui8ReqFlag, i, dataBuf + 4*i) == STATUS_FAIL) {
				return STATUS_FAIL;
			}
		}
		return STATUS_SUCCESS;
	}
	return STATUS_FAIL;
}



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
void ISO15693_init(void)
{
	uint8_t ui8LoopCount;

	g_ui8TagDetectedCount = 0;
	g_ui8RecursionCount = 0;

	// Initialize with invalid UID
	for (ui8LoopCount = 0; ui8LoopCount < 8; ui8LoopCount++) {
		g_pui8Iso15693UId[ui8LoopCount] = 0xFF;
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
uint8_t ISO15693_Inventory()
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
//! ISO15693_GetSystemInfo - Issue the Get System Information
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
uint16_t ISO15693_GetSystemInfo(uint8_t ui8ReqFlag)
{
	uint8_t ui8Offset = 0;
	uint16_t ui16NumberOfBlocks = 0x00;
	uint8_t ui8LoopCount = 1;
	uint8_t ui8RxLength = 0;

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
			ui8RxLength = TRF79xxA_getRxBytesReceived();

			for (ui8LoopCount = 1; ui8LoopCount < ui8RxLength; ui8LoopCount++)
			{
				// Contains System Info data to host
			}

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
//! ISO15693_ReadSingleBlock - Issue the Read Single Block
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
uint8_t ISO15693_ReadSingleBlock(uint8_t ui8ReqFlag, uint16_t ui16BlockNumber, uint8_t* data_buff)
{
	uint8_t ui8Offset = 0;
	uint8_t ui8Status = STATUS_FAIL;
	uint8_t ui8RxLength = 0;

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
			ui8RxLength = TRF79xxA_getRxBytesReceived();

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



//*****************************************************************************
//
//! ISO15693_WriteSingleBlock - Issue the Write Single Block command
//! for ISO15693 tags.
//!
//! \param ui8ReqFlag are the request flags for ISO15693 commands.
//! \param ui8BlockNumber is the block number to write data to.
//! \param ui8BlockSize is the tag block size.
//! \param pui8BlockData is the data to be written.
//!
//! Function issues an addressed Write Single Block with the specified request
//! flags as well as the address flag. The write single block command writes
//! the provided data to the addressed ISO15693 tag.
//!
//! The function will always add the address flag to the command as a good
//! practice to avoid overwriting other ISO15693 tags in the vicinity.
//!
//! This function natively supports writing to tags with more than 4 bytes of
//! data per block through the ui8BlockSize variable.
//!
//! \return ui8Status returns either STATUS_SUCCESS or STATUS_FAIL to
//! indicate if the Write Single Block was successful or not.
//
//*****************************************************************************

uint8_t ISO15693_WriteSingleBlock(uint8_t ui8ReqFlag, uint16_t ui16BlockNumber, uint8_t * pui8BlockData)
{
	uint8_t ui8Offset = 0;
	uint8_t ui8Status = STATUS_FAIL;
	uint8_t ui8LoopCount = 0;
	uint8_t ui8BlockSize = 4;

	// Command to Reader board
	g_pui8TrfBuffer[ui8Offset++] = 0x8F;		// Reset FIFO
	g_pui8TrfBuffer[ui8Offset++] = 0x91;		// Send with CRC
	g_pui8TrfBuffer[ui8Offset++] = 0x3D;		// Write Continuous
	if (ui8ReqFlag & 0x20) {
		g_pui8TrfBuffer[ui8Offset++] = (((0x0B + ui8BlockSize) & 0xF0) >> 0x04);	// Length of packet in bytes - upper and middle nibbles of transmit byte length
		g_pui8TrfBuffer[ui8Offset++] = ((0x0B + ui8BlockSize) << 0x04);			// Length of packet in bytes - lower and broken nibbles of transmit byte length
	}
	else {
		g_pui8TrfBuffer[ui8Offset++] = (((0x04 + ui8BlockSize) & 0xF0) >> 0x04);	// Length of packet in bytes - upper and middle nibbles of transmit byte length
		g_pui8TrfBuffer[ui8Offset++] = ((0x04 + ui8BlockSize) << 0x04);			// Length of packet in bytes - lower and broken nibbles of transmit byte length
	}

	// Content to send out
	g_pui8TrfBuffer[ui8Offset++] = ui8ReqFlag;										// ISO15693 flags
	g_pui8TrfBuffer[ui8Offset++] = 0x21;											// Write Single Block command code
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
	for (ui8LoopCount = 0; ui8LoopCount < ui8BlockSize; ui8LoopCount++) {
		g_pui8TrfBuffer[ui8Offset++] = pui8BlockData[ui8LoopCount];					// Data to write to tag
	}

	TRF79xxA_writeRaw(&g_pui8TrfBuffer[0], ui8Offset);		// Issue the Get System Information command

	g_sTrfStatus = TRF79xxA_getTrfStatus();

	// Special handling to cover option flag use case for TI Tag-It HF-I ISO15693 transponders
	if (g_sTrfStatus == TRF_IDLE || g_sTrfStatus == TX_WAIT)
	{
		// Check if the option flag is set
		if (ui8ReqFlag & 0x40)
		{
			TRF79xxA_waitTxIRQ(10);	// 10 millisecond TX timeout

			g_sTrfStatus = TRF79xxA_getTrfStatus();

			if (g_sTrfStatus == TX_COMPLETE)	// If transmit is complete
			{
				delayMilliSeconds(10);
				TRF79xxA_sendDirectCommand(TRF79XXA_TRANSMIT_NEXT_SLOT_CMD);		// Send out End of Frame marker
				TRF79xxA_waitRxIRQ(30);				// 30 millisecond RX timeout
			}
			else								// Otherwise return an error
			{
				return ui8Status = STATUS_FAIL;
			}
		}
		else
		{
			TRF79xxA_waitTxIRQ(10);	// 10 millisecond TX timeout
			TRF79xxA_waitRxIRQ(30);	// 30 millisecond RX timeout
		}
	}
	else if (g_sTrfStatus == TX_COMPLETE)
	{
		// Check if the option flag is set
		if (ui8ReqFlag & 0x40)
		{
			delayMilliSeconds(10);
			TRF79xxA_sendDirectCommand(TRF79XXA_TRANSMIT_NEXT_SLOT_CMD);		// Send out End of Frame marker
		}

		TRF79xxA_waitRxIRQ(30);				// 30 millisecond RX timeout
	}
	else
	{
		return ui8Status = STATUS_FAIL;
	}

	g_sTrfStatus = TRF79xxA_getTrfStatus();

	if (g_sTrfStatus == RX_COMPLETE)		// If data has been received
	{
		if (g_pui8TrfBuffer[0] == 0x00)		// Confirm "no error" in response flags byte
		{
			// Response received
			ui8Status = STATUS_SUCCESS;
		}
		else		// An error has been sent back in the response byte
		{
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
uint8_t ISO15693_runAnticollision(uint8_t ui8ReqFlags, uint8_t ui8MaskLength, uint8_t ui8Afi)
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


				for (ui8LoopCount1 = 0; ui8LoopCount1 < 8; ui8LoopCount1++)
				{
					// Contains UID data
				}

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
			ui8Status = ISO15693_runAnticollision(ui8ReqFlags, ui8MaskLength, ui8Afi);
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

