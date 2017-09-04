/*
 * iso15693.h
 */

#ifndef ISO15693_H_
#define ISO15693_H_

#define ISO15693_MAX_RECURSION_COUNT	8

uint8_t g_pui8Iso15693UId[8];				// Initialized as 0xFF
uint8_t g_pui8AnticollisionMaskBuffer[8];	// Initialized as 0x00
uint8_t g_ui8TagDetectedCount;
uint8_t g_ui8RecursionCount;


//===============================================================

uint8_t CustomReadTag(uint8_t ui8ReqFlag, uint8_t* dataBuf);
void ISO15693_init(void);


uint8_t ISO15693_Inventory(void);
uint8_t ISO15693_ReadSingleBlock(uint8_t ui8ReqFlag, uint16_t ui16BlockNumber, uint8_t* data_buff);
uint8_t ISO15693_WriteSingleBlock(uint8_t ui8ReqFlag, uint16_t ui16BlockNumber, uint8_t * pui8BlockData);
uint16_t ISO15693_GetSystemInfo(uint8_t ui8ReqFlag);
uint8_t ISO15693_runAnticollision(uint8_t ui8ReqFlags, uint8_t ui8MaskLength, uint8_t ui8Afi);


#endif /* ISO15693_H_ */
