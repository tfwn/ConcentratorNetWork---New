/************************************************************************************************
*                                   SRWF-6009
*    (c) Copyright 2015, Software Department, Sunray Technology Co.Ltd
*                               All Rights Reserved
*
* FileName     : Eeprom.h
* Description  :
* Version      :
* Function List:
*------------------------------Revision History--------------------------------------------------
* No.   Version     Date            Revised By      Item            Description
* 1     V1.0        08/03/2015      Zhangxp         SRWF-6009       Original Version
************************************************************************************************/

#ifndef  EEPROM_H
#define  EEPROM_H


#ifdef   EEPROM_GLOBALS
#define  EEPROM_EXT
#else
#define  EEPROM_EXT  extern
#endif

/************************************************************************************************
*                               Pubilc Macro Define Section
************************************************************************************************/
#define I2C1_DR_Address                         ((uint32_t)0x40005410)

#ifdef EEPROM_SEL_M01
#define I2C_SPEED                               400000
#define EEPROM_DEVADDR                          0xA4
#define EEPROM_PAGESIZE                         256
#define EEPROM_TOTALSIZE                        0x20000
#endif

#ifdef EEPROM_SEL_M512
#define I2C_SPEED                               100000
#define EEPROM_DEVADDR                          0xA4
#define EEPROM_PAGESIZE                         128
#define EEPROM_TOTALSIZE                        0x10000
#endif

#define EEPROM_FLAG_TIMEOUT                     5000
#define EEPROM_LONG_TIMEOUT                     50000

/************************************************************************************************
*                                   Enum Define Section
************************************************************************************************/

/************************************************************************************************
*                                  Struct Define Section
************************************************************************************************/

/************************************************************************************************
*                                 Variable Declare Section
************************************************************************************************/
EEPROM_EXT OS_EVENT *EepromAccessMutex;

/************************************************************************************************
*                            Function Declare Section
************************************************************************************************/
EEPROM_EXT void Eeprom_Init(void);
EEPROM_EXT bool Eeprom_ReadWrite(uint8 *BufPtr, uint32 SubAddr, uint16 Count, uint8 AccessMode);

#endif
/***************************************End of file*********************************************/

