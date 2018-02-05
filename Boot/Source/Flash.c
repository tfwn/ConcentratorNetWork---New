/************************************************************************************************
*                                   SRWF-6009-BOOT
*    (c) Copyright 2015, Software Department, Sunray Technology Co.Ltd
*                               All Rights Reserved
*
* FileName     : Flash.c
* Description  :
* Version      :
* Function List:
*------------------------------Revision History--------------------------------------------------
* No.   Version     Date            Revised By      Item            Description
* 1     V1.0        10/08/2015      Zhangxp         SRWF-6009-BOOT  Original Version
************************************************************************************************/

/************************************************************************************************
*                             Include File Section
************************************************************************************************/
#include "Stm32f10x.h"
#include "System_stm32f10x.h"
#include "Flash.h"
#include "Hal.h"


/************************************************************************************************
*                           Global Variable Declare Section
************************************************************************************************/


/************************************************************************************************
*                           Prototype Declare Section
************************************************************************************************/


/************************************************************************************************
*                           Function Declare Section
************************************************************************************************/

/************************************************************************************************
* Function Name: Flash_Read
* Decription   : 读FLASH
* Input        : Tp - 要读出Flash中数据的指针,Num - 读出数据的数量,Flash_Addr - 读出Flash的地址
* Output       : None
* Others       : None
************************************************************************************************/
void Flash_Write(uint8 *Tp, uint16 Num, uint32 Flash_Addr)
{
    uint16 i, val;
    uint32 addr;
    volatile FLASH_Status status;

    status = FLASH_COMPLETE;

    // Unlock the Flash Program Erase controller
    FLASH_Unlock();

    // Clear All pending flags
    FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    addr = Flash_Addr;

    for (i = 0; (i < Num && FLASH_COMPLETE == status); i += 2)
    {
        FEED_WATCHDOG
        val = (uint16)((*(Tp + 1) << 8 & 0xFF00) | (*Tp & 0x00FF));
        status = FLASH_ProgramHalfWord(addr, val);
        Tp += 2;
        addr += 2;
    }

    FLASH_Lock();
}

/************************************************************************************************
* Function Name: Flash_Erase
* Decription   : 擦除FLASH块
* Input        : StartAddr - 要擦除Flash块的起始地址, PageCount - 要擦除的页数
* Output       : None
* Others       : None
************************************************************************************************/
void Flash_Erase(uint32 StartAddr, uint8 PageCount)
{
    uint8 i;
    volatile FLASH_Status status;

    // Unlock the Flash Program Erase controller
    FLASH_Unlock();

    // Clear All pending flags
    FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    // Erase the FLASH pages
    status = FLASH_COMPLETE;
    for (i = 0; i < PageCount && FLASH_COMPLETE == status; i++)
    {
        FEED_WATCHDOG
        status = FLASH_ErasePage(StartAddr + (FLASH_PAGE_SIZE * i));
    }

    FLASH_Lock();
}


/**************************************End of file**********************************************/


