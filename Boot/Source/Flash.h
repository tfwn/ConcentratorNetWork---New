/************************************************************************************************
*                                   SRWF-6009-BOOT
*    (c) Copyright 2015, Software Department, Sunray Technology Co.Ltd
*                               All Rights Reserved
*
* FileName     : Flash.h
* Description  :
* Version      :
* Function List:
*------------------------------Revision History--------------------------------------------------
* No.   Version     Date            Revised By      Item            Description
* 1     V1.0        10/08/2015      Zhangxp         SRWF-6009-BOOT  Original Version
************************************************************************************************/
#ifndef _FLASH_H_
#define _FLASH_H_


/************************************************************************************************
*                               Macro Define Section
************************************************************************************************/
#define FLASH_PAGE_SIZE                 ((uint16)0x800)     // FLASH每一页的大小2K

#define FLASH_APPCODE_START_ADDR        ((uint32)0x08004000)// APP代码运行的开始位置,96K

#define FLASH_PARAM_START_ADDRESS       ((uint32)0x0801C000)// FLASH保存工作参数的区域,2K
#define FLASH_PARAM_PAGE_SIZE           1

#define FLASH_NODE_START_ADDRESS        (FLASH_PARAM_START_ADDRESS + FLASH_PAGE_SIZE * FLASH_PARAM_PAGE_SIZE)    // FLASH保存档案参数的区域,16页
#define FLASH_NODE_PAGE_SIZE            16                  // FLASH保存档案参数的区域,16页,32K

#define FLASH_UPGRADECODE_START_ADDR	((uint32)0x08027000)// 升级文件存放的开始位置
#define FLASH_UPGRADECODE_SIZE          48                  // 升级文件,最大为96K,每一页为2K

#define FLASH_UPGRADE_INFO_START        ((uint32)0x0807F800)// 升级文件参数保存位置,510K开始,占据2K空间
#define FLASH_UPGRADE_INFO_SIZE         1                   // 升级文件参数保存位置大小


/************************************************************************************************
*                                   Enum Define Section
************************************************************************************************/

/************************************************************************************************
*                                   Struct Define Section
************************************************************************************************/

/************************************************************************************************
*                                   Function Declare Section
************************************************************************************************/
extern void Flash_Write(uint8 * Tp, uint16 Num, uint32 Flash_Addr);
extern void Flash_Erase(uint32 StartAddr, uint8 PageCount);


#endif
/**************************************End of file**********************************************/


