/************************************************************************************************
*                                   SRWF-6009
*    (c) Copyright 2015, Software Department, Sunray Technology Co.Ltd
*                               All Rights Reserved
*
* FileName     : Flash.h
* Description  :
* Version      :
* Function List:
*------------------------------Revision History--------------------------------------------------
* No.   Version     Date            Revised By      Item            Description
* 1     V1.0        08/05/2015      Zhangxp         SRWF-6009       Original Version
************************************************************************************************/

#ifndef  FLASH_H
#define  FLASH_H

#ifdef   FLASH_GLOBALS
#define  FLASH_EXT
#else
#define  FLASH_EXT  extern
#endif

/************************************************************************************************
*                               Pubilc Macro Define Section
************************************************************************************************/
#define FLASH_PAGE_SIZE                         ((uint16)0x800)         // FLASH每一页的大小2K

#define FLASH_BOOT_SIZE                         ((uint16)0x4000)        // BOOT代码长度16K,从0K开始,最大占据16K空间

#define FLASH_APPCODE_START_ADDR                ((uint32)0x08004000)    // APP代码运行的开始位置,16K开始  0x08004000

#define FLASH_CONCENTRATOR_INFO_ADDRESS         ((uint32)0x0803E800)    // 集中器参数的区域,从250K开始,占据空间2K
#define FLASH_BACKUP_CONCENTRATOR_INFO_ADDRESS  ((uint32)0x0803F000)    // 集中器参数备份区域,从252K开始，占据空间2K

#define FLASH_NODE_INFO_ADDRESS                 ((uint32)0x08040000)    // 表节点参数区域,从256K开始,占据16K空间
#define FLASH_NODE_INFO_PAGE_SIZE               8                       // FLASH保存节点路径区域,8页,16K

#define FLASH_CUSTOM_ROUTE_ADDRESS1             ((uint32)0x0804B000)    // 自定义路径存储区1,从300K开始,占据22K空间
#define FLASH_CUSTOM_ROUTE_ADDRESS2             ((uint32)0x08050800)    // 自定义路径存储区2,从322K开始,占据22K空间
#define FLASH_CUSTOM_ROUTE_PAGE_SIZE            11                      // 自定义路径存储区占据空间,22K

#define FLASH_UPGRADECODE_START_ADDR            ((uint32)0x08060000)    // 升级文件存放的开始位置,384K开始,最大占据100K空间
#define FLASH_UPGRADECODE_SIZE                  50                      // 升级文件,最大为100K,每一页为2K

#define FLASH_UPGRADE_INFO_START                ((uint32)0x0807F800)    // 升级文件参数保存位置,510K开始,占据2K空间
#define FLASH_UPGRADE_INFO_SIZE                 1                       // 升级文件参数保存位置大小


#define FLASH_START_MEMORYID                    0x0218

/************************************************************************************************
*                                  Struct Define Section
************************************************************************************************/

/************************************************************************************************
*                                   Enum Define Section
************************************************************************************************/

/************************************************************************************************
*                            Variable Declare Section
************************************************************************************************/

/************************************************************************************************
*                            Function Declare Section
************************************************************************************************/
FLASH_EXT void Flash_LoadConcentratorInfo(void);
FLASH_EXT void Flash_SaveConcentratorInfo(void);
FLASH_EXT void Flash_LoadSubNodesInfo(void);
FLASH_EXT void Flash_SaveSubNodesInfo(void);

FLASH_EXT void Flash_Erase(uint32 StartAddr, uint8 PageCount);
FLASH_EXT void Flash_Read(uint8 *BufPtr, uint16 Num, uint32 Flash_Addr);
FLASH_EXT void Flash_Write(uint8 *BufPtr, uint16 Num, uint32 FlashAddr);


#endif
/***************************************End of file*********************************************/

