/************************************************************************************************
*                                   SRWF-6009
*    (c) Copyright 2015, Software Department, Sunray Technology Co.Ltd
*                               All Rights Reserved
*
* FileName     : Flash.c
* Description  :
* Version      :
* Function List:
*------------------------------Revision History--------------------------------------------------
* No.   Version     Date            Revised By      Item            Description
* 1     V1.0        08/05/2015      Zhangxp         SRWF-6009       Original Version
************************************************************************************************/

#define FLASH_GLOBALS

/************************************************************************************************
*                             Include File Section
************************************************************************************************/
#include "Stm32f10x_conf.h"
#include "ucos_ii.h"
#include "Flash.h"
#include "DataHandle.h"
#include "Database.h"
#include <string.h>

/************************************************************************************************
*                        Global Variable Declare Section
************************************************************************************************/
// 集中器默认参数值
const uint8 Default_LongAdr[] = {0x00, 0x00, 0x00, 0x00, 0x60, 0x09};
const uint8 Default_DscIp[] = {114, 80, 252, 3};
const uint16 Default_DscPort = 13289;
const uint8 Default_BackupDscIp[] = {210, 22, 152, 150};
const uint16 Default_BackupDscPort = 13289;
const char *Default_Apn = "";
const char *Default_Username = "";
const char *Default_Password = "";
const uint8 Default_HeatBeatPeriod = 12;

/************************************************************************************************
*                           Prototype Declare Section
************************************************************************************************/

/************************************************************************************************
*                           Function Declare Section
************************************************************************************************/

/************************************************************************************************
* Function Name: Flash_LoadConcentratorInfo
* Decription   : Flash读取集中器参数信息
* Input        : 无
* Output       : 无
* Others       : 无
************************************************************************************************/
void Flash_LoadConcentratorInfo(void)
{
    uint8 i;
    CONCENTRATOR_INFO *infoPtr;

    // 从数据存储区中读出数据
    infoPtr = (CONCENTRATOR_INFO *)FLASH_CONCENTRATOR_INFO_ADDRESS;
    if (infoPtr->Fcs == CalCrc16(infoPtr->LongAddr, LONG_ADDR_SIZE)) {
        memcpy(&Concentrator, infoPtr, sizeof(CONCENTRATOR_INFO));
        return;
    }

    // 从备份数据存储区中读出数据
    infoPtr = (CONCENTRATOR_INFO *)FLASH_BACKUP_CONCENTRATOR_INFO_ADDRESS;
    if (infoPtr->Fcs == CalCrc16(infoPtr->LongAddr, LONG_ADDR_SIZE)) {
        memcpy(&Concentrator, infoPtr, sizeof(CONCENTRATOR_INFO));
        return;
    }

    // 装载默认数据
    memcpy(Concentrator.LongAddr, Default_LongAdr, LONG_ADDR_SIZE);
    Concentrator.MaxNodeId = 0;
    Concentrator.CustomRouteSaveRegion = 0;

    Concentrator.Param.WorkType = RealTimeDataMode;
    Concentrator.Param.DataReplenishCtrl = 0;
    Concentrator.Param.DataUploadCtrl = 0;
    Concentrator.Param.DataUploadMode = 0;
    Concentrator.Param.DataEncryptCtrl = 0;
    Concentrator.Param.DataUploadTime = 0x22;
    for (i = 0; i < 4; i++) {
        Concentrator.Param.DataReplenishDay[i] = 0x00;
    }
    Concentrator.Param.DataReplenishHour = 20;
    Concentrator.Param.DataReplenishCount = 2;

    memcpy(Concentrator.GprsParam.PriorDscIp, Default_DscIp, 4);
    Concentrator.GprsParam.PriorDscPort = Default_DscPort;
    memcpy(Concentrator.GprsParam.BackupDscIp, Default_BackupDscIp, 4);
    Concentrator.GprsParam.BackupDscPort = Default_BackupDscPort;
    strcpy(Concentrator.GprsParam.Apn, Default_Apn);
    strcpy(Concentrator.GprsParam.Username, Default_Username);
    strcpy(Concentrator.GprsParam.Password, Default_Password);
    Concentrator.GprsParam.HeatBeatPeriod = Default_HeatBeatPeriod;
	Concentrator.SecondChannel = 0x3;
	Concentrator.SaveSecondChannel = 0x3;
}

/************************************************************************************************
* Function Name: Flash_SaveConcentratorInfo
* Decription   : Flash保存集中器参数信息
* Input        : 无
* Output       : 无
* Others       : 无
************************************************************************************************/
void Flash_SaveConcentratorInfo(void)
{
    // 如果与当前存储区数据一致,则不需要保存
    if (0 == memcmp((uint8 *)FLASH_CONCENTRATOR_INFO_ADDRESS, (uint8 *)(&Concentrator), sizeof(CONCENTRATOR_INFO))) {
        return;
    }

    // 擦除备份区
    OSSchedLock();
    Flash_Erase(FLASH_BACKUP_CONCENTRATOR_INFO_ADDRESS, 1);
    OSSchedUnlock();

    // 将当前数据写入备份区
    OSSchedLock();
    Flash_Write((uint8 *)FLASH_CONCENTRATOR_INFO_ADDRESS, sizeof(CONCENTRATOR_INFO), FLASH_BACKUP_CONCENTRATOR_INFO_ADDRESS);
    OSSchedUnlock();

    // 擦除当前存储区
    OSSchedLock();
    Flash_Erase(FLASH_CONCENTRATOR_INFO_ADDRESS, 1);
    OSSchedUnlock();

    // 将新数据写入存储区
    Concentrator.Fcs = CalCrc16(Concentrator.LongAddr, LONG_ADDR_SIZE);
    OSSchedLock();
    Flash_Write((uint8 *)(&Concentrator), sizeof(CONCENTRATOR_INFO), FLASH_CONCENTRATOR_INFO_ADDRESS);
    OSSchedUnlock();
}

/************************************************************************************************
* Function Name: Flash_LoadSubNodesInfo
* Decription   : Flash读取节点参数信息
* Input        : 无
* Output       : 无
* Others       : 无
************************************************************************************************/
void Flash_LoadSubNodesInfo(void)
{
    uint16 nodeId;

    for (nodeId = 0; nodeId < Concentrator.MaxNodeId && nodeId < MAX_NODE_NUM; nodeId++) {
        memcpy(&SubNodes[nodeId], (uint8 *)(FLASH_NODE_INFO_ADDRESS + nodeId * sizeof(SUBNODE_INFO)), sizeof(SUBNODE_INFO));
        IWDG_ReloadCounter();
    }
}

/************************************************************************************************
* Function Name: Flash_SaveSubNodesInfo
* Decription   : Flash保存节点参数信息
* Input        : 无
* Output       : SUCCESS-成功,ERROR-失败
* Others       : 无
************************************************************************************************/
void Flash_SaveSubNodesInfo(void)
{
    uint16 nodeId;

    OSSchedLock();
    Flash_Erase(FLASH_NODE_INFO_ADDRESS, FLASH_NODE_INFO_PAGE_SIZE);
    for (nodeId = 0; nodeId < Concentrator.MaxNodeId && nodeId < MAX_NODE_NUM; nodeId++) {
        IWDG_ReloadCounter();
        Flash_Write((uint8 *)(&SubNodes[nodeId]), sizeof(SUBNODE_INFO), FLASH_NODE_INFO_ADDRESS + nodeId * sizeof(SUBNODE_INFO));
    }
    OSSchedUnlock();
}

/************************************************************************************************
* Function Name: Flash_Erase
* Decription   : Flash擦除函数
* Input        : StartAddr-开始地址,PageCount-要擦除的页数
* Output       : 无
* Others       : 无
************************************************************************************************/
void Flash_Erase(uint32 StartAddr, uint8 PageCount)
{
    uint8 i;
    FLASH_Status flashStatus;

    // 解锁Flash编程擦除控制器
    FLASH_Unlock();

    // 清除所有标识
    FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    // 擦除Flash页
    flashStatus = FLASH_COMPLETE;
    for (i = 0; i < PageCount && FLASH_COMPLETE == flashStatus; i ++) {
        flashStatus = FLASH_ErasePage(StartAddr + (FLASH_PAGE_SIZE * i));
    }

    FLASH_Lock();
}

/************************************************************************************************
* Function Name: Flash_Read
* Decription   : Flash读函数
* Input        : BufPtr-指向读出数据缓冲区的指针,Num-读出数据的数量,FlashAddr-读Flash的地址
* Output       : 无
* Others       : 无
************************************************************************************************/
void Flash_Read(uint8 *BufPtr, uint16 Num, uint32 Flash_Addr)
{
    uint16 i;

    for (i = 0; i < Num; i++) {
        *(BufPtr + i) = *(uint8 *)(Flash_Addr + i);
    }
}

/************************************************************************************************
* Function Name: Flash_Write
* Decription   : Flash写函数
* Input        : BufPtr-指向写入数据缓冲区的指针,Num-写入数据的数量,FlashAddr-写入Flash的地址
* Output       : 无
* Others       : Num的数值必须为偶数
************************************************************************************************/
void Flash_Write(uint8 *BufPtr, uint16 Num, uint32 FlashAddr)
{
    uint16 i, val;
    uint32 addr;
    FLASH_Status flashStatus;

    if (0 == Num) {
        return;
    }

    flashStatus = FLASH_COMPLETE;

    // 解锁Flash编程擦除控制器
    FLASH_Unlock();

    // 清除所有标识
    FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    addr = FlashAddr;

    for (i = 0; i < Num && FLASH_COMPLETE == flashStatus; i += 2) {
        val = (uint16)(*(BufPtr + 1) << 8 & 0xFF00 | (*BufPtr & 0x00FF));
        flashStatus = FLASH_ProgramHalfWord(addr, val);
        BufPtr += 2;
        addr += 2;
    }
    FLASH_Lock();
}

/***************************************End of file*********************************************/

