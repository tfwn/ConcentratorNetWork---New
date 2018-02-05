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
// ������Ĭ�ϲ���ֵ
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
* Decription   : Flash��ȡ������������Ϣ
* Input        : ��
* Output       : ��
* Others       : ��
************************************************************************************************/
void Flash_LoadConcentratorInfo(void)
{
    uint8 i;
    CONCENTRATOR_INFO *infoPtr;

    // �����ݴ洢���ж�������
    infoPtr = (CONCENTRATOR_INFO *)FLASH_CONCENTRATOR_INFO_ADDRESS;
    if (infoPtr->Fcs == CalCrc16(infoPtr->LongAddr, LONG_ADDR_SIZE)) {
        memcpy(&Concentrator, infoPtr, sizeof(CONCENTRATOR_INFO));
        return;
    }

    // �ӱ������ݴ洢���ж�������
    infoPtr = (CONCENTRATOR_INFO *)FLASH_BACKUP_CONCENTRATOR_INFO_ADDRESS;
    if (infoPtr->Fcs == CalCrc16(infoPtr->LongAddr, LONG_ADDR_SIZE)) {
        memcpy(&Concentrator, infoPtr, sizeof(CONCENTRATOR_INFO));
        return;
    }

    // װ��Ĭ������
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
* Decription   : Flash���漯����������Ϣ
* Input        : ��
* Output       : ��
* Others       : ��
************************************************************************************************/
void Flash_SaveConcentratorInfo(void)
{
    // ����뵱ǰ�洢������һ��,����Ҫ����
    if (0 == memcmp((uint8 *)FLASH_CONCENTRATOR_INFO_ADDRESS, (uint8 *)(&Concentrator), sizeof(CONCENTRATOR_INFO))) {
        return;
    }

    // ����������
    OSSchedLock();
    Flash_Erase(FLASH_BACKUP_CONCENTRATOR_INFO_ADDRESS, 1);
    OSSchedUnlock();

    // ����ǰ����д�뱸����
    OSSchedLock();
    Flash_Write((uint8 *)FLASH_CONCENTRATOR_INFO_ADDRESS, sizeof(CONCENTRATOR_INFO), FLASH_BACKUP_CONCENTRATOR_INFO_ADDRESS);
    OSSchedUnlock();

    // ������ǰ�洢��
    OSSchedLock();
    Flash_Erase(FLASH_CONCENTRATOR_INFO_ADDRESS, 1);
    OSSchedUnlock();

    // ��������д��洢��
    Concentrator.Fcs = CalCrc16(Concentrator.LongAddr, LONG_ADDR_SIZE);
    OSSchedLock();
    Flash_Write((uint8 *)(&Concentrator), sizeof(CONCENTRATOR_INFO), FLASH_CONCENTRATOR_INFO_ADDRESS);
    OSSchedUnlock();
}

/************************************************************************************************
* Function Name: Flash_LoadSubNodesInfo
* Decription   : Flash��ȡ�ڵ������Ϣ
* Input        : ��
* Output       : ��
* Others       : ��
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
* Decription   : Flash����ڵ������Ϣ
* Input        : ��
* Output       : SUCCESS-�ɹ�,ERROR-ʧ��
* Others       : ��
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
* Decription   : Flash��������
* Input        : StartAddr-��ʼ��ַ,PageCount-Ҫ������ҳ��
* Output       : ��
* Others       : ��
************************************************************************************************/
void Flash_Erase(uint32 StartAddr, uint8 PageCount)
{
    uint8 i;
    FLASH_Status flashStatus;

    // ����Flash��̲���������
    FLASH_Unlock();

    // ������б�ʶ
    FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    // ����Flashҳ
    flashStatus = FLASH_COMPLETE;
    for (i = 0; i < PageCount && FLASH_COMPLETE == flashStatus; i ++) {
        flashStatus = FLASH_ErasePage(StartAddr + (FLASH_PAGE_SIZE * i));
    }

    FLASH_Lock();
}

/************************************************************************************************
* Function Name: Flash_Read
* Decription   : Flash������
* Input        : BufPtr-ָ��������ݻ�������ָ��,Num-�������ݵ�����,FlashAddr-��Flash�ĵ�ַ
* Output       : ��
* Others       : ��
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
* Decription   : Flashд����
* Input        : BufPtr-ָ��д�����ݻ�������ָ��,Num-д�����ݵ�����,FlashAddr-д��Flash�ĵ�ַ
* Output       : ��
* Others       : Num����ֵ����Ϊż��
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

    // ����Flash��̲���������
    FLASH_Unlock();

    // ������б�ʶ
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

