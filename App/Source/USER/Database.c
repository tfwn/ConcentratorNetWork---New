/************************************************************************************************
*                                   SRWF-6009
*    (c) Copyright 2015, Software Department, Sunray Technology Co.Ltd
*                               All Rights Reserved
*
* FileName     : Database.c
* Description  :
* Version      :
* Function List:
*------------------------------Revision History--------------------------------------------------
* No.   Version     Date            Revised By      Item            Description
* 1     V1.1        08/11/2015      Zhangxp         SRWF-6009       Original Version
************************************************************************************************/

#define DATABASE_GLOBALS

/************************************************************************************************
*                             Include File Section
************************************************************************************************/
#include "Stm32f10x_conf.h"
#include "ucos_ii.h"
#include "Bsp.h"
#include "Main.h"
#include "Rtc.h"
#include "Gprs.h"
#include "Eeprom.h"
#include "Flash.h"
#include "Led.h"
#include "SerialPort.h"
#include "DataHandle.h"
#include "Database.h"
#include <string.h>

/************************************************************************************************
*                        Global Variable Declare Section
************************************************************************************************/

/************************************************************************************************
*                           Prototype Declare Section
************************************************************************************************/

/************************************************************************************************
*                           Function Declare Section
************************************************************************************************/

/************************************************************************************************
* Function Name: Data_Init
* Decription   : 数据库初始化
* Input        : 无
* Output       : 无
* Others       : 无
************************************************************************************************/
void Data_Init(void)
{
    Flash_LoadConcentratorInfo();
    DataReplenishRound = Concentrator.Param.DataReplenishCount;
    Flash_LoadSubNodesInfo();
    Data_RefreshNodeStatus();
}

/************************************************************************************************
* Function Name: Data_RefreshNodeStatus
* Decription   : 数据刷新,从Eeprom中读出数据并刷新节点的状态
* Input        : 无
* Output       : 无
* Others       : 无
************************************************************************************************/
void Data_RefreshNodeStatus(void)
{
    uint8 dataLen, meterDataLen;
    uint16 nodeId;
    METER_DATA_SAVE_FORMAT *meterBufPtr;

    if ((void *)0 == (meterBufPtr = OSMemGetOpt(SmallMemoryPtr, 10, TIME_DELAY_MS(50)))) {
        return;
    }
    for (nodeId = 0; nodeId < Concentrator.MaxNodeId; nodeId++) {
        if (0 == memcmp(SubNodes[nodeId].LongAddr, NullAddress, LONG_ADDR_SIZE)) {
            continue;
        }
        // 判断是否是计量表,不是则排除
        if ((SubNodes[nodeId].DevType & 0xF0) == 0xF0) {
            continue;
        }
        // 读取Eeprom中对应的值
        dataLen = RealTimeDataMode == Concentrator.Param.WorkType ? REALTIME_DATA_AREA_SIZE : FREEZE_DATA_AREA_SIZE;
        meterDataLen = sizeof(METER_DATA_SAVE_FORMAT) - 1 + dataLen;
        Eeprom_ReadWrite((uint8 *)meterBufPtr, nodeId * NODE_INFO_SIZE, meterDataLen, READ_ATTR);
        if (0 == memcmp(meterBufPtr->Address, SubNodes[nodeId].LongAddr, LONG_ADDR_SIZE)) {
            SubNodes[nodeId].Property.LastResult = meterBufPtr->Property.LastResult;
            SubNodes[nodeId].Property.CurRouteNo = meterBufPtr->Property.CurRouteNo;
            SubNodes[nodeId].Property.UploadData = meterBufPtr->Property.UploadData;
            SubNodes[nodeId].Property.UploadPrio = meterBufPtr->Property.UploadPrio;
            SubNodes[nodeId].RxMeterDataDay = meterBufPtr->RxMeterDataDay;
            SubNodes[nodeId].CmdProp.Content = meterBufPtr->CmdProp.Content;
            memcpy(&SubNodes[nodeId].CmdData, &meterBufPtr->CmdData, sizeof(CMD_DATA));
            SubNodes[nodeId].RxChannel = meterBufPtr->RxChannel;
            SubNodes[nodeId].TxChannel = meterBufPtr->TxChannel;
            SubNodes[nodeId].AutoChannelSwitch = meterBufPtr->AutoChannelSwitch;
            SubNodes[nodeId].RxMeterVersion = meterBufPtr->RxMeterVersion;
            memcpy(SubNodes[nodeId].RxMeterDataTemp, meterBufPtr->RxMeterDataTemp, 4);
            if (meterBufPtr->Crc8MeterData == CalCrc8(meterBufPtr->MeterData, dataLen)) {
                SubNodes[nodeId].RxChannel = (uint8)(meterBufPtr->MeterData[dataLen - UPDOWN_RSSI_SIZE - 2] >> 4 & 0x0F);
                SubNodes[nodeId].TxChannel = (uint8)(meterBufPtr->MeterData[dataLen - UPDOWN_RSSI_SIZE - 2] & 0x0F);
            } else {
                SubNodes[nodeId].RxChannel = DEFAULT_TX_CHANNEL;
                SubNodes[nodeId].TxChannel = DEFAULT_RX_CHANNEL;
            }
        } else {
            SubNodes[nodeId].Property.LastResult = 2;
            SubNodes[nodeId].Property.UploadData = TRUE;
            SubNodes[nodeId].Property.UploadPrio = FALSE;
            SubNodes[nodeId].RxMeterDataDay = 0;
            memset(SubNodes[nodeId].RxMeterDataTemp, 0, 4);
            SubNodes[nodeId].CmdProp.Content = 0x0;
            memset(&SubNodes[nodeId].CmdData, 0, sizeof(CMD_DATA));
            SubNodes[nodeId].RxChannel = DEFAULT_TX_CHANNEL;
            SubNodes[nodeId].TxChannel = DEFAULT_RX_CHANNEL;
            SubNodes[nodeId].AutoChannelSwitch = FALSE;
            SubNodes[nodeId].RxMeterVersion = 0;
        }
    }
    OSMemPut(SmallMemoryPtr, meterBufPtr);
}

/************************************************************************************************
* Function Name: Data_RdWrConcentratorParam
* Decription   : 读写集中器参数
* Input        : DataFrmPtr-指向数据帧的指针
* Output       : 无
* Others       : 工作类型(1)+工作参数(1)+数据上传时间点(1)+数据补抄日期(4)+补抄时间点(1)+补抄次数(1)+上传数据类型(1)
************************************************************************************************/
void Data_RdWrConcentratorParam(DATA_FRAME_STRUCT *DataFrmPtr)
{
    uint8 i, index;
    uint8 *dataBufPtr = DataFrmPtr->DataBuf;

    if (Read_CONC_Work_Param == DataFrmPtr->Command) {
        index = 0;
        *(dataBufPtr + index++) = Concentrator.Param.WorkType;
        *(dataBufPtr + index) = 0;
        if (Concentrator.Param.DataReplenishCtrl) {
            *(dataBufPtr + index) |= 0x80;
        }
        if (Concentrator.Param.DataUploadCtrl) {
            *(dataBufPtr + index) |= 0x40;
        }
        if (Concentrator.Param.DataEncryptCtrl) {
            *(dataBufPtr + index) |= 0x20;
        }
        if (Concentrator.Param.DataUploadMode) {
            *(dataBufPtr + index) |= 0x10;
        }
        index++;
        *(dataBufPtr + index++) = Concentrator.Param.DataUploadTime;
        for (i = 0; i < 4; i++) {
            *(dataBufPtr + index++) = Concentrator.Param.DataReplenishDay[i];
        }
        *(dataBufPtr + index++) = Concentrator.Param.DataReplenishHour;
        *(dataBufPtr + index++) = Concentrator.Param.DataReplenishCount;

        DataFrmPtr->DataLen = index;
    } else {
        if (Concentrator.Param.WorkType != (WORK_TYPE)(*dataBufPtr)) {
            if (DataFrmPtr->PortNo != Usart_Debug && DataFrmPtr->PortNo != Usb_Port) {
                *dataBufPtr = OP_Failure;
                DataFrmPtr->DataLen = 1;
                return;
            }
            Concentrator.Param.WorkType = (WORK_TYPE)(*dataBufPtr);
            DevResetTimer = 10000;
        }
        index = 1;
        Concentrator.Param.DataReplenishCtrl = (*(dataBufPtr + index) & 0x80) ? 1 : 0;
        Concentrator.Param.DataUploadCtrl = (*(dataBufPtr + index) & 0x40) ? 1 : 0;
        Concentrator.Param.DataEncryptCtrl = (*(dataBufPtr + index) & 0x20) ? 1 : 0;
        Concentrator.Param.DataUploadMode = (*(dataBufPtr + index) & 0x10) ? 1 : 0;
        index++;
        Concentrator.Param.DataUploadTime = *(dataBufPtr + index++);
        for (i = 0; i < 4; i++) {
            Concentrator.Param.DataReplenishDay[i] = *(dataBufPtr + index++);
        }
        Concentrator.Param.DataReplenishHour = *(dataBufPtr + index++) & 0x3F;
        Concentrator.Param.DataReplenishCount = *(dataBufPtr + index++);
        if (Concentrator.Param.DataReplenishCount > 10) {
            Concentrator.Param.DataReplenishCount = 10;
        }

        Flash_SaveConcentratorInfo();
        *dataBufPtr = OP_Succeed;
        DataFrmPtr->DataLen = 1;
        DataReplenishTimer = 5;
        DataUploadTimer = 30;
        DataReplenishRound = Concentrator.Param.DataReplenishCount;
    }
}

/************************************************************************************************
* Function Name: Data_MeterDataInit
* Decription   : 表数据初始化
* Input        : MeterBufPtr-数据存储的指针,NodeId-表编号,MeterDataLen-初始化数组的长度
* Output       : 无
* Others       : 无
************************************************************************************************/
void Data_MeterDataInit(METER_DATA_SAVE_FORMAT *MeterBufPtr, uint16 NodeId, uint8 MeterDataLen)
{
    memset(MeterBufPtr, 0, MeterDataLen);
    memcpy(MeterBufPtr->Address, SubNodes[NodeId].LongAddr, LONG_ADDR_SIZE);
    MeterBufPtr->Property.UploadData= SubNodes[NodeId].Property.UploadData = TRUE;
    MeterBufPtr->Property.UploadPrio = SubNodes[NodeId].Property.UploadPrio = FALSE;
    MeterBufPtr->RxMeterDataDay = SubNodes[NodeId].RxMeterDataDay = 0;
	memset(SubNodes[NodeId].RxMeterDataTemp, 0, 4);
	memset(MeterBufPtr->RxMeterDataTemp, 0, 4);
    MeterBufPtr->Crc8MeterData = 0xFF;
	SubNodes[NodeId].CmdProp.Content = 0x0;
	memset(&SubNodes[NodeId].CmdData, 0, sizeof(CMD_DATA));
    MeterBufPtr->RxChannel = SubNodes[NodeId].RxChannel = DEFAULT_TX_CHANNEL;
    MeterBufPtr->TxChannel = SubNodes[NodeId].TxChannel = DEFAULT_RX_CHANNEL;
    MeterBufPtr->AutoChannelSwitch = SubNodes[NodeId].AutoChannelSwitch = FALSE;
    MeterBufPtr->RxMeterVersion = SubNodes[NodeId].RxMeterVersion = 0;
}

/************************************************************************************************
* Function Name: Data_SetConcentratorAddr
* Decription   : 设置集中器地址,只能通过物理连接的通道设定
* Input        : DataFrmPtr-指向数据帧的指针
* Output       : 无
* Others       : 下行:集中器ID的BCD码(6)
*                上行:操作状态(1)
************************************************************************************************/
void Data_SetConcentratorAddr(DATA_FRAME_STRUCT *DataFrmPtr)
{
    if (DataFrmPtr->PortNo != Usart_Debug && DataFrmPtr->PortNo != Usb_Port) {
        DataFrmPtr->DataBuf[0] = OP_NoFunction;
    } else {
        if (0 == memcmp(Concentrator.LongAddr, DataFrmPtr->DataBuf, LONG_ADDR_SIZE)) {
            DataFrmPtr->DataBuf[0] = OP_Succeed;
        } else if (NULL_U16_ID != Data_FindNodeId(0, DataFrmPtr->DataBuf)) {
            DataFrmPtr->DataBuf[0] = OP_ObjectRepetitive;
        } else {
            memcpy(Concentrator.LongAddr, DataFrmPtr->DataBuf, LONG_ADDR_SIZE);
            Flash_SaveConcentratorInfo();
            DataFrmPtr->DataBuf[0] = OP_Succeed;
            DevResetTimer = 10000;      // 10秒钟后设备复位
        }
    }
    DataFrmPtr->DataLen = 1;
    return;
}

/************************************************************************************************
* Function Name: Data_FindNodeId
* Decription   : 从指定位置查找指定属性的节点ID
* Input        : StartId-指定查找的开始位置, BufPtr-指向节点长地址的指针
* Output       : 符合要求的节点ID值
* Others       : 无
************************************************************************************************/
uint16 Data_FindNodeId(uint16 StartId, uint8 *BufPtr)
{
    uint16 i;

    if (0 == memcmp(BufPtr, Concentrator.LongAddr, LONG_ADDR_SIZE)) {
        return DATA_CENTER_ID;
    }
    if (0 == memcmp(BufPtr, NullAddress, LONG_ADDR_SIZE)) {
        return NULL_U16_ID;
    }
    for (i = StartId; i < Concentrator.MaxNodeId; i++) {
        if (0 == memcmp(BufPtr, SubNodes[i].LongAddr, LONG_ADDR_SIZE)) {
            return i;
        }
    }
    return NULL_U16_ID;
}

/************************************************************************************************
* Function Name: Data_CleanCustomRoutes
* Decription   : 清理自定义路由中的无效数据
* Input        : 无
* Output       : 无
* Others       : 注意-该函数只是清理并保存了自定义路由数据区,同时也有可能改动了ConcentratorInfo和
*                     SubNodes两个结构体的数据并未保存,所以调用完此函数后要考虑到保存这两个数据库
************************************************************************************************/
void Data_CleanCustomRoutes(void)
{
    uint16 nodeId;
    uint8 i, j, routeFlag, *curFlashPtr, *newFlashPtr;;
    CUSTOM_ROUTE_INFO customRoute;

    curFlashPtr = Concentrator.CustomRouteSaveRegion == 0 ? (uint8 *)FLASH_CUSTOM_ROUTE_ADDRESS1 : (uint8 *)FLASH_CUSTOM_ROUTE_ADDRESS2;
    newFlashPtr = Concentrator.CustomRouteSaveRegion == 0 ? (uint8 *)FLASH_CUSTOM_ROUTE_ADDRESS2 : (uint8 *)FLASH_CUSTOM_ROUTE_ADDRESS1;
    Flash_Erase((uint32)newFlashPtr, FLASH_CUSTOM_ROUTE_PAGE_SIZE);
    if (Concentrator.MaxNodeId > 0) {
        for (nodeId = 0; nodeId < Concentrator.MaxNodeId; nodeId++) {
            if (0 == memcmp(SubNodes[nodeId].LongAddr, NullAddress, LONG_ADDR_SIZE)) {
                continue;
            }
            memcpy((uint8 *)(&customRoute), (uint8 *)(curFlashPtr + nodeId * sizeof(CUSTOM_ROUTE_INFO)), sizeof(CUSTOM_ROUTE_INFO));
            if (customRoute.AddrCrc16 != CalCrc16(SubNodes[nodeId].LongAddr, LONG_ADDR_SIZE)) {
                continue;
            }
            routeFlag = 0;
            for (i = 0; i < MAX_CUSTOM_ROUTES; i++) {
                for (j = 0; j < MAX_CUSTOM_ROUTE_LEVEL; j++) {
                    if (NULL_U16_ID == customRoute.RouteNode[i][j]) {
                        break;
                    }
                    if (0 == memcmp(SubNodes[customRoute.RouteNode[i][j]].LongAddr, NullAddress, LONG_ADDR_SIZE)) {
                        customRoute.RouteNode[i][0] = NULL_U16_ID;
                        break;
                    }
                }
                if (NULL_U16_ID != customRoute.RouteNode[i][0]) {
                    routeFlag |= (1 << i);
                }
            }
            if (routeFlag) {
                Flash_Write((uint8 *)(&customRoute), sizeof(CUSTOM_ROUTE_INFO), (uint32)(newFlashPtr + nodeId * sizeof(CUSTOM_ROUTE_INFO)));
            }
        }
    }
    Concentrator.CustomRouteSaveRegion = ~Concentrator.CustomRouteSaveRegion;
}

/************************************************************************************************
* Function Name: Data_ClearMeterData
* Decription   : 清除所有表数据,即从节点上传过来的
* Input        : 无
* Output       : 成功或者错误
* Others       : 主要是保存在Eeprom中的数据
************************************************************************************************/
bool Data_ClearMeterData(void)
{
    uint8 *msg;
    uint16 nodeId;

    if ((void *)0 == (msg = OSMemGetOpt(SmallMemoryPtr, 10, TIME_DELAY_MS(50)))) {
        return ERROR;
    }

    memset(msg, 0xFF, LONG_ADDR_SIZE);
    for (nodeId = 0; nodeId < MAX_NODE_NUM; nodeId++) {
        Eeprom_ReadWrite(msg, nodeId * NODE_INFO_SIZE, LONG_ADDR_SIZE, WRITE_ATTR);
    }
    OSMemPut(SmallMemoryPtr, msg);
    DataReplenishRound = Concentrator.Param.DataReplenishCount;
    return SUCCESS;
}

/************************************************************************************************
* Function Name: Data_ClearDatabase
* Decription   : 数据库初始化,清除所有表数据,中继数据,自定义路径,组网信息等
* Input        : 无
* Output       : 无
* Others       : 无
************************************************************************************************/
void Data_ClearDatabase(void)
{
    if (0 == Concentrator.MaxNodeId) {
        return;
    }
    Concentrator.MaxNodeId = 0;
    Concentrator.CustomRouteSaveRegion = 0;
    Flash_SaveConcentratorInfo();
    memset((uint8 *)(&SubNodes), 0xFF, sizeof(SUBNODE_INFO) * MAX_NODE_NUM);
    OSSchedLock();
    Flash_Erase(FLASH_NODE_INFO_ADDRESS, FLASH_NODE_INFO_PAGE_SIZE);
    Flash_Erase(FLASH_CUSTOM_ROUTE_ADDRESS1, FLASH_CUSTOM_ROUTE_PAGE_SIZE);
    Flash_Erase(FLASH_CUSTOM_ROUTE_ADDRESS2, FLASH_CUSTOM_ROUTE_PAGE_SIZE);
    OSSchedUnlock();
}

/************************************************************************************************
* Function Name: Data_ReadNodesCount
* Decription   : 数据库读节点的数量
* Input        : DataFrmPtr-指向数据帧的指针
* Output       : 无
* Others       : 下行:设备类型(1)
*                上行:设备类型(1)+档案数量(2)
************************************************************************************************/
void Data_ReadNodesCount(DATA_FRAME_STRUCT *DataFrmPtr)
{
    uint8 *dataBufPtr = DataFrmPtr->DataBuf;
    uint16 i, nodeCount;

    nodeCount = 0;
    for (i = 0; i < Concentrator.MaxNodeId; i++) {
        if (0 == memcmp(SubNodes[i].LongAddr, NullAddress, LONG_ADDR_SIZE)) {
            continue;
        }
        if (0xFF == *dataBufPtr || *dataBufPtr == SubNodes[i].DevType) {
            nodeCount++;
        }
    }
    *(dataBufPtr + 1) = (uint8)nodeCount;
    *(dataBufPtr + 2) = (uint8)(nodeCount >>8);
    DataFrmPtr->DataLen = 3;
}

/************************************************************************************************
* Function Name: Data_ReadNodes
* Decription   : 数据库读节点信息
* Input        : DataFrmPtr-指向数据帧的指针
* Output       : 无
* Others       : 下行:设备类型(1)+起始序号(2)+读取的数量
*                上行:该设备类型节点总数量(2)+本次应答的数量+N个表具的信息(8N)
************************************************************************************************/
void Data_ReadNodes(DATA_FRAME_STRUCT *DataFrmPtr)
{
    uint8 devType, rdCount, ackCount, *p, *dataBufPtr;
    uint16 i, rdStart, ackTotal;

    dataBufPtr = DataFrmPtr->DataBuf;
    devType = *dataBufPtr;
    rdStart = *(dataBufPtr + 1) + *(dataBufPtr + 2) * 256;
    rdCount = *(dataBufPtr + 3);

    ackTotal = 0;
    ackCount = 0;
    p = dataBufPtr + 3;
    for (i = 0; i < Concentrator.MaxNodeId; i++) {
        if (0 == memcmp(SubNodes[i].LongAddr, NullAddress, LONG_ADDR_SIZE)) {
            continue;
        }
        if (0xFF == devType || SubNodes[i].DevType == devType) {
            if (ackTotal >= rdStart && ackCount < rdCount) {
                memcpy(p, SubNodes[i].LongAddr, LONG_ADDR_SIZE);
                p += LONG_ADDR_SIZE;
                *p++ = SubNodes[i].DevType;
                *p++ = SubNodes[i].Property.LastResult;
                ackCount++;
            }
            ackTotal++;
        }
    }
    *dataBufPtr = (uint8)ackTotal;
    *(dataBufPtr + 1) = (uint8)(ackTotal >> 8);
    *(dataBufPtr + 2) = ackCount;
    DataFrmPtr->DataLen = ackCount * (LONG_ADDR_SIZE + 2) + 3;
}

/************************************************************************************************
* Function Name: Data_WriteNodes
* Decription   : 数据库写节点信息
* Input        : DataFrmPtr-指向数据帧的指针
* Output       : 无
* Others       : 下行:设置节点数量+最后一包标识+N个表具的BCD码(8N:包括地址,设备类型,最后一次抄表结果)
*                上行:设置节点数量+表具的BCD码和操作状态字(7N)
************************************************************************************************/
void Data_WriteNodes(DATA_FRAME_STRUCT *DataFrmPtr)
{
    uint8 err, nodeNum, saveNow, *rdPtr, *wrPtr, *dataBufPtr;
    uint16 i;
    bool save = FALSE;
    OP_RESULT result;

    // 提取设置节点的数量和最后一包标识
    dataBufPtr = DataFrmPtr->DataBuf;
    nodeNum = *dataBufPtr;
    saveNow = *(dataBufPtr + 1);
    if (DataFrmPtr->DataLen != nodeNum * (LONG_ADDR_SIZE + 2) + 2) {
        *dataBufPtr = OP_FormatError;
        DataFrmPtr->DataLen = 1;
        return;
    }
    // 设置读写指针
    rdPtr = dataBufPtr + 2;
    wrPtr = dataBufPtr + 1;
    // 向节点列表中增加节点
    while (nodeNum-- > 0) {
        if (NULL_U16_ID != Data_FindNodeId(0, rdPtr)) {
            result = OP_ObjectRepetitive;
        } else {
            if (Concentrator.MaxNodeId >= MAX_NODE_NUM) {
                result = OP_Objectoverflow;
            } else {
                save = TRUE;
                // 寻找空位置
                for (i = 0; i < Concentrator.MaxNodeId; i++) {
                    if (0 == memcmp(SubNodes[i].LongAddr, NullAddress, LONG_ADDR_SIZE)) {
                        break;
                    }
                }
                if (i >= Concentrator.MaxNodeId) {
                    Concentrator.MaxNodeId++;
                }
                memcpy(SubNodes[i].LongAddr, rdPtr, LONG_ADDR_SIZE);
                SubNodes[i].DevType = (DEVICE_TYPE)(*(rdPtr + LONG_ADDR_SIZE));
                SubNodes[i].Property.LastResult = 2;
                SubNodes[i].Property.UploadData = TRUE;
                SubNodes[i].Property.UploadPrio = LOW;
                SubNodes[i].RxMeterDataDay = 0;
				memset(SubNodes[i].RxMeterDataTemp, 0, 4);
				SubNodes[i].CmdProp.Content = 0x0;
				memset(&SubNodes[i].CmdData, 0, sizeof(CMD_DATA));
                SubNodes[i].RxChannel = DEFAULT_TX_CHANNEL;
                SubNodes[i].TxChannel = DEFAULT_RX_CHANNEL;
                SubNodes[i].AutoChannelSwitch = FALSE;
                SubNodes[i].RxMeterVersion = 0;
                result = OP_Succeed;
            }
        }
        memcpy(wrPtr, rdPtr, LONG_ADDR_SIZE);
        *(wrPtr + LONG_ADDR_SIZE) = result;
        rdPtr += LONG_ADDR_SIZE + 2;
        wrPtr += LONG_ADDR_SIZE + 1;
    }

    // 延时后保存,必须延时同样的时间,否则易造成数据不一致的情况
    if (TRUE == save) {
        if (saveNow) {
            OSFlagPost(GlobalEventFlag, (OS_FLAGS)FLAG_DELAY_SAVE_TIMER, OS_FLAG_SET, &err);
        } else {
            SubNodesSaveDelayTimer = DATA_SAVE_DELAY_TIMER;
        }
    }
    DataFrmPtr->DataLen = 1 + (LONG_ADDR_SIZE + 1) * *DataFrmPtr->DataBuf;
}

/************************************************************************************************
* Function Name: Data_DeleteNodes
* Decription   : 数据库删除节点信息
* Input        : DataFrmPtr-指向数据帧的指针
* Output       : 无
* Others       : 下行:N个表具的BCD码(6N)
*                上行:表具的BCD码和操作状态字(7N)
*          注意: 删除后要将包含此节点的自定义路由和以此节点为邻居的节点删除掉
************************************************************************************************/
void Data_DeleteNodes(DATA_FRAME_STRUCT *DataFrmPtr)
{
    uint8 *dataBufPtr;
    uint16 i, j, nodeId, nodeNum;

    dataBufPtr = DataFrmPtr->DataBuf;
    nodeNum = DataFrmPtr->DataLen / LONG_ADDR_SIZE;
    // 因为返回的数据比原数据要长且共用同一个存储区域,所以要进行数据搬移
    i = nodeNum * LONG_ADDR_SIZE - 1;
    j = nodeNum * (LONG_ADDR_SIZE + 1) - 1;
    while (i > LONG_ADDR_SIZE - 2) {
        if (0 == (j + 1) % (LONG_ADDR_SIZE + 1)) {
            *(dataBufPtr + j) = OP_Executing;
            j -= 1;
        }
        *(dataBufPtr + j--) = *(dataBufPtr + i--);
    }

    // 从节点列表中删除节点
    while (nodeNum-- > 0) {
        nodeId = Data_FindNodeId(0, dataBufPtr);
        if (NULL_U16_ID == nodeId || DATA_CENTER_ID == nodeId) {
            *(dataBufPtr + LONG_ADDR_SIZE) = OP_ObjectNotExist;
        } else {
            if (Concentrator.MaxNodeId > 0 && nodeId == Concentrator.MaxNodeId - 1) {
                Concentrator.MaxNodeId--;
            }
            memcpy(SubNodes[nodeId].LongAddr, NullAddress, LONG_ADDR_SIZE);
            *(dataBufPtr + LONG_ADDR_SIZE) = OP_Succeed;
        }
        dataBufPtr += LONG_ADDR_SIZE + 1;
    }

    // 保存集中器信息和节点信息
    Data_CleanCustomRoutes();
    Flash_SaveConcentratorInfo();
    Flash_SaveSubNodesInfo();

    nodeNum = DataFrmPtr->DataLen / LONG_ADDR_SIZE;
    DataFrmPtr->DataLen = (LONG_ADDR_SIZE + 1) * nodeNum;
}

/************************************************************************************************
* Function Name: Data_ModifyNodes
* Decription   : 数据库更改节点地址信息(0x54)
* Input        : BufPtr-指向数据帧的指针
* Output       : 无
* Others       : 下行:原表号(6)+新表号(6)+设备类型
*                上行:操作状态
************************************************************************************************/
void Data_ModifyNodes(DATA_FRAME_STRUCT *DataFrmPtr)
{
    uint16 oldNodeId, newNodeId;
    uint8 *dataBufPtr = DataFrmPtr->DataBuf;

    DataFrmPtr->DataLen = 1;

    // 查找原表号是否存在
    oldNodeId = Data_FindNodeId(0, dataBufPtr);
    if (NULL_U16_ID == oldNodeId) {
        *dataBufPtr = OP_ObjectNotExist;
        return;
    }
    // 查找新表号是否存在
    newNodeId = Data_FindNodeId(0, dataBufPtr + LONG_ADDR_SIZE);
    if (DATA_CENTER_ID == newNodeId) {
        *dataBufPtr = OP_ObjectRepetitive;
        return;
    }
    if (NULL_U16_ID != newNodeId && newNodeId != oldNodeId) {
        *dataBufPtr = OP_ObjectRepetitive;
        return;
    }
    // 替换原表号并保存
    memcpy(SubNodes[oldNodeId].LongAddr, dataBufPtr + LONG_ADDR_SIZE, LONG_ADDR_SIZE);
    SubNodes[oldNodeId].DevType = (DEVICE_TYPE)(*(dataBufPtr + LONG_ADDR_SIZE * 2));
    Flash_SaveSubNodesInfo();
    *dataBufPtr = OP_Succeed;
    return;
}

/************************************************************************************************
* Function Name: Data_GetTimeSlot
* Decription   : 获取某个节点的时隙和总的节点数
* Input        : NodeIdPtr-节点地址指针,TimeSlotPtr-指向保存该节点时隙和节点总数的指针
* Output       : None
* Others       : 时隙为4个字节,依次为时隙低字节,时隙高字节,总表数低字节,总表数高字节.
************************************************************************************************/
void Data_GetTimeSlot(uint8 *NodeIdPtr, uint8 *TimeSlotPtr)
{
    uint16 i, timeSlot, nodeTotal;

    timeSlot = 0;
    nodeTotal = 0;
    // 寻找该节点的时隙值
    for (i = 0; i < Concentrator.MaxNodeId; i++) {
        if (0 == memcmp(SubNodes[i].LongAddr, NullAddress, LONG_ADDR_SIZE)) {
            continue;
        }
        nodeTotal++;
        if (0 == memcmp(SubNodes[i].LongAddr, NodeIdPtr, LONG_ADDR_SIZE)) {
            timeSlot = nodeTotal;
        }
    }
    if (0 == timeSlot) {
        memset(TimeSlotPtr, 0, 4);
    } else {
        *TimeSlotPtr++ = (uint8)timeSlot;
        *TimeSlotPtr++ = (uint8)(timeSlot >> 8);
        *TimeSlotPtr++ = (uint8)nodeTotal;
        *TimeSlotPtr++ = (uint8)(nodeTotal >> 8);
    }
}

/************************************************************************************************
* Function Name: Data_GetRoute
* Decription   : 获取某个节点的路径
* Input        : NodeId-该节点的ID,Buf-保存该路径的数据指针
* Output       : 路径级数
* Others       :
************************************************************************************************/
uint8 Data_GetRoute(uint16 NodeId, uint8 *RouteBufPtr)
{
    bool findRoute;
    uint8 level, *curFlashPtr;
    uint16 i, tmpNodeId;
    CUSTOM_ROUTE_INFO *p;

    // 如果该节点有自定义路由,则使用自定义路由的路径
    curFlashPtr = Concentrator.CustomRouteSaveRegion == 0 ? (uint8 *)FLASH_CUSTOM_ROUTE_ADDRESS1 : (uint8 *)FLASH_CUSTOM_ROUTE_ADDRESS2;
    p = (CUSTOM_ROUTE_INFO *)(curFlashPtr + NodeId * sizeof(CUSTOM_ROUTE_INFO));
    memcpy(RouteBufPtr, Concentrator.LongAddr, LONG_ADDR_SIZE);
    level = 0;
    findRoute = FALSE;
    if (p->AddrCrc16 == CalCrc16(SubNodes[NodeId].LongAddr, LONG_ADDR_SIZE)) {
        for (i = 0; i < MAX_CUSTOM_ROUTES && FALSE == findRoute; i++) {
            if (NULL_U16_ID == p->RouteNode[SubNodes[NodeId].Property.CurRouteNo % MAX_CUSTOM_ROUTES][0]) {
                SubNodes[NodeId].Property.CurRouteNo = (SubNodes[NodeId].Property.CurRouteNo + 1) % MAX_CUSTOM_ROUTES;
            } else {
                for (level = 0; level < MAX_CUSTOM_ROUTE_LEVEL; level++) {
                    tmpNodeId = p->RouteNode[SubNodes[NodeId].Property.CurRouteNo % MAX_CUSTOM_ROUTES][level];
                    if (NULL_U16_ID == tmpNodeId || tmpNodeId >= Concentrator.MaxNodeId) {
                        break;
                    } else {
                        findRoute = TRUE;
                        RouteBufPtr += LONG_ADDR_SIZE;
                        memcpy(RouteBufPtr, SubNodes[tmpNodeId].LongAddr, LONG_ADDR_SIZE);
                    }
                }
            }
        }
    }
    RouteBufPtr += LONG_ADDR_SIZE;
    memcpy(RouteBufPtr, SubNodes[NodeId].LongAddr, LONG_ADDR_SIZE);
    return (level + 2);
}

/************************************************************************************************
* Function Name: Data_ReadCustomRoute
* Decription   : 读取表具,中继器的自定义路由信息(0x55)
* Input        : DataFrmPtr-指向数据帧的指针
* Output       : 无
* Others       : 下行:表具号(6)
*                上行:操作状态(1)+表具号(6)+设备类型(1)+路径1的节点数量+路径1信息(6N)+
*                     路径2的节点数量+路径2信息(6M)
************************************************************************************************/
void Data_ReadCustomRoute(DATA_FRAME_STRUCT *DataFrmPtr)
{
    uint8 *dataBufPtr, *numPtr, *curFlashPtr;
    uint16 i, nodeId, len;
    CUSTOM_ROUTE_INFO *customRoutePtr;

    // 查找表具的节点ID
    dataBufPtr = DataFrmPtr->DataBuf;
    nodeId = Data_FindNodeId(0, dataBufPtr);
    for (i = LONG_ADDR_SIZE; i > 0; i--) {
        *(dataBufPtr + i) = *(dataBufPtr + i - 1);
    }
    len = LONG_ADDR_SIZE + 1;
    if (NULL_U16_ID == nodeId) {
        *dataBufPtr = OP_ObjectNotExist;
        *(dataBufPtr + len++) = Dev_Empty;
        *(dataBufPtr + len++) = 0;
        *(dataBufPtr + len++) = 0;
        DataFrmPtr->DataLen = len;
        return;
    }

    // 查找该节点ID的自定义路径
    *dataBufPtr = OP_Succeed;
    *(dataBufPtr + len++) = SubNodes[nodeId].DevType;
    curFlashPtr = Concentrator.CustomRouteSaveRegion == 0 ? (uint8 *)FLASH_CUSTOM_ROUTE_ADDRESS1 : (uint8 *)FLASH_CUSTOM_ROUTE_ADDRESS2;
    customRoutePtr = (CUSTOM_ROUTE_INFO *)(curFlashPtr + nodeId * sizeof(CUSTOM_ROUTE_INFO));
    if (customRoutePtr->AddrCrc16 != CalCrc16(SubNodes[nodeId].LongAddr, LONG_ADDR_SIZE)) {
        *(dataBufPtr + len++) = 0;
        *(dataBufPtr + len++) = 0;
        DataFrmPtr->DataLen = len;
        return;
    }
    for (i = 0; i < MAX_CUSTOM_ROUTES; i++) {
        numPtr = dataBufPtr + len++;
        *numPtr = 0;
        while (NULL_U16_ID != customRoutePtr->RouteNode[i][*numPtr]) {
            memcpy(dataBufPtr + len, SubNodes[customRoutePtr->RouteNode[i][*numPtr]].LongAddr, LONG_ADDR_SIZE);
            len += LONG_ADDR_SIZE;
            *numPtr += 1;
            if (*numPtr >= MAX_CUSTOM_ROUTE_LEVEL) {
                break;
            }
        }
    }
    DataFrmPtr->DataLen = len;
}

/************************************************************************************************
* Function Name: Data_WriteCustomRoute
* Decription   : 清除或设置表具,中继器的自定义路由信息(0x56)
* Input        : DataFrmPtr-指向数据帧的指针
* Output       : 无
* Others       : 下行:表具号(6)+路径1的节点数量+路径1信息(6N)+路径2的节点数量+路径2信息(6M)
*                上行:表具号(6)+操作状态
*                注意:当路径条数为0时表示清除该节点的自定义路径
************************************************************************************************/
void Data_WriteCustomRoute(DATA_FRAME_STRUCT *DataFrmPtr)
{
    uint8 retry, count, *dataBufPtr, *curFlashPtr, *newFlashPtr;
    uint16 i, j, k, nodeId, routeId;
    CUSTOM_ROUTE_INFO customRoute;

    // 先判断增加的路径节点是否合法
    dataBufPtr = DataFrmPtr->DataBuf;
    DataFrmPtr->DataLen = LONG_ADDR_SIZE + 1;
    nodeId = Data_FindNodeId(0, dataBufPtr);
    if (NULL_U16_ID == nodeId || DATA_CENTER_ID == nodeId) {
        *(DataFrmPtr->DataBuf + LONG_ADDR_SIZE) = OP_ObjectNotExist;
        return;
    }
    customRoute.AddrCrc16 = CalCrc16(SubNodes[nodeId].LongAddr, LONG_ADDR_SIZE);
    dataBufPtr += LONG_ADDR_SIZE;
    for (i = 0; i < MAX_CUSTOM_ROUTES; i++) {
        count = *dataBufPtr++;
        if (count > MAX_CUSTOM_ROUTE_LEVEL) {
            *(DataFrmPtr->DataBuf + LONG_ADDR_SIZE) = OP_ParameterError;
            return;
        }
        for (j = 0; j < count; j++) {
            routeId = Data_FindNodeId(0, dataBufPtr);
            if (NULL_U16_ID == routeId) {
                *(DataFrmPtr->DataBuf + LONG_ADDR_SIZE) = OP_ObjectNotExist;
                return;
            }
            if (DATA_CENTER_ID == routeId || nodeId == routeId) {
                *(DataFrmPtr->DataBuf + LONG_ADDR_SIZE) = OP_ObjectRepetitive;
                return;
            }
            for (k = 0; k < j; k++) {
                if (routeId == customRoute.RouteNode[i][k]) {
                    *(DataFrmPtr->DataBuf + LONG_ADDR_SIZE) = OP_ObjectRepetitive;
                    return;
                }
            }
            customRoute.RouteNode[i][j] = routeId;
            dataBufPtr += LONG_ADDR_SIZE;
        }
        for ( ; j < MAX_CUSTOM_ROUTE_LEVEL; j++) {
            customRoute.RouteNode[i][j] = NULL_U16_ID;
        }
    }

    // 判断和现有的路径是否重复
    curFlashPtr = 0 == Concentrator.CustomRouteSaveRegion ? (uint8 *)FLASH_CUSTOM_ROUTE_ADDRESS1 : (uint8 *)FLASH_CUSTOM_ROUTE_ADDRESS2;
    if (0 == memcmp(&customRoute, curFlashPtr + nodeId * sizeof(CUSTOM_ROUTE_INFO), sizeof(CUSTOM_ROUTE_INFO))) {
        *(DataFrmPtr->DataBuf + LONG_ADDR_SIZE) = OP_Succeed;
        return;
    }
    // 写入该自定义路径信息
    OSSchedLock();
    Flash_Write((uint8 *)(&customRoute), sizeof(CUSTOM_ROUTE_INFO), (uint32)(curFlashPtr + nodeId * sizeof(CUSTOM_ROUTE_INFO)));
    OSSchedUnlock();
    // 检查是否成功,不成功则重写
    if (0 == memcmp(&customRoute, curFlashPtr + nodeId * sizeof(CUSTOM_ROUTE_INFO), sizeof(CUSTOM_ROUTE_INFO))) {
        *(DataFrmPtr->DataBuf + LONG_ADDR_SIZE) = OP_Succeed;
        return;
    }
    newFlashPtr = 0 == Concentrator.CustomRouteSaveRegion ? (uint8 *)FLASH_CUSTOM_ROUTE_ADDRESS2 : (uint8 *)FLASH_CUSTOM_ROUTE_ADDRESS1;
    // 如果失败则重试5次
    for (retry = 0; retry < 5; retry++) {
        OSSchedLock();
        Flash_Erase((uint32)newFlashPtr, FLASH_CUSTOM_ROUTE_PAGE_SIZE);
        OSSchedUnlock();
        for (i = 0; i < Concentrator.MaxNodeId; i++) {
            if (0 == memcmp(SubNodes[i].LongAddr, NullAddress, LONG_ADDR_SIZE)) {
                continue;
            }
            if (nodeId == i) {
                OSSchedLock();
                Flash_Write((uint8 *)(&customRoute), sizeof(CUSTOM_ROUTE_INFO), (uint32)(newFlashPtr + i * sizeof(CUSTOM_ROUTE_INFO)));
                OSSchedUnlock();
                if (0 != memcmp(&customRoute, newFlashPtr + i * sizeof(CUSTOM_ROUTE_INFO), sizeof(CUSTOM_ROUTE_INFO))) {
                    break;
                }
            } else {
                OSSchedLock();
                Flash_Write(curFlashPtr + i * sizeof(CUSTOM_ROUTE_INFO), sizeof(CUSTOM_ROUTE_INFO), (uint32)(newFlashPtr + i * sizeof(CUSTOM_ROUTE_INFO)));
                OSSchedUnlock();
                if (0 != memcmp(curFlashPtr + i * sizeof(CUSTOM_ROUTE_INFO), newFlashPtr + i * sizeof(CUSTOM_ROUTE_INFO), sizeof(CUSTOM_ROUTE_INFO))) {
                    break;
                }
            }
        }
        if (i >= Concentrator.MaxNodeId) {
            *(DataFrmPtr->DataBuf + LONG_ADDR_SIZE) = OP_Succeed;
            Concentrator.CustomRouteSaveRegion = ~Concentrator.CustomRouteSaveRegion;
            Flash_SaveConcentratorInfo();
            return;
        }
    }
    *(DataFrmPtr->DataBuf + LONG_ADDR_SIZE) = OP_Failure;
    return;;
}

/************************************************************************************************
* Function Name: Data_BatchReadCustomRoutes
* Decription   : 批量读取表具,中继器的自定义路由信息(0x57)
* Input        : DataFrmPtr-指向数据帧的指针
* Output       : 无
* Others       : 下行:起始序号(2)+读取的数量(1)
*                上行:节点总数量(2)+本次读取数量(1)+N*(设备长地址(6)+设备类型(1)+
*                     路径1节点数量X(1)+X*路径1信息队列(6)+路径2节点数量Y(1)+Y*路径2信息队列(6)
************************************************************************************************/
void Data_BatchReadCustomRoutes(DATA_FRAME_STRUCT *DataFrmPtr)
{
    uint8 i, readCount, ackCount, blockLen, *dataBufPtr, *numPtr, *curFlashPtr;
    uint16 nodeId, startId, totalNodes;
    CUSTOM_ROUTE_INFO *customRoutePtr;

    startId = ((uint16 *)DataFrmPtr->DataBuf)[0];
    readCount = DataFrmPtr->DataBuf[2];
    ackCount = 0;
    totalNodes = 0;
    blockLen = LONG_ADDR_SIZE + 1 + (1 + LONG_ADDR_SIZE * MAX_CUSTOM_ROUTE_LEVEL) * 2;
    dataBufPtr = DataFrmPtr->DataBuf + 3;
    curFlashPtr = Concentrator.CustomRouteSaveRegion == 0 ? (uint8 *)FLASH_CUSTOM_ROUTE_ADDRESS1 : (uint8 *)FLASH_CUSTOM_ROUTE_ADDRESS2;
    for (nodeId = 0; nodeId < Concentrator.MaxNodeId; nodeId++) {
        if (0 == memcmp(SubNodes[nodeId].LongAddr, NullAddress, LONG_ADDR_SIZE)) {
            continue;
        }
        totalNodes++;
        if (totalNodes > startId && ackCount < readCount && dataBufPtr - DataFrmPtr->DataBuf + blockLen < GPRS_DATA_MAX_DATA) {
            ackCount++;
            memcpy(dataBufPtr, SubNodes[nodeId].LongAddr, LONG_ADDR_SIZE);
            dataBufPtr += LONG_ADDR_SIZE;
            *dataBufPtr++ = SubNodes[nodeId].DevType;
            customRoutePtr = (CUSTOM_ROUTE_INFO *)(curFlashPtr + nodeId * sizeof(CUSTOM_ROUTE_INFO));
            if (customRoutePtr->AddrCrc16 != CalCrc16(SubNodes[nodeId].LongAddr, LONG_ADDR_SIZE)) {
                *dataBufPtr++ = 0;
                *dataBufPtr++ = 0;
                continue;
            }
            for (i = 0; i < MAX_CUSTOM_ROUTES; i++) {
                numPtr = dataBufPtr++;
                *numPtr = 0;
                while (NULL_U16_ID != customRoutePtr->RouteNode[i][*numPtr]) {
                    memcpy(dataBufPtr, SubNodes[customRoutePtr->RouteNode[i][*numPtr]].LongAddr, LONG_ADDR_SIZE);
                    dataBufPtr += LONG_ADDR_SIZE;
                    *numPtr += 1;
                    if (*numPtr >= MAX_CUSTOM_ROUTE_LEVEL) {
                        break;
                    }
                }
            }
        }
    }
    DataFrmPtr->DataBuf[0] = (uint8)totalNodes;
    DataFrmPtr->DataBuf[1] = (uint8)(totalNodes >> 8);
    DataFrmPtr->DataBuf[2] = ackCount;
    DataFrmPtr->DataLen = dataBufPtr - DataFrmPtr->DataBuf;
}

/************************************************************************************************
* Function Name: Data_BatchWriteCustomRoutes
* Decription   : 清除或设置表具,中继器的自定义路由信息(0x58)
* Input        : DataFrmPtr-指向数据帧的指针
* Output       : 无
* Others       : 下行:(表具号(6)+路径1的节点数量X(1)+路径1信息(6X)+路径2的节点数量Y+路径2信息(6Y))*N
*                上行:(表具号(6)+操作状态)*N
************************************************************************************************/
void Data_BatchWriteCustomRoutes(DATA_FRAME_STRUCT *DataFrmPtr)
{
    bool errFind, checkPassed;
    OP_RESULT result;
    uint8 i, j, k, retry, nodeCount, routeCount;
    uint8 *msg, *txDataBufPtr, *rxDataBufPtr, *curFlashPtr, *newFlashPtr;
    uint16 nodeId, routeId, *nodeIdPtr;
    CUSTOM_ROUTE_INFO customRoute, *customRoutePtr;

    if ((void *)0 == (nodeIdPtr = OSMemGetOpt(SmallMemoryPtr, 10, TIME_DELAY_MS(50)))) {
        DataFrmPtr->DataBuf[0] = OP_Failure;
        DataFrmPtr->DataLen = 1;
        return;
    }
    if ((void *)0 == (msg = OSMemGetOpt(LargeMemoryPtr, 10, TIME_DELAY_MS(50)))) {
        OSMemPut(SmallMemoryPtr, nodeIdPtr);
        DataFrmPtr->DataBuf[0] = OP_Failure;
        DataFrmPtr->DataLen = 1;
        return;
    }
    memcpy(msg, DataFrmPtr->DataBuf, DataFrmPtr->DataLen);

    // 如果出现错误则重试5次
    for (retry = 0; retry < 5; retry++) {
        nodeCount = 0;
        txDataBufPtr = DataFrmPtr->DataBuf;
        rxDataBufPtr = msg;
        curFlashPtr = 0 == Concentrator.CustomRouteSaveRegion ? (uint8 *)FLASH_CUSTOM_ROUTE_ADDRESS1 : (uint8 *)FLASH_CUSTOM_ROUTE_ADDRESS2;
        newFlashPtr = 0 == Concentrator.CustomRouteSaveRegion ? (uint8 *)FLASH_CUSTOM_ROUTE_ADDRESS2 : (uint8 *)FLASH_CUSTOM_ROUTE_ADDRESS1;
        OSSchedLock();
        Flash_Erase((uint32)newFlashPtr, FLASH_CUSTOM_ROUTE_PAGE_SIZE);
        OSSchedUnlock();
        while (rxDataBufPtr - msg < DataFrmPtr->DataLen) {
            nodeId = Data_FindNodeId(0, rxDataBufPtr);
            memcpy(txDataBufPtr, rxDataBufPtr, LONG_ADDR_SIZE);
            txDataBufPtr += LONG_ADDR_SIZE;
            rxDataBufPtr += LONG_ADDR_SIZE;
            if (NULL_U16_ID == nodeId) {
                *(nodeIdPtr + nodeCount++) = NULL_U16_ID;
                for (i = 0; i < MAX_CUSTOM_ROUTES; i++) {
                    routeCount = *rxDataBufPtr++;
                    rxDataBufPtr += routeCount * LONG_ADDR_SIZE;
                }
                *txDataBufPtr++ = OP_ObjectNotExist;
            } else {
                errFind = FALSE;
                checkPassed = TRUE;
                customRoute.AddrCrc16 = CalCrc16(SubNodes[nodeId].LongAddr, LONG_ADDR_SIZE);
                for (i = 0; i < MAX_CUSTOM_ROUTES; i++) {
                    for (j = 0; j < MAX_CUSTOM_ROUTE_LEVEL; j++) {
                        customRoute.RouteNode[i][j] = NULL_U16_ID;
                    }
                }
                for (i = 0; i < MAX_CUSTOM_ROUTES; i++) {
                    routeCount = *rxDataBufPtr++;
                    for (j = 0; j < routeCount && j < MAX_CUSTOM_ROUTE_LEVEL; j++) {
                        routeId = Data_FindNodeId(0, rxDataBufPtr);
                        rxDataBufPtr += LONG_ADDR_SIZE;
                        if (NULL_U16_ID == routeId) {
                            result = OP_ObjectNotExist;
                            errFind = TRUE;
                            rxDataBufPtr += (routeCount - j - 1) * LONG_ADDR_SIZE;
                            break;
                        } else {
                            customRoute.RouteNode[i][j] = routeId;
                        }
                    }
                    if (FALSE == errFind && routeCount > 1) {
                        for (j = 0; j < routeCount && FALSE == errFind; j++) {
                            if (customRoute.RouteNode[i][j] == nodeId || customRoute.RouteNode[i][j] == DATA_CENTER_ID) {
                                result = OP_ObjectRepetitive;
                                errFind = TRUE;
                            }
                        }
                        for (j = 0; j < routeCount - 1 && FALSE == errFind; j++) {
                            for (k = j + 1; k < routeCount && errFind == FALSE; k++) {
                                if (customRoute.RouteNode[i][j] == customRoute.RouteNode[i][k]) {
                                    result = OP_ObjectRepetitive;
                                    errFind = TRUE;
                                }
                            }
                        }
                    }
                }
                if (TRUE == errFind) {
                    *txDataBufPtr++ = result;
                    *(nodeIdPtr + nodeCount++) = NULL_U16_ID;
                } else {
                    *txDataBufPtr++ = OP_Succeed;
                    *(nodeIdPtr + nodeCount++) = nodeId;
                    if (customRoute.RouteNode[0][0] != NULL_U16_ID || customRoute.RouteNode[1][0] != NULL_U16_ID) {
                        OSSchedLock();
                        Flash_Write((uint8 *)(&customRoute), sizeof(CUSTOM_ROUTE_INFO), (uint32)(newFlashPtr + nodeId * sizeof(CUSTOM_ROUTE_INFO)));
                        OSSchedUnlock();
                        if (0 != memcmp(&customRoute, newFlashPtr + nodeId * sizeof(CUSTOM_ROUTE_INFO), sizeof(CUSTOM_ROUTE_INFO))) {
                            checkPassed = FALSE;
                            break;
                        }
                    }
                }
            }
        }
        if (FALSE == checkPassed) {
            continue;
        }
        // 复制其他没有修改的路由
        for (nodeId = 0; nodeId < Concentrator.MaxNodeId; nodeId++) {
            if (0 == memcmp(SubNodes[nodeId].LongAddr, NullAddress, LONG_ADDR_SIZE)) {
                continue;
            }
            for (i = 0; i < nodeCount; i++) {
                if (nodeId == *(nodeIdPtr + i)) {
                    break;
                }
            }
            if (i < nodeCount) {
                continue;
            }
            customRoutePtr = (CUSTOM_ROUTE_INFO *)(curFlashPtr + nodeId * sizeof(CUSTOM_ROUTE_INFO));
            if (customRoutePtr->AddrCrc16 != CalCrc16(SubNodes[nodeId].LongAddr, LONG_ADDR_SIZE)) {
                continue;
            }
            if (NULL_U16_ID == customRoutePtr->RouteNode[0][0] && NULL_U16_ID == customRoutePtr->RouteNode[1][0]) {
                continue;
            }
            OSSchedLock();
            Flash_Write((uint8 *)customRoutePtr, sizeof(CUSTOM_ROUTE_INFO), (uint32)(newFlashPtr + nodeId * sizeof(CUSTOM_ROUTE_INFO)));
            OSSchedUnlock();
            if (0 != memcmp(customRoutePtr, newFlashPtr + nodeId * sizeof(CUSTOM_ROUTE_INFO), sizeof(CUSTOM_ROUTE_INFO))) {
                break;
            }
        }
        if (nodeId < Concentrator.MaxNodeId) {
            continue;
        } else {
            break;
        }
    }
    OSMemPut(SmallMemoryPtr, nodeIdPtr);
    OSMemPut(LargeMemoryPtr, msg);
    if (retry >= 5) {
        DataFrmPtr->DataBuf[0] = OP_Failure;
        DataFrmPtr->DataLen = 1;
        return;
    }
    Concentrator.CustomRouteSaveRegion = ~Concentrator.CustomRouteSaveRegion;
    Flash_SaveConcentratorInfo();
    DataFrmPtr->DataLen = txDataBufPtr - DataFrmPtr->DataBuf;
}


/************************************************************************************************
* Function Name: Data_BatchWriteMeterCmdLoadProc
* Decription   : 批量写表端下发命令保存处理函数(0x69)
* Input        : DataFrmPtr-指向数据帧的指针
* Output       : 无
* Others       : 下行:(表具号[6]+数据[1+8])*N
*                上行:(表具号[6]+操作状态[1])*N
************************************************************************************************/
void Data_BatchWriteMeterCmdLoadProc(DATA_FRAME_STRUCT *DataFrmPtr)
{
    uint8 *msg, *txDataBufPtr, *rxDataBufPtr;
    uint16 nodeId;
    METER_DATA_SAVE_FORMAT *meterBufPtr;
	uint8 meterDataLen, dataLen;

    dataLen = 0;
    if (RealTimeDataMode == Concentrator.Param.WorkType) {
        dataLen = REALTIME_DATA_AREA_SIZE - UPDOWN_RSSI_SIZE;
    } else if (FreezeDataMode == Concentrator.Param.WorkType) {
        dataLen = FREEZE_DATA_AREA_SIZE - UPDOWN_RSSI_SIZE;
    } else {
        DataFrmPtr->DataBuf[0] = OP_Failure;
        DataFrmPtr->DataLen = 1;
        return ;
    }

    if ((void *)0 == (msg = OSMemGetOpt(LargeMemoryPtr, 10, TIME_DELAY_MS(50)))) {
        DataFrmPtr->DataBuf[0] = OP_Failure;
        DataFrmPtr->DataLen = 1;
        return;
    }
    // 再申请一个内存,用于读取Eeprom中的数据
    if ((void *)0 == (meterBufPtr = OSMemGetOpt(SmallMemoryPtr, 10, TIME_DELAY_MS(50)))) {
        DataFrmPtr->DataBuf[0] = OP_Failure;
        DataFrmPtr->DataLen = 1;
        OSMemPut(LargeMemoryPtr, msg);
        return ;
    }
    meterDataLen = sizeof(METER_DATA_SAVE_FORMAT) - 1 + dataLen;

    memcpy(msg, DataFrmPtr->DataBuf, DataFrmPtr->DataLen);
    txDataBufPtr = DataFrmPtr->DataBuf;
    rxDataBufPtr = msg;
    while (rxDataBufPtr - msg < DataFrmPtr->DataLen) {
        nodeId = Data_FindNodeId(0, rxDataBufPtr);
        memcpy(txDataBufPtr, rxDataBufPtr, LONG_ADDR_SIZE);
        txDataBufPtr += LONG_ADDR_SIZE;
        rxDataBufPtr += LONG_ADDR_SIZE;
        if (NULL_U16_ID == nodeId) {
            *txDataBufPtr++ = OP_ObjectNotExist;
        } else {
            SubNodes[nodeId].CmdProp.Content = *rxDataBufPtr++;
            memcpy(&SubNodes[nodeId].CmdData, rxDataBufPtr, sizeof(CMD_DATA));

            Eeprom_ReadWrite((uint8 *)meterBufPtr, nodeId * NODE_INFO_SIZE, meterDataLen, READ_ATTR);
            if (0 != memcmp(SubNodes[nodeId].LongAddr, meterBufPtr->Address, LONG_ADDR_SIZE)) {
                Data_MeterDataInit(meterBufPtr, nodeId, meterDataLen);
            }

            if ( (meterBufPtr->CmdProp.Content != SubNodes[nodeId].CmdProp.Content) ||
                (0 == memcmp(&meterBufPtr->CmdData, &SubNodes[nodeId].CmdData, sizeof(CMD_DATA)))) {
                meterBufPtr->CmdProp.Content = SubNodes[nodeId].CmdProp.Content;
                memcpy(&meterBufPtr->CmdData, &SubNodes[nodeId].CmdData, sizeof(CMD_DATA));
                Eeprom_ReadWrite((uint8 *)meterBufPtr, nodeId * NODE_INFO_SIZE, meterDataLen, WRITE_ATTR);
            }

            rxDataBufPtr += sizeof(CMD_DATA);
            *txDataBufPtr++ = OP_Succeed;
        }
    }

	OSMemPut(SmallMemoryPtr, meterBufPtr);
    OSMemPut(LargeMemoryPtr, msg);
    Flash_SaveSubNodesInfo();
    DataFrmPtr->DataLen = txDataBufPtr - DataFrmPtr->DataBuf;
}


/************************************************************************************************
* Function Name: Data_GprsParameter
* Decription   : 读取或设置Gprs的参数信息
* Input        : DataFrmPtr-指向数据帧的指针
* Output       : 无
* Others       :
*   读下行:空数据域
*   读上行:IP(4)+PORT(2)+保留(1)+APN(n)+用户名(n)+密码(n)
*   写下行:IP(4)+PORT(2)+保留(1)+APN(n)+用户名(n)+密码(n)
*   写上行:操作状态(1)
************************************************************************************************/
void Data_GprsParameter(DATA_FRAME_STRUCT *DataFrmPtr)
{
    uint8 *msg, *dataBufPtr;
    uint16 dataLen;
    GPRS_PARAMETER *gprsParamPtr;

    dataBufPtr = DataFrmPtr->DataBuf;
    gprsParamPtr = &Concentrator.GprsParam;
    msg = OSMemGetOpt(SmallMemoryPtr, 10, TIME_DELAY_MS(50));
    if ((void *)0 != msg) {
        // 首选IP(4)+首选PORT(2)+备用IP(4)+备用PORT(2)+信息输出类型(1)+APN长度(1)+APN(N)+用户名长度(1)+用户名(N)+密码长度(1)+密码(N)
        memcpy(msg, gprsParamPtr->PriorDscIp, 4);
        ((uint16 *)(&msg[4]))[0] = gprsParamPtr->PriorDscPort;
        memcpy(&msg[6], gprsParamPtr->BackupDscIp, 4);
        ((uint16 *)(&msg[10]))[0] = gprsParamPtr->BackupDscPort;
        msg[12] = gprsParamPtr->HeatBeatPeriod;
        dataLen = 13;
        if ((msg[dataLen++] = strlen(gprsParamPtr->Apn)) > 0) {
            strcpy((char *)(msg + dataLen), gprsParamPtr->Apn);
            dataLen += msg[dataLen - 1];
        }
        if ((msg[dataLen++] = strlen(gprsParamPtr->Username)) > 0) {
            strcpy((char *)(msg + dataLen), gprsParamPtr->Username);
            dataLen += msg[dataLen - 1];
        }
        if ((msg[dataLen++] = strlen(gprsParamPtr->Password)) > 0) {
            strcpy((char *)(msg + dataLen), gprsParamPtr->Password);
            dataLen += msg[dataLen - 1];
        }
        if (Read_GPRS_Param == DataFrmPtr->Command) {
            // 下行:空数据域
            // 上行:首选IP(4)+首选PORT(2)+备用IP(4)+备用PORT(2)+保留(1)+APN(n)+用户名(n)+密码(n)
            memcpy(dataBufPtr, msg, dataLen);
        } else if(Write_GPRS_Param == DataFrmPtr->Command) {
            // 下行:首选IP(4)+首选PORT(2)+备用IP(4)+备用PORT(2)+保留(1)+APN长度(1)+APN(n)+用户名长度(1)+用户名(n)+密码长度(1)+密码(n)
            // 上行:操作状态(1)
            if (0 == memcmp(dataBufPtr, msg, dataLen) && dataLen == DataFrmPtr->DataLen) {
                *dataBufPtr = OP_Succeed;
            } else {
                memcpy(gprsParamPtr->PriorDscIp, dataBufPtr, 4);
                gprsParamPtr->PriorDscPort = *(uint16 *)(dataBufPtr + 4);
                memcpy(gprsParamPtr->BackupDscIp, dataBufPtr + 6, 4);
                gprsParamPtr->BackupDscPort = *(uint16 *)(dataBufPtr + 10);
                gprsParamPtr->HeatBeatPeriod = *(dataBufPtr + 12);
                if (gprsParamPtr->HeatBeatPeriod < 3) {
                    gprsParamPtr->HeatBeatPeriod = 3;
                }
                dataLen = 13;
                memset(gprsParamPtr->Apn, 0, sizeof(gprsParamPtr->Apn));
                memcpy(gprsParamPtr->Apn, dataBufPtr + dataLen + 1, *(dataBufPtr + dataLen));
                dataLen += *(dataBufPtr + dataLen) + 1;
                memset(gprsParamPtr->Username, 0, sizeof(gprsParamPtr->Username));
                memcpy(gprsParamPtr->Username, dataBufPtr + dataLen + 1, *(dataBufPtr + dataLen));
                dataLen += *(dataBufPtr + dataLen) + 1;
                memset(gprsParamPtr->Password, 0, sizeof(gprsParamPtr->Password));
                memcpy(gprsParamPtr->Password, dataBufPtr + dataLen + 1, *(dataBufPtr + dataLen));
                Flash_SaveConcentratorInfo();
                *dataBufPtr = OP_Succeed;
                DevResetTimer = 10000;      // 10秒钟后设备复位
            }
            dataLen = 1;
        }
        OSMemPut(SmallMemoryPtr, msg);
    } else {
        *dataBufPtr = OP_Failure;
        dataLen = 1;
    }
    DataFrmPtr->DataLen = dataLen;
}

/************************************************************************************************
* Function Name: Data_SwUpdate
* Decription   : 程序升级函数
* Input        : DataFrmPtr-指向数据帧的指针
* Output       : 无
* Others       : 下行: Crc16(2)+写入地址(4)+升级代码总长度(4)+本包升级代码长度(2)+升级代码(N)
*                上行: Crc16(2)+写入地址(4)+操作结果(1)
************************************************************************************************/
void Data_SwUpdate(DATA_FRAME_STRUCT *DataFrmPtr)
{
    uint16 crc16, pkgCodeLen;
    uint32 writeAddr, codeLength;
    uint8 *codeBufPtr, *dataBufPtr;
    uint8 buf[12];

    // 提取数据
    dataBufPtr = DataFrmPtr->DataBuf;
    crc16 = ((uint16 *)dataBufPtr)[0];
    writeAddr = ((uint32 *)(dataBufPtr + 2))[0];
    codeLength = ((uint32 *)(dataBufPtr + 6))[0];
    pkgCodeLen = ((uint16 *)(dataBufPtr + 10))[0];
    codeBufPtr = dataBufPtr + 12;

    // 如果升级代码长度错误
    if (codeLength > FLASH_UPGRADECODE_SIZE * FLASH_PAGE_SIZE) {
        *(dataBufPtr + 6) = OP_ParameterError;
        DataFrmPtr->DataLen = 7;
        return;
    }

    // 如果收到的写入地址为0,表示有一个新的升级要进行
    if (0 == writeAddr) {
        Flash_Erase(FLASH_UPGRADECODE_START_ADDR, FLASH_UPGRADECODE_SIZE);
        Flash_Erase(FLASH_UPGRADE_INFO_START, FLASH_UPGRADE_INFO_SIZE);
        // 升级信息保存格式: Crc16(2)+升级文件保存位置(4)+升级代码总长度(4)+Crc16(2)
        memcpy(buf, dataBufPtr, sizeof(buf));
        ((uint32 *)(&buf[2]))[0] = FLASH_UPGRADECODE_START_ADDR;
        ((uint16 *)(&buf[10]))[0] = CalCrc16(buf, 10);
        Flash_Write(buf, 16, FLASH_UPGRADE_INFO_START);
    }

    // 如果程序的校验字节或升级代码总长度错误则返回错误
    if (crc16 != ((uint16 *)FLASH_UPGRADE_INFO_START)[0] ||
        codeLength != ((uint32 *)(FLASH_UPGRADE_INFO_START + 6))[0]) {
        *(dataBufPtr + 6) = OP_ParameterError;
        DataFrmPtr->DataLen = 7;
        return;
    }

    // 写入升级代码
    if (codeLength >= writeAddr + pkgCodeLen) {
        if (0 != memcmp(codeBufPtr, (uint8 *)(FLASH_UPGRADECODE_START_ADDR + writeAddr), pkgCodeLen)) {
            Flash_Write(codeBufPtr, pkgCodeLen, FLASH_UPGRADECODE_START_ADDR + writeAddr);
            if (0 != memcmp(codeBufPtr, (uint8 *)(FLASH_UPGRADECODE_START_ADDR + writeAddr), pkgCodeLen)) {
                *(dataBufPtr + 6) = OP_Failure;
                DataFrmPtr->DataLen = 7;
                return;
            }
        }
    } else {
        *(dataBufPtr + 6) = OP_ParameterError;
        DataFrmPtr->DataLen = 7;
        return;
    }

    // 检查是否是最后一包
    if (writeAddr + pkgCodeLen >= codeLength) {
        if (crc16 == CalCrc16((uint8 *)FLASH_UPGRADECODE_START_ADDR, codeLength)) {
            *(dataBufPtr + 6) = OP_Succeed;
            DevResetTimer = 10000;      // 10秒钟后设备复位
        } else {
            *(dataBufPtr + 6) = OP_Failure;
        }
    } else {
        *(dataBufPtr + 6) = OP_Succeed;
    }
    DataFrmPtr->DataLen = 7;
    return;
}

/************************************************************************************************
* Function Name: Data_EepromCheckProc
* Decription   : Eeprom存储芯片检查程序
* Input        : DataFrmPtr-指向数据帧的指针
* Output       : 无
* Others       : 下行: 空数据域
*                上行: 操作结果(1)+Eeprom种类(1)+错误位置(4)
************************************************************************************************/
void Data_EepromCheckProc(DATA_FRAME_STRUCT *DataFrmPtr)
{
    uint8 i, retry, type, *msg, *buf, *dataBufPtr;
    uint16 j, err[2];
    uint32 addr;

    DataFrmPtr->DataLen = 6;
    dataBufPtr = DataFrmPtr->DataBuf;
#ifdef EEPROM_SEL_M512
    type = 0;
#endif
#ifdef EEPROM_SEL_M01
    type = 1;
#endif
    msg = OSMemGetOpt(LargeMemoryPtr, 10, TIME_DELAY_MS(50));
    buf = OSMemGetOpt(LargeMemoryPtr, 10, TIME_DELAY_MS(50));
    if ((void *)0 == msg || (void *)0 == buf) {
        *dataBufPtr++ = OP_Failure;
        *dataBufPtr++ = type;
        *dataBufPtr++ = 0xAA;
        *dataBufPtr++ = 0xAA;
        *dataBufPtr++ = 0xAA;
        *dataBufPtr++ = 0xAA;
    } else {
        addr = 0;
        while (addr < EEPROM_TOTALSIZE) {
            Led_FlashTime(WORKSTATUS_LED, Delay100ms, Delay100ms, FALSE);
            Eeprom_ReadWrite(msg, addr, MEM_LARGE_BLOCK_LEN, READ_ATTR);
            for (i = 0; i < 2; i++) {
                for (j = 0; j < MEM_LARGE_BLOCK_LEN; j++) {
                    *(msg + j) ^= 0xFF;
                }
                for (retry = 0; retry < 3; retry++) {
                    Eeprom_ReadWrite(msg, addr, MEM_LARGE_BLOCK_LEN, WRITE_ATTR);
                    Eeprom_ReadWrite(buf, addr, MEM_LARGE_BLOCK_LEN, READ_ATTR);
                    err[i] = 0xFFFF;
                    for (j = 0; j < MEM_LARGE_BLOCK_LEN; j++) {
                        if (*(msg + j) != *(buf + j)) {
                            err[i] = j;
                            break;
                        }
                    }
                    if (0xFFFF == err[i]) {
                        break;
                    }
                }
            }
            if (0xFFFF != err[0] || 0xFFFF != err[1]) {
                break;
            }
            addr += MEM_LARGE_BLOCK_LEN;
        }
        if (0xFFFF != err[0] || 0xFFFF != err[1]) {
            addr += err[0] < err[1] ? err[0] : err[1];
            *dataBufPtr++ = OP_Failure;
        } else {
            addr = 0xFFFFFFFF;
            *dataBufPtr++ = OP_Succeed;
        }
        *dataBufPtr++ = type;
        *dataBufPtr++ = (uint8)(addr >> 0);
        *dataBufPtr++ = (uint8)(addr >> 8);
        *dataBufPtr++ = (uint8)(addr >> 16);
        *dataBufPtr++ = (uint8)(addr >> 24);
    }
    Led_FlashTime(WORKSTATUS_LED, Delay100ms, Delay3000ms, FALSE);
    if ((void *)0 != msg) {
        OSMemPut(LargeMemoryPtr, msg);
    }
    if ((void *)0 != buf) {
        OSMemPut(LargeMemoryPtr, buf);
    }
}

/***************************************End of file*********************************************/

