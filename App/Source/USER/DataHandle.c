/************************************************************************************************
*                                   SRWF-6009
*    (c) Copyright 2015, Software Department, Sunray Technology Co.Ltd
*                               All Rights Reserved
*
* FileName     : DataHandle.c
* Description  :
* Version      :
* Function List:
*------------------------------Revision History--------------------------------------------------
* No.   Version     Date            Revised By      Item            Description
* 1     V1.1        08/12/2015      Zhangxp         SRWF-6009       Original Version
************************************************************************************************/

#define DATAHANDLE_GLOBALS

/************************************************************************************************
*                             Include File Section
************************************************************************************************/
#include "Stm32f10x_conf.h"
#include "ucos_ii.h"
#include "Bsp.h"
#include "Main.h"
#include "Rtc.h"
#include "Timer.h"
#include "SerialPort.h"
#include "Gprs.h"
#include "Flash.h"
#include "Eeprom.h"
#include "DataHandle.h"
#include "Database.h"
#include <string.h>

/************************************************************************************************
*                        Global Variable Declare Section
************************************************************************************************/
uint8 PkgNo;
PORT_NO MonitorPort = Usart_Debug;                      // 监控端口
uint8 SubNodesSaveDelayTimer = 0;                       // 档案延时保存时间
uint8 DataReplenishRound;                               // 数据补抄轮次
uint16 DataReplenishTimer = 30;                         // 数据补抄定时器
uint16 DataUploadTimer = 60;                            // 数据上传定时器
uint16 ModifyScanChannel  = 0;                          // 修改2E28扫描信道定时器
uint16 RTCTimingTimer = 60;                             // RTC校时任务启动定时器
TASK_STATUS_STRUCT TaskRunStatus;                       // 任务运行状态
DATA_HANDLE_TASK DataHandle_TaskArry[MAX_DATA_HANDLE_TASK_NUM];
const uint8 Uart_RfTx_Filter[] = {SYNCWORD1, SYNCWORD2};
const uint8 DayMaskTab[] = {0xF0, 0xE0, 0xC0, 0x80};
const uint8 ModifyScanChannel_KEY[] = {0x5A, 0x5B, 0x5C, 0x5D, 0x5E, 0x5F, 0x60};

extern void Gprs_OutputDebugMsg(bool NeedTime, uint8 *StrPtr);

#define PRINT_INFO    0
#define AUTO_CHANNEL 0

void DebugOutputLength(uint8 *StrPtr, uint8 SrcLength)
{
    uint16 len;
    uint8 *bufPtr;

    if ((void *)0 == (bufPtr = OSMemGetOpt(LargeMemoryPtr, 20, TIME_DELAY_MS(50)))) {
        return;
    }
    len = BcdToAscii( StrPtr, (uint8 *)bufPtr, SrcLength, 3);
    DataHandle_OutputMonitorMsg(Gprs_Connect_Msg, bufPtr, len);
    OSMemPut(LargeMemoryPtr, bufPtr);
    return;
}

/************************************************************************************************
*                           Prototype Declare Section
************************************************************************************************/
//void BigDataDebug(uint8 *BufPtr);

/************************************************************************************************
*                           Function Declare Section
************************************************************************************************/

/************************************************************************************************
* Function Name: DataHandle_GetEmptyTaskPtr
* Decription   : 在任务队列中搜索空的任务指针
* Input        : 无
* Output       : 任务的指针
* Others       : 无
************************************************************************************************/
DATA_HANDLE_TASK *DataHandle_GetEmptyTaskPtr(void)
{
    uint8 i;

    // 搜索未被占用的空间,创建数据上传任务
    for (i = 0; i < MAX_DATA_HANDLE_TASK_NUM; i++) {
        if ((void *)0 == DataHandle_TaskArry[i].StkPtr) {
            return (&DataHandle_TaskArry[i]);
        }
    }

    // 任务队列全部使用返回空队列
    return ((void *)0);
}

/************************************************************************************************
* Function Name: DataHandle_SetPkgProperty
* Decription   : 设置包属性值
* Input        : PkgXor-报文与运营商编码不异或标志: 0-不异或 1-异或
*                NeedAck-是否需要回执 0-不需回执 1-需要回执
*                PkgType-帧类型 0-命令帧 1-应答帧
*                Dir-上下行标识 0-下行 1-上行
* Output       : 属性值
* Others       : 无
************************************************************************************************/
PKG_PROPERTY DataHandle_SetPkgProperty(bool PkgXor, bool NeedAck, bool PkgType, bool Dir)
{
    PKG_PROPERTY pkgProp;

    pkgProp.Content = 0;
    pkgProp.PkgXor = PkgXor;//不再判断异或标志，将此bit变换为 crc8 和 crc16 的判断标志位。
    pkgProp.NeedAck = NeedAck;
    pkgProp.Encrypt = Concentrator.Param.DataEncryptCtrl;
    pkgProp.PkgType = PkgType;
    pkgProp.Direction = Dir;
    return pkgProp;
}

/************************************************************************************************
* Function Name: DataHandle_SetPkgPath
* Decription   : 设置数据包的路径
* Input        : DataFrmPtr-数据指针
*                ReversePath-是否需要翻转路径
* Output       : 无
* Others       : 无
************************************************************************************************/
void DataHandle_SetPkgPath(DATA_FRAME_STRUCT *DataFrmPtr, bool ReversePath)
{
    uint8 i, tmpBuf[LONG_ADDR_SIZE];

    if (0 == memcmp(BroadcastAddrIn, DataFrmPtr->Route[DataFrmPtr->RouteInfo.CurPos], LONG_ADDR_SIZE)) {
        memcpy(DataFrmPtr->Route[DataFrmPtr->RouteInfo.CurPos], Concentrator.LongAddr, LONG_ADDR_SIZE);
    }
    // 路径是否翻转处理
    if (REVERSED == ReversePath) {
        DataFrmPtr->RouteInfo.CurPos = DataFrmPtr->RouteInfo.Level - 1 - DataFrmPtr->RouteInfo.CurPos;
        for (i = 0; i < DataFrmPtr->RouteInfo.Level / 2; i++) {
            memcpy(tmpBuf, DataFrmPtr->Route[i], LONG_ADDR_SIZE);
            memcpy(DataFrmPtr->Route[i], DataFrmPtr->Route[DataFrmPtr->RouteInfo.Level - 1 - i], LONG_ADDR_SIZE);
            memcpy(DataFrmPtr->Route[DataFrmPtr->RouteInfo.Level - 1 - i], tmpBuf, LONG_ADDR_SIZE);
        }
    }
}

/************************************************************************************************
* Function Name: DataHandle_ExtractData
* Decription   : 按协议提取出数据并检验数据的正确性
* Input        : BufPtr-原数据指针
* Output       : 成功或错误说明
* Others       : 注意-成功调用此函数后BufPtr指向提取数据后的内存
************************************************************************************************/
EXTRACT_DATA_RESULT DataHandle_ExtractData(uint8 *BufPtr)
{
    uint8 i, *msg;
    uint16 tmp;
    PORT_BUF_FORMAT *portBufPtr;
    DATA_FRAME_STRUCT *dataFrmPtr;

    // 按协议格式提取相应的数据
    portBufPtr = (PORT_BUF_FORMAT *)BufPtr;
    if (FALSE == portBufPtr->Property.FilterDone) {
        return Error_Data;
    }
    // 申请一个内存用于存放提取后的数据
    if ((void *)0 == (msg = OSMemGetOpt(LargeMemoryPtr, 10, TIME_DELAY_MS(50)))) {
        return Error_GetMem;
    }
    dataFrmPtr = (DATA_FRAME_STRUCT *)msg;
    dataFrmPtr->PortNo = portBufPtr->Property.PortNo;
    dataFrmPtr->PkgLength = ((uint16 *)portBufPtr->Buffer)[0] & 0x03FF;
    dataFrmPtr->PkgProp.Content = portBufPtr->Buffer[2];
    dataFrmPtr->PkgSn = portBufPtr->Buffer[3];
    dataFrmPtr->Command = (COMMAND_TYPE)(portBufPtr->Buffer[4]);
    dataFrmPtr->DeviceType = portBufPtr->Buffer[5];
    dataFrmPtr->Life_Ack.Content = portBufPtr->Buffer[6];
    dataFrmPtr->RouteInfo.Content = portBufPtr->Buffer[7];
	memset(dataFrmPtr->Route[0], 0, (MAX_ROUTER_NUM+1)*LONG_ADDR_SIZE);
    for (i = 0; i < dataFrmPtr->RouteInfo.Level && i < MAX_ROUTER_NUM; i++) {
        memcpy(dataFrmPtr->Route[i], &portBufPtr->Buffer[8 + LONG_ADDR_SIZE * i], LONG_ADDR_SIZE);
    }

	if( XOR_CRC16 == dataFrmPtr->PkgProp.PkgXor){
		// crc16
		dataFrmPtr->DownRssi = *(portBufPtr->Buffer + dataFrmPtr->PkgLength- 5);
		dataFrmPtr->UpRssi = *(portBufPtr->Buffer + dataFrmPtr->PkgLength - 4);
		dataFrmPtr->Crc16 = ((portBufPtr->Buffer[dataFrmPtr->PkgLength - 2] << 8)&0xff00)|(portBufPtr->Buffer[dataFrmPtr->PkgLength - 3]&0xFF);
		tmp = LONG_ADDR_SIZE * dataFrmPtr->RouteInfo.Level + DATA_FIXED_AREA_LENGTH_CRC16;

		if (dataFrmPtr->PkgLength < tmp || dataFrmPtr->PkgLength > MEM_LARGE_BLOCK_LEN - 1) {
			OSMemPut(LargeMemoryPtr, msg);
			return Error_DataLength;
		}
		dataFrmPtr->DataLen = dataFrmPtr->PkgLength - tmp;
		if (dataFrmPtr->DataLen < MEM_LARGE_BLOCK_LEN - sizeof(DATA_FRAME_STRUCT)) {
			memcpy(dataFrmPtr->DataBuf, portBufPtr->Buffer + 8 + LONG_ADDR_SIZE * dataFrmPtr->RouteInfo.Level, dataFrmPtr->DataLen);
		} else {
			OSMemPut(LargeMemoryPtr, msg);
			return Error_DataOverFlow;
		}

		// 检查Crc16是否正确
		if (dataFrmPtr->Crc16 != CalCrc16(portBufPtr->Buffer, dataFrmPtr->PkgLength - 3) || portBufPtr->Length < dataFrmPtr->PkgLength) {
			OSMemPut(LargeMemoryPtr, msg);
			return Error_DataCrcCheck;
		}
	} else {
		// crc8
		dataFrmPtr->DownRssi = *(portBufPtr->Buffer + dataFrmPtr->PkgLength- 4);
		dataFrmPtr->UpRssi = *(portBufPtr->Buffer + dataFrmPtr->PkgLength - 3);
		dataFrmPtr->Crc8 = *(portBufPtr->Buffer + dataFrmPtr->PkgLength - 2);
		tmp = LONG_ADDR_SIZE * dataFrmPtr->RouteInfo.Level + DATA_FIXED_AREA_LENGTH_CRC8;
		if (dataFrmPtr->PkgLength < tmp || dataFrmPtr->PkgLength > MEM_LARGE_BLOCK_LEN - 1) {
			OSMemPut(LargeMemoryPtr, msg);
			return Error_DataLength;
		}
		dataFrmPtr->DataLen = dataFrmPtr->PkgLength - tmp;
		if (dataFrmPtr->DataLen < MEM_LARGE_BLOCK_LEN - sizeof(DATA_FRAME_STRUCT)) {
			memcpy(dataFrmPtr->DataBuf, portBufPtr->Buffer + 8 + LONG_ADDR_SIZE * dataFrmPtr->RouteInfo.Level, dataFrmPtr->DataLen);
		} else {
			OSMemPut(LargeMemoryPtr, msg);
			return Error_DataOverFlow;
		}

		// 检查Crc8是否正确
		if (dataFrmPtr->Crc8 != CalCrc8(portBufPtr->Buffer, dataFrmPtr->PkgLength - 2) || portBufPtr->Length < dataFrmPtr->PkgLength) {
			OSMemPut(LargeMemoryPtr, msg);
			return Error_DataCrcCheck;
		}
	}

    // 检查结束符是否是 0x16
    if ( 0x16 != *(portBufPtr->Buffer + dataFrmPtr->PkgLength - 1)) {
        OSMemPut(LargeMemoryPtr, msg);
        return Error_Data;
    }

    // 检查是否为广播地址或本机地址
    dataFrmPtr->RouteInfo.CurPos += 1;
    if ((0 == memcmp(Concentrator.LongAddr, dataFrmPtr->Route[dataFrmPtr->RouteInfo.CurPos], LONG_ADDR_SIZE) ||
        0 == memcmp(BroadcastAddrIn, dataFrmPtr->Route[dataFrmPtr->RouteInfo.CurPos], LONG_ADDR_SIZE)) &&
        dataFrmPtr->RouteInfo.CurPos < dataFrmPtr->RouteInfo.Level) {
        memcpy(BufPtr, msg, MEM_LARGE_BLOCK_LEN);
        OSMemPut(LargeMemoryPtr, msg);
        return Ok_Data;
    }

    // 要进行后续处理,所以此处将提取出的数据返回
    memcpy(BufPtr, msg, MEM_LARGE_BLOCK_LEN);
    OSMemPut(LargeMemoryPtr, msg);
    return Error_DstAddress;
}


/************************************************************************************************
* Function Name: Test_DataHandle_CreateTxData
* Decription   : 创建发送数据包
* Input        : DataFrmPtr-待发送的数据
* Output       : 成功或错误
* Others       : 该函数执行完毕后会释放DataBufPtr指向的存储区,还会将路由区的地址排列翻转
************************************************************************************************/
ErrorStatus Test_DataHandle_CreateTxData(DATA_FRAME_STRUCT *DataFrmPtr, uint8 version)
{
    uint8 err;
    uint16 tmp, nodeId;
    PORT_BUF_FORMAT *txPortBufPtr;
    uint8 ackChannel = 0;

    // 先申请一个内存用于中间数据处理
    if ((void *)0 == (txPortBufPtr = OSMemGetOpt(LargeMemoryPtr, 10, TIME_DELAY_MS(50)))) {
        OSMemPut(LargeMemoryPtr, DataFrmPtr);
        return ERROR;
    }
    txPortBufPtr->Property.PortNo = DataFrmPtr->PortNo;
    txPortBufPtr->Property.FilterDone = 1;
    memcpy(txPortBufPtr->Buffer, Uart_RfTx_Filter, sizeof(Uart_RfTx_Filter));
    txPortBufPtr->Length = sizeof(Uart_RfTx_Filter);

    tmp = txPortBufPtr->Length;
	if( XOR_CRC16 == DataFrmPtr->PkgProp.PkgXor){
    	DataFrmPtr->PkgLength = DataFrmPtr->DataLen + DataFrmPtr->RouteInfo.Level * LONG_ADDR_SIZE + DATA_FIXED_AREA_LENGTH_CRC16;
	}else{
    	DataFrmPtr->PkgLength = DataFrmPtr->DataLen + DataFrmPtr->RouteInfo.Level * LONG_ADDR_SIZE + DATA_FIXED_AREA_LENGTH_CRC8;
	}
    ((uint16 *)(&txPortBufPtr->Buffer[txPortBufPtr->Length]))[0] = DataFrmPtr->PkgLength;
    txPortBufPtr->Length += 2;
    txPortBufPtr->Buffer[txPortBufPtr->Length++] = DataFrmPtr->PkgProp.Content;         // 报文标识
    txPortBufPtr->Buffer[txPortBufPtr->Length++] = DataFrmPtr->PkgSn;                   // 任务号
    txPortBufPtr->Buffer[txPortBufPtr->Length++] = DataFrmPtr->Command;                 // 命令字
    // 为了和已出货的第一批表兼容
    if (0 == DataFrmPtr->Life_Ack.AckChannel) {
        txPortBufPtr->Buffer[txPortBufPtr->Length++] = Dev_Server;                      // 设备类型
    } else {
        txPortBufPtr->Buffer[txPortBufPtr->Length++] = DataFrmPtr->DeviceType;          // 设备类型
    }
    txPortBufPtr->Buffer[txPortBufPtr->Length++] = DataFrmPtr->Life_Ack.Content;        // 生命周期和应答信道
    txPortBufPtr->Buffer[txPortBufPtr->Length++] = DataFrmPtr->RouteInfo.Content;       // 路径信息
    memcpy(&txPortBufPtr->Buffer[txPortBufPtr->Length], DataFrmPtr->Route[0], DataFrmPtr->RouteInfo.Level * LONG_ADDR_SIZE);
    txPortBufPtr->Length += DataFrmPtr->RouteInfo.Level * LONG_ADDR_SIZE;
    memcpy(txPortBufPtr->Buffer + txPortBufPtr->Length, DataFrmPtr->DataBuf, DataFrmPtr->DataLen);      // 数据域
    txPortBufPtr->Length += DataFrmPtr->DataLen;
    txPortBufPtr->Buffer[txPortBufPtr->Length++] = 0x55;                                // 下行信号强度
    txPortBufPtr->Buffer[txPortBufPtr->Length++] = 0x55;                                // 上行信号强度
	if( XOR_CRC16 == DataFrmPtr->PkgProp.PkgXor){
		// crc16校验，低字节在前
	    uint16 crc16 = CalCrc16((uint8 *)(&txPortBufPtr->Buffer[tmp]), txPortBufPtr->Length-tmp);
	    txPortBufPtr->Buffer[txPortBufPtr->Length++] = (uint8)((crc16)&0xFF);
	    txPortBufPtr->Buffer[txPortBufPtr->Length++] = (uint8)((crc16 >> 8)&0xFF);
	} else {
		txPortBufPtr->Buffer[txPortBufPtr->Length] = CalCrc8((uint8 *)(&txPortBufPtr->Buffer[tmp]), txPortBufPtr->Length - tmp);	 // Crc8校验
		txPortBufPtr->Length += 1;
	}
    txPortBufPtr->Buffer[txPortBufPtr->Length++] = TAILBYTE;

	nodeId = Data_FindNodeId(0, DataFrmPtr->Route[DataFrmPtr->RouteInfo.Level - 1]);

    txPortBufPtr->Buffer[txPortBufPtr->Length++] = 0x00;
    // 为了和已出货的第一批表兼容
    if (0 == DataFrmPtr->Life_Ack.AckChannel) {
        if ( DataFrmPtr->RouteInfo.Level > 0x2 && version > 0x3){
            // 多级路由情况下 , 且版本号高于 3.
            ackChannel = (DataFrmPtr->Route[DataFrmPtr->RouteInfo.CurPos+1][LONG_ADDR_SIZE-1] & 0x0F)/2 + 0x0B;
            txPortBufPtr->Buffer[txPortBufPtr->Length++] = ackChannel;
        } else {
            txPortBufPtr->Buffer[txPortBufPtr->Length++] = (DEFAULT_RX_CHANNEL + CHANNEL_OFFSET);
        }
		txPortBufPtr->Buffer[txPortBufPtr->Length++] = (DEFAULT_RX_CHANNEL + CHANNEL_OFFSET);
    } else {
        if ( DataFrmPtr->RouteInfo.Level > 0x2 && version > 0x3 && SubNodes[nodeId].AutoChannelSwitch == TRUE ){
            // 多级路由情况下 , 且版本号高于 3.
            ackChannel = (DataFrmPtr->Route[DataFrmPtr->RouteInfo.CurPos+1][LONG_ADDR_SIZE-1] & 0x0F)/2 + 0x0B;
            txPortBufPtr->Buffer[txPortBufPtr->Length++] = ackChannel;
			txPortBufPtr->Buffer[txPortBufPtr->Length++] = DataFrmPtr->Route[DataFrmPtr->RouteInfo.CurPos+1][LONG_ADDR_SIZE-1] & 0x0F;
        } else if( DataFrmPtr->RouteInfo.Level > 0x2 && version > 0x3 && SubNodes[nodeId].AutoChannelSwitch == FALSE ){
            ackChannel = (DataFrmPtr->Route[DataFrmPtr->RouteInfo.CurPos+1][LONG_ADDR_SIZE-1] & 0x0F)/2 + 0x0B;
            txPortBufPtr->Buffer[txPortBufPtr->Length++] = ackChannel;
			txPortBufPtr->Buffer[txPortBufPtr->Length++] = DEFAULT_TX_CHANNEL;
		} else {
            txPortBufPtr->Buffer[txPortBufPtr->Length++] = DataFrmPtr->Life_Ack.AckChannel;
			txPortBufPtr->Buffer[txPortBufPtr->Length++] = (DEFAULT_RX_CHANNEL + CHANNEL_OFFSET);
		}
    }

    OSMemPut(LargeMemoryPtr, DataFrmPtr);
    if (Uart_Gprs == txPortBufPtr->Property.PortNo) {
        if (FALSE == Gprs.Online ||
            OS_ERR_NONE != OSMboxPost(Gprs.MboxTx, txPortBufPtr)) {
            OSMemPut(LargeMemoryPtr, txPortBufPtr);
            return ERROR;
        } else {
            OSFlagPost(GlobalEventFlag, FLAG_GPRS_TX, OS_FLAG_SET, &err);
            return SUCCESS;
        }
    } else {
        if (txPortBufPtr->Property.PortNo < Port_Total &&
            OS_ERR_NONE != OSMboxPost(SerialPort.Port[txPortBufPtr->Property.PortNo].MboxTx, txPortBufPtr)) {
            OSMemPut(LargeMemoryPtr, txPortBufPtr);
            return ERROR;
        } else {
            OSFlagPost(GlobalEventFlag, (OS_FLAGS)(1 << txPortBufPtr->Property.PortNo + SERIALPORT_TX_FLAG_OFFSET), OS_FLAG_SET, &err);
            return SUCCESS;
        }
    }
}


/************************************************************************************************
* Function Name: DataHandle_CreateTxData
* Decription   : 创建发送数据包
* Input        : DataFrmPtr-待发送的数据
* Output       : 成功或错误
* Others       : 该函数执行完毕后会释放DataBufPtr指向的存储区,还会将路由区的地址排列翻转
************************************************************************************************/
ErrorStatus DataHandle_CreateTxData(DATA_FRAME_STRUCT *DataFrmPtr, uint8 version)
{
    uint8 err;
    uint16 tmp, nodeId;
    PORT_BUF_FORMAT *txPortBufPtr;
    uint8 ackChannel = 0;

    // 先申请一个内存用于中间数据处理
    if ((void *)0 == (txPortBufPtr = OSMemGetOpt(LargeMemoryPtr, 10, TIME_DELAY_MS(50)))) {
        OSMemPut(LargeMemoryPtr, DataFrmPtr);
        return ERROR;
    }
    txPortBufPtr->Property.PortNo = DataFrmPtr->PortNo;
    txPortBufPtr->Property.FilterDone = 1;
    memcpy(txPortBufPtr->Buffer, Uart_RfTx_Filter, sizeof(Uart_RfTx_Filter));
    txPortBufPtr->Length = sizeof(Uart_RfTx_Filter);

    tmp = txPortBufPtr->Length;
	if( XOR_CRC16 == DataFrmPtr->PkgProp.PkgXor){
    	DataFrmPtr->PkgLength = DataFrmPtr->DataLen + DataFrmPtr->RouteInfo.Level * LONG_ADDR_SIZE + DATA_FIXED_AREA_LENGTH_CRC16;
	}else{
    	DataFrmPtr->PkgLength = DataFrmPtr->DataLen + DataFrmPtr->RouteInfo.Level * LONG_ADDR_SIZE + DATA_FIXED_AREA_LENGTH_CRC8;
	}
    ((uint16 *)(&txPortBufPtr->Buffer[txPortBufPtr->Length]))[0] = DataFrmPtr->PkgLength;
    txPortBufPtr->Length += 2;
    txPortBufPtr->Buffer[txPortBufPtr->Length++] = DataFrmPtr->PkgProp.Content;         // 报文标识
    txPortBufPtr->Buffer[txPortBufPtr->Length++] = DataFrmPtr->PkgSn;                   // 任务号
    txPortBufPtr->Buffer[txPortBufPtr->Length++] = DataFrmPtr->Command;                 // 命令字
    // 为了和已出货的第一批表兼容
    if (0 == DataFrmPtr->Life_Ack.AckChannel) {
        txPortBufPtr->Buffer[txPortBufPtr->Length++] = Dev_Server;                      // 设备类型
    } else {
        txPortBufPtr->Buffer[txPortBufPtr->Length++] = DataFrmPtr->DeviceType;          // 设备类型
    }
    txPortBufPtr->Buffer[txPortBufPtr->Length++] = DataFrmPtr->Life_Ack.Content;        // 生命周期和应答信道
    txPortBufPtr->Buffer[txPortBufPtr->Length++] = DataFrmPtr->RouteInfo.Content;       // 路径信息
    memcpy(&txPortBufPtr->Buffer[txPortBufPtr->Length], DataFrmPtr->Route[0], DataFrmPtr->RouteInfo.Level * LONG_ADDR_SIZE);
    txPortBufPtr->Length += DataFrmPtr->RouteInfo.Level * LONG_ADDR_SIZE;
    memcpy(txPortBufPtr->Buffer + txPortBufPtr->Length, DataFrmPtr->DataBuf, DataFrmPtr->DataLen);      // 数据域
    txPortBufPtr->Length += DataFrmPtr->DataLen;
    txPortBufPtr->Buffer[txPortBufPtr->Length++] = 0x55;                                // 下行信号强度
    txPortBufPtr->Buffer[txPortBufPtr->Length++] = 0x55;                                // 上行信号强度
	if( XOR_CRC16 == DataFrmPtr->PkgProp.PkgXor){
		// crc16校验，低字节在前
	    uint16 crc16 = CalCrc16((uint8 *)(&txPortBufPtr->Buffer[tmp]), txPortBufPtr->Length-tmp);
	    txPortBufPtr->Buffer[txPortBufPtr->Length++] = (uint8)((crc16)&0xFF);
	    txPortBufPtr->Buffer[txPortBufPtr->Length++] = (uint8)((crc16 >> 8)&0xFF);
	} else {
		txPortBufPtr->Buffer[txPortBufPtr->Length] = CalCrc8((uint8 *)(&txPortBufPtr->Buffer[tmp]), txPortBufPtr->Length - tmp);	 // Crc8校验
		txPortBufPtr->Length += 1;
	}
    txPortBufPtr->Buffer[txPortBufPtr->Length++] = TAILBYTE;

	nodeId = Data_FindNodeId(0, DataFrmPtr->Route[DataFrmPtr->RouteInfo.Level - 1]);

    if (CMD_PKG == DataFrmPtr->PkgProp.PkgType) {
        txPortBufPtr->Buffer[txPortBufPtr->Length++] = 0x1E;
        if (DATA_CENTER_ID == nodeId || NULL_U16_ID == nodeId) {
            if ( version > 0x3 && SubNodes[nodeId].AutoChannelSwitch == TRUE ){
                // 多级路由情况下
                ackChannel = DataFrmPtr->Route[DataFrmPtr->RouteInfo.CurPos+1][LONG_ADDR_SIZE-1] & 0xF;
                txPortBufPtr->Buffer[txPortBufPtr->Length++] = ackChannel;
            }else{
                txPortBufPtr->Buffer[txPortBufPtr->Length++] = DEFAULT_TX_CHANNEL;
            }
        } else {
            if ( (version > 0x3 && SubNodes[nodeId].AutoChannelSwitch == TRUE) || version == 0x44 ){
                // 多级路由情况下
                ackChannel = DataFrmPtr->Route[DataFrmPtr->RouteInfo.CurPos+1][LONG_ADDR_SIZE-1] & 0xF;
                txPortBufPtr->Buffer[txPortBufPtr->Length++] = ackChannel;
            }else{
                txPortBufPtr->Buffer[txPortBufPtr->Length++] = SubNodes[nodeId].RxChannel;
            }
        }
        txPortBufPtr->Buffer[txPortBufPtr->Length++] = (DEFAULT_RX_CHANNEL + CHANNEL_OFFSET);
    } else {
        txPortBufPtr->Buffer[txPortBufPtr->Length++] = 0x00;
        // 为了和已出货的第一批表兼容
        if (0 == DataFrmPtr->Life_Ack.AckChannel) {
            if ( DataFrmPtr->RouteInfo.Level > 0x2 && version > 0x3){
                // 多级路由情况下 , 且版本号高于 3.
                ackChannel = (DataFrmPtr->Route[DataFrmPtr->RouteInfo.CurPos+1][LONG_ADDR_SIZE-1] & 0x0F)/2 + 0x0B;
                txPortBufPtr->Buffer[txPortBufPtr->Length++] = ackChannel;
            } else {
                txPortBufPtr->Buffer[txPortBufPtr->Length++] = (DEFAULT_RX_CHANNEL + CHANNEL_OFFSET);
            }
        } else {
            if ( DataFrmPtr->RouteInfo.Level > 0x2 && version > 0x3 && SubNodes[nodeId].AutoChannelSwitch == TRUE ){
                // 多级路由情况下 , 且版本号高于 3.
                ackChannel = (DataFrmPtr->Route[DataFrmPtr->RouteInfo.CurPos+1][LONG_ADDR_SIZE-1] & 0x0F)/2 + 0x0B;
                txPortBufPtr->Buffer[txPortBufPtr->Length++] = ackChannel;
            } else {
                txPortBufPtr->Buffer[txPortBufPtr->Length++] = DataFrmPtr->Life_Ack.AckChannel;
            }
        }
        txPortBufPtr->Buffer[txPortBufPtr->Length++] = (DEFAULT_RX_CHANNEL + CHANNEL_OFFSET);
    }

    OSMemPut(LargeMemoryPtr, DataFrmPtr);
    if (Uart_Gprs == txPortBufPtr->Property.PortNo) {
        if (FALSE == Gprs.Online ||
            OS_ERR_NONE != OSMboxPost(Gprs.MboxTx, txPortBufPtr)) {
            OSMemPut(LargeMemoryPtr, txPortBufPtr);
            return ERROR;
        } else {
            OSFlagPost(GlobalEventFlag, FLAG_GPRS_TX, OS_FLAG_SET, &err);
            return SUCCESS;
        }
    } else {
        if (txPortBufPtr->Property.PortNo < Port_Total &&
            OS_ERR_NONE != OSMboxPost(SerialPort.Port[txPortBufPtr->Property.PortNo].MboxTx, txPortBufPtr)) {
            OSMemPut(LargeMemoryPtr, txPortBufPtr);
            return ERROR;
        } else {
            OSFlagPost(GlobalEventFlag, (OS_FLAGS)(1 << txPortBufPtr->Property.PortNo + SERIALPORT_TX_FLAG_OFFSET), OS_FLAG_SET, &err);
            return SUCCESS;
        }
    }
}

// ****用于大数据调试
/*
void BigDataDebug(uint8 *BufPtr)
{
    PORT_BUF_FORMAT *portBufPtr;
    DATA_FRAME_STRUCT *DataFrmPtr;
    uint16 len;
    uint8 *p;

    portBufPtr = (PORT_BUF_FORMAT *)BufPtr;
    len = portBufPtr->Buffer[0] + portBufPtr->Buffer[1] * 256;
    p = portBufPtr->Buffer + len - 4 - 2;
    if (0x39 == *p) {
        return;
    }
    if ((void *)0 == (DataFrmPtr = OSMemGetOpt(LargeMemoryPtr, 10, TIME_DELAY_MS(50)))) {
        return;
    }
    DataFrmPtr->PortNo = Uart_Gprs;
    DataFrmPtr->PkgLength = DATA_FIXED_AREA_LENGTH;
    DataFrmPtr->PkgSn = PkgNo++;
    DataFrmPtr->DeviceType = Dev_Concentrator;
    DataFrmPtr->Life_Ack.Content = 0x0F;
    DataFrmPtr->RouteInfo.CurPos = 0;
    DataFrmPtr->RouteInfo.Level = 2;
    memcpy(DataFrmPtr->Route[0], Concentrator.LongAddr, LONG_ADDR_SIZE);
    memcpy(DataFrmPtr->Route[1], BroadcastAddrOut, LONG_ADDR_SIZE);
    DataFrmPtr->DataLen = 2 + len;
    DataFrmPtr->DataBuf[0] = 0xEE;
    DataFrmPtr->DataBuf[1] = 0xEE;
    memcpy(DataFrmPtr->DataBuf + 2, portBufPtr->Buffer, len);
    DataFrmPtr->PkgProp = DataHandle_SetPkgProperty(XOR_CRC8, NONE_ACK, CMD_PKG, UP_DIR);
    DataHandle_SetPkgPath(DataFrmPtr, UNREVERSED);
    DataHandle_CreateTxData(DataFrmPtr);
}
*/
// ****用于大数据调试

/************************************************************************************************
* Function Name: DataHandle_DataDelaySaveProc
* Decription   : 数据延时保存处理函数
* Input        : 无
* Output       : 无
* Others       : 当多组数据需要保存时启动一次延时保存,以延长Flash的寿命
************************************************************************************************/
void DataHandle_DataDelaySaveProc(void)
{
    SubNodesSaveDelayTimer = 0;
    Flash_SaveSubNodesInfo();
    Flash_SaveConcentratorInfo();
}

/************************************************************************************************
* Function Name: DataHandle_OutputMonitorMsg
* Decription   : 集中器主动输出监控信息
* Input        : MsgType-信息的类型,MsgPtr-输出信息指针,MsgLen-信息的长度
* Output       : 无
* Others       : 无
************************************************************************************************/
void DataHandle_OutputMonitorMsg(MONITOR_MSG_TYPE MsgType, uint8 *MsgPtr, uint16 MsgLen)
{
    DATA_FRAME_STRUCT *dataFrmPtr;

    if ((void *)0 == (dataFrmPtr = OSMemGetOpt(LargeMemoryPtr, 20, TIME_DELAY_MS(50)))) {
        return;
    }
    dataFrmPtr->PortNo = MonitorPort;
    dataFrmPtr->PkgProp = DataHandle_SetPkgProperty(XOR_CRC8, NONE_ACK, CMD_PKG, UP_DIR);
    dataFrmPtr->PkgSn = PkgNo++;
    dataFrmPtr->Command = Output_Monitior_Msg_Cmd;
    dataFrmPtr->DeviceType = Dev_Concentrator;
    dataFrmPtr->Life_Ack.Content = 0x0F;
    dataFrmPtr->RouteInfo.CurPos = 0;
    dataFrmPtr->RouteInfo.Level = 2;
    memcpy(dataFrmPtr->Route[0], Concentrator.LongAddr, LONG_ADDR_SIZE);
    memcpy(dataFrmPtr->Route[1], BroadcastAddrOut, LONG_ADDR_SIZE);
    dataFrmPtr->DataBuf[0] = MsgType;
    memcpy(&dataFrmPtr->DataBuf[1], MsgPtr, MsgLen);
    dataFrmPtr->DataLen = 1 + MsgLen;
    DataHandle_SetPkgPath(dataFrmPtr, UNREVERSED);
    DataHandle_CreateTxData(dataFrmPtr, NULL);
    return;
}

uint32 Byte4ToUint32(uint8 *bs)
{
	return (uint32)((bs[3]<<24)+(bs[2]<<16) + (bs[1]<<8) + bs[0]);
}

/************************************************************************************************
* Function Name: DataHandle_MeterDataSaveProc
* Decription   : 抄表数据保存处理函数
* Input        : DataFrmPtr-指向数据帧的指针
* Output       : 成功或失败
* Others       : 用于保存实时/定时/定量和冻结数据和上下行信号强度
************************************************************************************************/
bool DataHandle_MeterDataSaveProc(DATA_FRAME_STRUCT *DataFrmPtr)
{
    uint8 length, dataLen, meterDataLen, *msg, MeterVersion;
    uint16 nodeId;
    METER_DATA_SAVE_FORMAT *meterBufPtr;
    RTC_TIME rtcTimer;
    uint32 RtcTemp, MeterDataTemp;
    uint8 ErrorData[10];

    memset(ErrorData, 0xFF, 10);
    nodeId = Data_FindNodeId(0, DataFrmPtr->Route[0]);
    if (NULL_U16_ID == nodeId) {
        return ERROR;
    }
    if (DataFrmPtr->Command != Meter_ActiveReport_Data &&
        DataFrmPtr->Command != Read_Meter_Data &&
        DataFrmPtr->Command != Read_Meter_Freeze_Data) {
        return ERROR;
    }
    // 申请空间用于保存抄表数据
    if ((void *)0 == (msg = OSMemGetOpt(SmallMemoryPtr, 10, TIME_DELAY_MS(50)))) {
        return ERROR;
    }
    length = DataFrmPtr->DataLen - 1;       // 减去数据格式编号字节
    Rtc_Get(&rtcTimer, Format_Bcd);
    RtcTemp = RTC_GetCounter();
    dataLen = 0;
    if (RealTimeDataMode == Concentrator.Param.WorkType && length + 5 + UPDOWN_RSSI_SIZE == REALTIME_DATA_AREA_SIZE) {
        *(msg + 0) = (uint8)(rtcTimer.Year);
        *(msg + 1) = (uint8)(rtcTimer.Year >> 8);
        *(msg + 2) = rtcTimer.Month;
        *(msg + 3) = rtcTimer.Day;
        *(msg + 4) = rtcTimer.Hour;
        memcpy(msg + 5, DataFrmPtr->DataBuf + 1, REALTIME_DATA_AREA_SIZE - 5 - UPDOWN_RSSI_SIZE);
        dataLen = REALTIME_DATA_AREA_SIZE - UPDOWN_RSSI_SIZE;
    } else if (FreezeDataMode == Concentrator.Param.WorkType && length + UPDOWN_RSSI_SIZE == FREEZE_DATA_AREA_SIZE) {
        memcpy(msg, DataFrmPtr->DataBuf + 1, FREEZE_DATA_AREA_SIZE - UPDOWN_RSSI_SIZE);
        dataLen = FREEZE_DATA_AREA_SIZE - UPDOWN_RSSI_SIZE;
    } else {
        OSMemPut(SmallMemoryPtr, msg);
        return ERROR;
    }
    *(msg + dataLen++) = DataFrmPtr->DownRssi;
    *(msg + dataLen++) = DataFrmPtr->UpRssi;

    // 再申请一个内存,用于读取Eeprom中的数据
    if ((void *)0 == (meterBufPtr = OSMemGetOpt(SmallMemoryPtr, 10, TIME_DELAY_MS(50)))) {
        OSMemPut(SmallMemoryPtr, msg);
        return ERROR;
    }
    meterDataLen = sizeof(METER_DATA_SAVE_FORMAT) - 1 + dataLen;
    Eeprom_ReadWrite((uint8 *)meterBufPtr, nodeId * NODE_INFO_SIZE, meterDataLen, READ_ATTR);
    if (0 != memcmp(SubNodes[nodeId].LongAddr, meterBufPtr->Address, LONG_ADDR_SIZE)) {
        Data_MeterDataInit(meterBufPtr, nodeId, meterDataLen);
    }
    meterBufPtr->Property.CurRouteNo = SubNodes[nodeId].Property.CurRouteNo;
    meterBufPtr->Property.LastResult = SubNodes[nodeId].Property.LastResult = 1;
	if(0 == memcmp(DataFrmPtr->DataBuf+2, ErrorData, 10)){
		meterBufPtr->Property.LastResult = SubNodes[nodeId].Property.LastResult = 2;
	}
    // 该节点的收发信道位于数据末尾第4个字节(要考虑到收发信道的问题)
    SubNodes[nodeId].RxChannel = (uint8)(*(msg + dataLen - UPDOWN_RSSI_SIZE - 2) >> 4);
    SubNodes[nodeId].TxChannel = (uint8)(*(msg + dataLen - UPDOWN_RSSI_SIZE - 2)&0x0F);

	// 表端版本号，位于数据末尾第 3 个字节(要考虑到收发信道的问题)
	MeterVersion = (uint8)(*(msg + dataLen - UPDOWN_RSSI_SIZE - 1));

    MeterDataTemp = Byte4ToUint32(&meterBufPtr->RxMeterDataTemp[0]);

    if ((0 != memcmp(meterBufPtr->MeterData, msg, dataLen)) ||
        (meterBufPtr->RxMeterDataDay != rtcTimer.Day) ||
        (RtcTemp - MeterDataTemp >= 23*60*60) ) {
        meterBufPtr->Crc8MeterData = CalCrc8(msg, dataLen);
        memcpy(meterBufPtr->MeterData, msg, dataLen);
        meterBufPtr->RxMeterDataDay = SubNodes[nodeId].RxMeterDataDay = rtcTimer.Day;

		meterBufPtr->CmdProp.Content = SubNodes[nodeId].CmdProp.Content;
		memcpy(&meterBufPtr->CmdData, &SubNodes[nodeId].CmdData, sizeof(CMD_DATA));

        meterBufPtr->RxChannel = SubNodes[nodeId].RxChannel;
        meterBufPtr->TxChannel = SubNodes[nodeId].TxChannel;
        meterBufPtr->RxMeterVersion = SubNodes[nodeId].RxMeterVersion = MeterVersion;

		// 记录当前节点抄表时间
        meterBufPtr->RxMeterDataTemp[0] = (uint8)(RtcTemp & 0xFF);
        meterBufPtr->RxMeterDataTemp[1] = (uint8)((RtcTemp & 0xFF00)>>8);
        meterBufPtr->RxMeterDataTemp[2] = (uint8)((RtcTemp & 0xFF0000)>>16);
        meterBufPtr->RxMeterDataTemp[3] = (uint8)((RtcTemp>>24) & 0xFF);
        memcpy(SubNodes[nodeId].RxMeterDataTemp, meterBufPtr->RxMeterDataTemp, 4);

        meterBufPtr->Property.UploadData = SubNodes[nodeId].Property.UploadData = FALSE;
        meterBufPtr->Property.UploadPrio = SubNodes[nodeId].Property.UploadPrio = LOW;
        if ((RealTimeAlarmData == DataFrmPtr->DataBuf[0]) ||
            (FwFreezingAlarmData == DataFrmPtr->DataBuf[0])) {
            meterBufPtr->Property.UploadPrio = SubNodes[nodeId].Property.UploadPrio = HIGH;
            DataUploadTimer = 2;
        }
        Eeprom_ReadWrite((uint8 *)meterBufPtr, nodeId * NODE_INFO_SIZE, meterDataLen, WRITE_ATTR);
    }
    OSMemPut(SmallMemoryPtr, msg);
    OSMemPut(SmallMemoryPtr, meterBufPtr);
    return SUCCESS;
}

/************************************************************************************************
* Function Name: DataHandle_MeterDataLoadProc
* Decription   : 抄表数据读取处理函数,该结果不影响集中器主动上传表数据
* Input        : DataFrmPtr-指向数据帧的指针
* Output       : 无
* Others       : 下行:长地址(6)
*                冻结数据上行:操作状态(1)+长地址(6)+冻结数据(N)
*                实时数据上行:操作状态(1)+长地址(6)+接收时日期(5)+数据(N)
************************************************************************************************/
void DataHandle_MeterDataLoadProc(DATA_FRAME_STRUCT *DataFrmPtr)
{
    COMMAND_TYPE cmd;
    uint8 i, dataLen, meterDataLen, *dataBufPtr;
    uint16 nodeId;
    METER_DATA_SAVE_FORMAT *meterBufPtr;
    RTC_TIME rtcTimer;
    uint8 iStart, iLoop;
    int useVal = 0;
    uint8 FreezeStatus = SUCCESS;

    cmd = DataFrmPtr->Command;
    dataLen = 0;
    nodeId = Data_FindNodeId(0, DataFrmPtr->DataBuf);
    for (i = LONG_ADDR_SIZE; i > 0; i--) {
        DataFrmPtr->DataBuf[i] = DataFrmPtr->DataBuf[i - 1];
    }
    dataBufPtr = DataFrmPtr->DataBuf + 1 + LONG_ADDR_SIZE;

    // 没有此节点或请求的数据类型与集中器的工作模式不一致或申请内存失败等情况
    if (NULL_U16_ID == nodeId) {
        DataFrmPtr->DataBuf[0] = OP_ObjectNotExist;
    } else if ((SubNodes[nodeId].DevType & 0xF0) == 0xF0) {
        DataFrmPtr->DataBuf[0] = OP_ParameterError;
    } else if ((Read_Freeze_Data == cmd && FreezeDataMode != Concentrator.Param.WorkType)) {
        DataFrmPtr->DataBuf[0] = OP_NoFunction;
    }
	//else if ((Read_RealTime_Data == cmd && RealTimeDataMode != Concentrator.Param.WorkType) ||
      //  (Read_Freeze_Data == cmd && FreezeDataMode != Concentrator.Param.WorkType)) {
      //  DataFrmPtr->DataBuf[0] = OP_NoFunction;
    //}
	else if ((void *)0 == (meterBufPtr = OSMemGetOpt(SmallMemoryPtr, 10, TIME_DELAY_MS(50)))) {
        DataFrmPtr->DataBuf[0] = OP_Failure;
    } else if ((Read_RealTime_Data == cmd && FreezeDataMode == Concentrator.Param.WorkType)) {
        dataLen = RealTimeDataMode == Concentrator.Param.WorkType ? REALTIME_DATA_AREA_SIZE : FREEZE_DATA_AREA_SIZE;
        meterDataLen = sizeof(METER_DATA_SAVE_FORMAT) - 1 + dataLen;
        Eeprom_ReadWrite((uint8 *)meterBufPtr, nodeId * NODE_INFO_SIZE, meterDataLen, READ_ATTR);
        if (0 != memcmp(SubNodes[nodeId].LongAddr, meterBufPtr->Address, LONG_ADDR_SIZE)) {
            Data_MeterDataInit(meterBufPtr, nodeId, meterDataLen);
        }
        if (meterBufPtr->Crc8MeterData == CalCrc8(meterBufPtr->MeterData, dataLen)) {
            DataFrmPtr->DataBuf[0] = OP_Succeed;
            Rtc_Get(&rtcTimer, Format_Bcd);
            dataLen = 0;
            *(dataBufPtr + dataLen++) = (uint8)(rtcTimer.Year);
            *(dataBufPtr + dataLen++) = (uint8)(rtcTimer.Year >> 8);
            *(dataBufPtr + dataLen++) = meterBufPtr->MeterData[1];//rtcTimer.Month;
            *(dataBufPtr + dataLen++) = meterBufPtr->MeterData[2];//rtcTimer.Day;
            *(dataBufPtr + dataLen++) = 0x23;//meterBufPtr->MeterData[3];//rtcTimer.Hour;
            //meterBufPtr->MeterData[4];//rtcTimer.Min;

            iStart = 5;
            for(iLoop = 0; iLoop < 4; iLoop ++){
                useVal <<= 8;
                useVal += meterBufPtr->MeterData[iStart + 4 - iLoop - 1];
            }
            FreezeStatus = SUCCESS;
            if(useVal > 0xF0FFFFFF){
                FreezeStatus = ERROR;
            }
            useVal *= 1000;
            useVal += meterBufPtr->MeterData[iStart + 4] + meterBufPtr->MeterData[iStart + 5] * 256;
            iStart += LONG_ADDR_SIZE;
            for ( iLoop = 0; iLoop < 47; iLoop++)
            {
                useVal += meterBufPtr->MeterData[iStart++];
                useVal += meterBufPtr->MeterData[iStart++] * 256;
            }
            meterBufPtr->MeterData[5] = (char)((useVal/1000) & 0xff);
            meterBufPtr->MeterData[6] = (char)(((useVal/1000) >> 8) & 0xff);
            meterBufPtr->MeterData[7] = (char)(((useVal/1000) >> 16) & 0xff);
            meterBufPtr->MeterData[8] = (char)(((useVal/1000) >> 24) & 0xff);
            meterBufPtr->MeterData[9] = (char)((useVal%1000) & 0xff);
            meterBufPtr->MeterData[10] = (char)(((useVal%1000) >> 8) & 0xff);

            memcpy(&dataBufPtr[dataLen], &meterBufPtr->MeterData[5], 6); // 正转用量[6]
            dataLen += 6;
            memset(&dataBufPtr[dataLen], 0, 6);							// 反转用量[6]
            dataLen += 6;
            memcpy(&dataBufPtr[dataLen], &meterBufPtr->MeterData[1+104], 8+2);//其他数据[10]
            dataLen += 10;

            //如果冻结数据太大
            if(ERROR == FreezeStatus){
                DataFrmPtr->DataBuf[0] = OP_ErrorMeterData;
                memset(dataBufPtr, 0, dataLen);
            }
        } else {
            DataFrmPtr->DataBuf[0] = OP_ErrorMeterData;
            memset(dataBufPtr, 0, dataLen);
        }
        OSMemPut(SmallMemoryPtr, meterBufPtr);
	} else {
        dataLen = RealTimeDataMode == Concentrator.Param.WorkType ? REALTIME_DATA_AREA_SIZE : FREEZE_DATA_AREA_SIZE;
        meterDataLen = sizeof(METER_DATA_SAVE_FORMAT) - 1 + dataLen;
        Eeprom_ReadWrite((uint8 *)meterBufPtr, nodeId * NODE_INFO_SIZE, meterDataLen, READ_ATTR);
        if (0 != memcmp(SubNodes[nodeId].LongAddr, meterBufPtr->Address, LONG_ADDR_SIZE)) {
            Data_MeterDataInit(meterBufPtr, nodeId, meterDataLen);
        }
        if (meterBufPtr->Crc8MeterData == CalCrc8(meterBufPtr->MeterData, dataLen)) {
            DataFrmPtr->DataBuf[0] = OP_Succeed;
            memcpy(dataBufPtr, meterBufPtr->MeterData, dataLen);
        } else {
            DataFrmPtr->DataBuf[0] = OP_ErrorMeterData;
            memset(dataBufPtr, 0, dataLen);
        }
        OSMemPut(SmallMemoryPtr, meterBufPtr);
    }
    DataFrmPtr->DataLen = 1 + LONG_ADDR_SIZE + dataLen;
}

/************************************************************************************************
* Function Name: DataHandle_MeterDataBatchLoadProc
* Decription   : 抄表数据批量读取处理函数,该结果不影响集中器主动上传表数据
* Input        : DataFrmPtr-指向数据帧的指针
* Output       : 无
* Others       : 下行:起始节点序号(2)+读取的数量(1)
*                冻结数据上行:节点总数量(2)+本次返回的数量N(1)+N*(操作状态(1)+长地址(6)+冻结数据(M))
*                实时数据上行:节点总数量(2)+本次返回的数量N(1)+N*(操作状态(1)+长地址(6)+时间(5)+数据(M))
************************************************************************************************/
void DataHandle_MeterDataBatchLoadProc(DATA_FRAME_STRUCT *DataFrmPtr)
{
    COMMAND_TYPE cmd;
    uint8 readCount, ackCount, dataLen, blockLen, meterDataLen, *dataBufPtr, *opStatusPtr;
    uint16 nodeId, startId, totalNodes;
    METER_DATA_SAVE_FORMAT *meterBufPtr;
	uint8 FreezeLen, RealTimeLen;
	RTC_TIME rtcTimer;
	uint8 iStart, iLoop,FreezeStatus;
	int useVal = 0;

    if ((void *)0 == (meterBufPtr = OSMemGetOpt(SmallMemoryPtr, 10, TIME_DELAY_MS(50)))) {
        DataFrmPtr->DataBuf[0] = OP_Failure;
        DataFrmPtr->DataLen = 1;
        return;
    }
    cmd = DataFrmPtr->Command;
    if ((Batch_Read_Freeze_Data == cmd && FreezeDataMode != Concentrator.Param.WorkType)) {
        DataFrmPtr->DataBuf[0] = OP_NoFunction;
        DataFrmPtr->DataLen = 1;
        OSMemPut(SmallMemoryPtr, meterBufPtr);
        return;
    }

	// 将冻结数据中的用量信息等提出给定时定量命令用
	if(Batch_Read_RealTime_Data == cmd && FreezeDataMode == Concentrator.Param.WorkType){
		FreezeLen = FREEZE_DATA_AREA_SIZE;		//冻结数据长度
		dataLen = REALTIME_DATA_AREA_SIZE;		//定时定量数据长度
		//dataLen = RealTimeDataMode == Concentrator.Param.WorkType ? REALTIME_DATA_AREA_SIZE : FREEZE_DATA_AREA_SIZE;
		//meterDataLen = sizeof(METER_DATA_SAVE_FORMAT) - 1 + dataLen;
		Rtc_Get(&rtcTimer, Format_Bcd);

		meterDataLen = sizeof(METER_DATA_SAVE_FORMAT) - 1 + FreezeLen;
		startId = ((uint16 *)DataFrmPtr->DataBuf)[0];
		readCount = DataFrmPtr->DataBuf[2];
		ackCount = 0;
		totalNodes = 0;
		dataBufPtr = DataFrmPtr->DataBuf + 3;
		blockLen = dataLen + 1 + LONG_ADDR_SIZE;
		for (nodeId = 0; nodeId < Concentrator.MaxNodeId; nodeId++) {
            if (0 == memcmp(SubNodes[nodeId].LongAddr, NullAddress, LONG_ADDR_SIZE)) {
                continue;
            } else if ((SubNodes[nodeId].DevType & 0xF0) == 0xF0) {
                continue;
            } else {
                totalNodes++;
                if (totalNodes > startId && ackCount < readCount && dataBufPtr - DataFrmPtr->DataBuf + blockLen < GPRS_DATA_MAX_DATA) {
                    ackCount++;
                    opStatusPtr = dataBufPtr++;
                    memcpy(dataBufPtr, SubNodes[nodeId].LongAddr, LONG_ADDR_SIZE);
                    dataBufPtr += LONG_ADDR_SIZE;
                    //从Eeprom 中读取冻结数据
                    Eeprom_ReadWrite((uint8 *)meterBufPtr, nodeId * NODE_INFO_SIZE, meterDataLen, READ_ATTR);
                    if (0 != memcmp(SubNodes[nodeId].LongAddr, meterBufPtr->Address, LONG_ADDR_SIZE)) {
                        Data_MeterDataInit(meterBufPtr, nodeId, meterDataLen);
                    }
                    if (meterBufPtr->Crc8MeterData == CalCrc8(meterBufPtr->MeterData, FreezeLen)) {
                        *opStatusPtr = OP_Succeed;
                        RealTimeLen = 0;
                        *(dataBufPtr + RealTimeLen++) = (uint8)(rtcTimer.Year);
                        *(dataBufPtr + RealTimeLen++) = (uint8)(rtcTimer.Year >> 8);
                        *(dataBufPtr + RealTimeLen++) = meterBufPtr->MeterData[1];//rtcTimer.Month;
                        *(dataBufPtr + RealTimeLen++) = meterBufPtr->MeterData[2];//rtcTimer.Day;
                        *(dataBufPtr + RealTimeLen++) = 0x23;//meterBufPtr->MeterData[3];//rtcTimer.Hour;
                        //meterBufPtr->MeterData[4];//rtcTimer.Min;
                        iStart = 5;
                        for(iLoop = 0; iLoop < 4; iLoop ++){
                            useVal <<= 8;
                            useVal += meterBufPtr->MeterData[iStart + 4 - iLoop - 1];
                        }
						FreezeStatus = SUCCESS;
						if(useVal > 0xF0FFFFFF){
							FreezeStatus = ERROR;
						}
                        useVal *= 1000;
                        useVal += meterBufPtr->MeterData[iStart + 4] + meterBufPtr->MeterData[iStart + 5] * 256;
                        iStart += LONG_ADDR_SIZE;
                        for ( iLoop = 0; iLoop < 47; iLoop++)
                        {
                            useVal += meterBufPtr->MeterData[iStart++];
                            useVal += meterBufPtr->MeterData[iStart++] * 256;
                        }
                        meterBufPtr->MeterData[5] = (char)((useVal/1000) & 0xff);
                        meterBufPtr->MeterData[6] = (char)(((useVal/1000) >> 8) & 0xff);
                        meterBufPtr->MeterData[7] = (char)(((useVal/1000) >> 16) & 0xff);
                        meterBufPtr->MeterData[8] = (char)(((useVal/1000) >> 24) & 0xff);
                        meterBufPtr->MeterData[9] = (char)((useVal%1000) & 0xff);
                        meterBufPtr->MeterData[10] = (char)(((useVal%1000) >> 8) & 0xff);

                        memcpy(&dataBufPtr[RealTimeLen], &meterBufPtr->MeterData[5], 6); // 正转用量[6]
                        RealTimeLen += 6;
                        memset(&dataBufPtr[RealTimeLen], 0, 6); 						// 反转用量[6]
                        RealTimeLen += 6;
                        memcpy(&dataBufPtr[RealTimeLen], &meterBufPtr->MeterData[1+104], 8+2);//其他数据[10]
                        RealTimeLen += 10;
						//如果冻结数据太大
						if(ERROR == FreezeStatus){
							*opStatusPtr = OP_ErrorMeterData;
							memset(dataBufPtr, 0, RealTimeLen);
						}
                    } else {
                        *opStatusPtr = OP_ErrorMeterData;
                        memset(dataBufPtr, 0, dataLen);
                    }
                    dataBufPtr += dataLen;
                }
            }
		}
		DataFrmPtr->DataBuf[0] = (uint8)totalNodes;
		DataFrmPtr->DataBuf[1] = (uint8)(totalNodes >> 8);
		DataFrmPtr->DataBuf[2] = ackCount;
		DataFrmPtr->DataLen = dataBufPtr - DataFrmPtr->DataBuf;
		OSMemPut(SmallMemoryPtr, meterBufPtr);
		return;
	}


    dataLen = RealTimeDataMode == Concentrator.Param.WorkType ? REALTIME_DATA_AREA_SIZE : FREEZE_DATA_AREA_SIZE;
    meterDataLen = sizeof(METER_DATA_SAVE_FORMAT) - 1 + dataLen;
    startId = ((uint16 *)DataFrmPtr->DataBuf)[0];
    readCount = DataFrmPtr->DataBuf[2];
    ackCount = 0;
    totalNodes = 0;
    dataBufPtr = DataFrmPtr->DataBuf + 3;
    blockLen = dataLen + 1 + LONG_ADDR_SIZE;
    for (nodeId = 0; nodeId < Concentrator.MaxNodeId; nodeId++) {
        if (0 == memcmp(SubNodes[nodeId].LongAddr, NullAddress, LONG_ADDR_SIZE)) {
            continue;
        } else if ((SubNodes[nodeId].DevType & 0xF0) == 0xF0) {
            continue;
        } else {
            totalNodes++;
            if (totalNodes > startId && ackCount < readCount && dataBufPtr - DataFrmPtr->DataBuf + blockLen < GPRS_DATA_MAX_DATA) {
                ackCount++;
                opStatusPtr = dataBufPtr++;
                memcpy(dataBufPtr, SubNodes[nodeId].LongAddr, LONG_ADDR_SIZE);
                dataBufPtr += LONG_ADDR_SIZE;
                Eeprom_ReadWrite((uint8 *)meterBufPtr, nodeId * NODE_INFO_SIZE, meterDataLen, READ_ATTR);
                if (0 != memcmp(SubNodes[nodeId].LongAddr, meterBufPtr->Address, LONG_ADDR_SIZE)) {
                    Data_MeterDataInit(meterBufPtr, nodeId, meterDataLen);
                }
                if (meterBufPtr->Crc8MeterData == CalCrc8(meterBufPtr->MeterData, dataLen)) {
                    *opStatusPtr = OP_Succeed;
                    memcpy(dataBufPtr, meterBufPtr->MeterData, dataLen);
                } else {
                    *opStatusPtr = OP_ErrorMeterData;
                    memset(dataBufPtr, 0, dataLen);
                }
                dataBufPtr += dataLen;
            }
        }
    }
    DataFrmPtr->DataBuf[0] = (uint8)totalNodes;
    DataFrmPtr->DataBuf[1] = (uint8)(totalNodes >> 8);
    DataFrmPtr->DataBuf[2] = ackCount;
    DataFrmPtr->DataLen = dataBufPtr - DataFrmPtr->DataBuf;
    OSMemPut(SmallMemoryPtr, meterBufPtr);
    return;
}

/************************************************************************************************
* Function Name: DataHandle_ReadMeterCmdLoadProc
* Decription   : 下发命令读取处理函数
* Input        : DataFrmPtr-指向数据帧的指针
* Output       : 无
* Others       : 下行:长地址(6)
*                上行:操作状态(1)+长地址(6)+下发命令数据(N)
************************************************************************************************/
void DataHandle_ReadMeterCmdLoadProc(DATA_FRAME_STRUCT *DataFrmPtr)
{
    uint8 i, dataLen, *dataBufPtr;
    uint16 nodeId;

    dataLen = 0;
    nodeId = Data_FindNodeId(0, DataFrmPtr->DataBuf);
    for (i = LONG_ADDR_SIZE; i > 0; i--) {
        DataFrmPtr->DataBuf[i] = DataFrmPtr->DataBuf[i - 1];
    }
    dataBufPtr = DataFrmPtr->DataBuf + 1 + LONG_ADDR_SIZE;

    // 没有此节点或请求的数据类型与集中器的工作模式不一致或申请内存失败等情况
    if (NULL_U16_ID == nodeId) {
        DataFrmPtr->DataBuf[0] = OP_ObjectNotExist;
    } else if ((SubNodes[nodeId].DevType & 0xF0) == 0xF0) {
        DataFrmPtr->DataBuf[0] = OP_ParameterError;
    } else {
    	DataFrmPtr->DataBuf[0] = OP_Succeed;
        dataBufPtr[0] = SubNodes[nodeId].CmdProp.Content;
        memcpy(&dataBufPtr[1], &SubNodes[nodeId].CmdData, sizeof(CMD_DATA));
        dataLen = 1 + sizeof(CMD_DATA); // 1 + 8
        dataBufPtr[dataLen++] = SubNodes[nodeId].AutoChannelSwitch;
    }
    DataFrmPtr->DataLen = 1 + LONG_ADDR_SIZE + dataLen;
}

/************************************************************************************************
* Function Name: DataHandle_ReadMeterCmdBatchLoadProc
* Decription   : 下发命令批量读取处理函数
* Input        : DataFrmPtr-指向数据帧的指针
* Output       : 无
* Others       : 下行:起始节点序号(2)+读取的数量(1)
*                上行:节点总数量(2)+本次返回的数量N(1)+N*(操作状态(1)+长地址(6)+下发命令数据(M))
************************************************************************************************/
void DataHandle_ReadMeterCmdBatchLoadProc(DATA_FRAME_STRUCT *DataFrmPtr)
{
    uint8 readCount, ackCount, dataLen, blockLen, *dataBufPtr, *opStatusPtr;
    uint16 nodeId, startId, totalNodes;

	dataLen = 1 + sizeof(CMD_DATA); // 1 + 8

    startId = ((uint16 *)DataFrmPtr->DataBuf)[0];
    readCount = DataFrmPtr->DataBuf[2];
    ackCount = 0;
    totalNodes = 0;
    dataBufPtr = DataFrmPtr->DataBuf + 3;
    blockLen = dataLen + 1 + LONG_ADDR_SIZE;
    for (nodeId = 0; nodeId < Concentrator.MaxNodeId; nodeId++) {
        if (0 == memcmp(SubNodes[nodeId].LongAddr, NullAddress, LONG_ADDR_SIZE)) {
            continue;
        } else if ((SubNodes[nodeId].DevType & 0xF0) == 0xF0) {
            continue;
        } else {
            totalNodes++;
            if (totalNodes > startId && ackCount < readCount && dataBufPtr - DataFrmPtr->DataBuf + blockLen < GPRS_DATA_MAX_DATA) {
                ackCount++;
                opStatusPtr = dataBufPtr++;
                memcpy(dataBufPtr, SubNodes[nodeId].LongAddr, LONG_ADDR_SIZE);
                dataBufPtr += LONG_ADDR_SIZE;
                *opStatusPtr = OP_Succeed;
                memcpy(dataBufPtr, &SubNodes[nodeId].CmdProp.Content, dataLen);
                dataBufPtr += dataLen;
				*dataBufPtr++ = SubNodes[nodeId].AutoChannelSwitch;
				//dataBufPtr[dataLen++] = SubNodes[nodeId].AutoChannelSwitch;
            }
        }
    }
    DataFrmPtr->DataBuf[0] = (uint8)totalNodes;
    DataFrmPtr->DataBuf[1] = (uint8)(totalNodes >> 8);
    DataFrmPtr->DataBuf[2] = ackCount;
    DataFrmPtr->DataLen = dataBufPtr - DataFrmPtr->DataBuf;
    return;
}


/************************************************************************************************
* Function Name: DataHandle_DataCommandResultTask
* Decription   : 命令下发结果处理任务
* Input        : p_arg-保存原来数据的指针
* Output       : 无
************************************************************************************************/
void DataHandle_DataCommandResultTask(void *p_arg)
{
    uint8 err;
    uint32 waitAckTime, startTime;
    DATA_HANDLE_TASK *taskPtr;
    DATA_FRAME_STRUCT *rxDataFrmPtr;
    uint8 dataLen = 0, meterDataLen, bufLen = 0;
    uint16 nodeId;
    METER_DATA_SAVE_FORMAT *meterBufPtr;

    taskPtr = (DATA_HANDLE_TASK *)p_arg;

    TaskRunStatus.DataForward = TRUE;
    startTime = Timer1ms;
    waitAckTime = (taskPtr->RouteLevel - 1) * DELAYTIME_ONE_LAYER * 2;
    while (waitAckTime > 0) {
        rxDataFrmPtr = OSMboxPend(taskPtr->Mbox, TIME_DELAY_MS(waitAckTime), &err);
        if ((void *)0 != rxDataFrmPtr) {
		// 数据域第二字节为操作结果
            if( 0xAA == rxDataFrmPtr->DataBuf[bufLen+1] ){
                nodeId = Data_FindNodeId(0, rxDataFrmPtr->Route[0]);
                if (NULL_U16_ID == nodeId ) {
                    OSMemPut(LargeMemoryPtr, rxDataFrmPtr);
                    waitAckTime = (Timer1ms > startTime) ? (Timer1ms - startTime) : 0;
                    continue;
                }
                // 再申请一个内存,用于读取Eeprom中的数据
                if ((void *)0 == (meterBufPtr = OSMemGetOpt(SmallMemoryPtr, 10, TIME_DELAY_MS(50)))) {
                    OSMemPut(LargeMemoryPtr, rxDataFrmPtr);
                    waitAckTime = (Timer1ms > startTime) ? (Timer1ms - startTime) : 0;
                    continue;
                }
                meterDataLen = sizeof(METER_DATA_SAVE_FORMAT) - 1;
                Eeprom_ReadWrite((uint8 *)meterBufPtr, nodeId * NODE_INFO_SIZE, meterDataLen, READ_ATTR);
                if (0 != memcmp(SubNodes[nodeId].LongAddr, meterBufPtr->Address, LONG_ADDR_SIZE)) {
                    Data_MeterDataInit(meterBufPtr, nodeId, meterDataLen);
                }
				// 设置信道 : 操作命令(0x1) + 操作结果(0xAA或0xAB) + 信道号(低4位为表端主动上报信道，高4位为当前通信信道)
                if( Meter_ACK_Set_Channel == rxDataFrmPtr->DataBuf[bufLen] )
                {// 设置信道
                	if( (SubNodes[nodeId].CmdProp.SetChannel != 0) &&
						((SubNodes[nodeId].CmdData.SetChannel&0x7F) == (rxDataFrmPtr->DataBuf[bufLen+2]&0xF)))
                	{
                		Concentrator.SecondChannel = SubNodes[nodeId].CmdData.SetChannel&0x7F;
						// 当检测到表端自动信道已经打开，并且此时正在设置信道。则手动修改 Eprom 中保存的 RX/TX 内容
						if(SubNodes[nodeId].AutoChannelSwitch == TRUE )
						{
							if (RealTimeDataMode == Concentrator.Param.WorkType ) {
								dataLen = REALTIME_DATA_AREA_SIZE - UPDOWN_RSSI_SIZE;
							} else if (FreezeDataMode == Concentrator.Param.WorkType ) {
								dataLen = FREEZE_DATA_AREA_SIZE - UPDOWN_RSSI_SIZE;
							}
							meterDataLen = sizeof(METER_DATA_SAVE_FORMAT) - 1 + dataLen + UPDOWN_RSSI_SIZE;
							Eeprom_ReadWrite((uint8 *)meterBufPtr, nodeId * NODE_INFO_SIZE, meterDataLen, READ_ATTR);
							SubNodes[nodeId].AutoChannelSwitch = TRUE;
							SubNodes[nodeId].RxChannel = SubNodes[nodeId].LongAddr[LONG_ADDR_SIZE-1] & 0xF;
							SubNodes[nodeId].TxChannel = Concentrator.SecondChannel;
							if(Concentrator.SaveSecondChannel != Concentrator.SecondChannel){
								ModifyScanChannel = 5;
						    }
							meterBufPtr->RxChannel = SubNodes[nodeId].RxChannel;
							meterBufPtr->TxChannel = SubNodes[nodeId].TxChannel;
							meterBufPtr->MeterData[meterDataLen - sizeof(METER_DATA_SAVE_FORMAT) - UPDOWN_RSSI_SIZE - 1] =
													(SubNodes[nodeId].TxChannel & 0xF) | ((SubNodes[nodeId].RxChannel << 4)&0xF0);
							meterBufPtr->Crc8MeterData = CalCrc8(&meterBufPtr->MeterData[0], dataLen + UPDOWN_RSSI_SIZE);
						}
						SubNodes[nodeId].CmdProp.SetChannel = 0;
						SubNodes[nodeId].CmdData.SetChannel = 0;
						meterBufPtr->CmdProp.SetChannel = 0;
						meterBufPtr->CmdData.SetChannel = 0;
						Eeprom_ReadWrite((uint8 *)meterBufPtr, nodeId * NODE_INFO_SIZE, meterDataLen, WRITE_ATTR);
					}
                }
				// 打开自动信道 : 操作命令(0x2) + 操作结果(0xAA或0xAB) + 2字节表端状态(第二字节 bit0 为开关自动信道)
                else if( Meter_ACK_Open_Auto_Channel == rxDataFrmPtr->DataBuf[bufLen] )
                {// 打开自动信道设置
                	if( (SubNodes[nodeId].CmdProp.AutoChannel != 0) &&
						(((SubNodes[nodeId].CmdData.AutoChannel>>7)&0x1) == ((rxDataFrmPtr->DataBuf[bufLen+3]>>0)&0x1)))
					{
	                    if(SubNodes[nodeId].CmdData.AutoChannel == 0x80){
	                       // 当自动信道打开时，表端接收信道是表号尾号的最后一位。发射信道是已经设置好的信道，集中器暂不可知（或者集中器统一设置）。
	                       SubNodes[nodeId].AutoChannelSwitch = TRUE;
	                       SubNodes[nodeId].RxChannel = SubNodes[nodeId].LongAddr[LONG_ADDR_SIZE-1] & 0xF;
	                       SubNodes[nodeId].TxChannel = Concentrator.SecondChannel;
						   if(Concentrator.SaveSecondChannel != Concentrator.SecondChannel){
							   ModifyScanChannel = 5;
						   }
	                    }else{
	                       SubNodes[nodeId].AutoChannelSwitch = FALSE;
	                       SubNodes[nodeId].RxChannel = DEFAULT_TX_CHANNEL;
	                       SubNodes[nodeId].TxChannel = DEFAULT_TX_CHANNEL;
	                    }

	                    if (RealTimeDataMode == Concentrator.Param.WorkType ) {
	                       dataLen = REALTIME_DATA_AREA_SIZE - UPDOWN_RSSI_SIZE;
	                    } else if (FreezeDataMode == Concentrator.Param.WorkType ) {
	                       dataLen = FREEZE_DATA_AREA_SIZE - UPDOWN_RSSI_SIZE;
	                    }
	                    meterDataLen = sizeof(METER_DATA_SAVE_FORMAT) - 1 + dataLen + UPDOWN_RSSI_SIZE;
	                    Eeprom_ReadWrite((uint8 *)meterBufPtr, nodeId * NODE_INFO_SIZE, meterDataLen, READ_ATTR);

	                    meterBufPtr->MeterData[meterDataLen - sizeof(METER_DATA_SAVE_FORMAT) - UPDOWN_RSSI_SIZE - 1] =
	                               (SubNodes[nodeId].TxChannel & 0xF) | ((SubNodes[nodeId].RxChannel << 4)&0xF0);
	                    meterBufPtr->Crc8MeterData = CalCrc8(&meterBufPtr->MeterData[0], dataLen + UPDOWN_RSSI_SIZE);

						SubNodes[nodeId].CmdProp.AutoChannel = 0;
						SubNodes[nodeId].CmdData.AutoChannel = 0;
						meterBufPtr->CmdProp.AutoChannel = 0;
						meterBufPtr->CmdData.AutoChannel = 0;
						meterBufPtr->RxChannel = SubNodes[nodeId].RxChannel;
						meterBufPtr->TxChannel = SubNodes[nodeId].TxChannel;
						meterBufPtr->AutoChannelSwitch = SubNodes[nodeId].AutoChannelSwitch;
						Eeprom_ReadWrite((uint8 *)meterBufPtr, nodeId * NODE_INFO_SIZE, meterDataLen, WRITE_ATTR);
					}
                }
				// 阀控 : 操作命令(0x3) + 操作结果(0xAA或0xAB) + 阀状态(1开阀，2关阀)
                else if( Meter_ACK_Control_Valve == rxDataFrmPtr->DataBuf[bufLen] )
                {// 阀控   低四位：1开阀，2关阀
					if( (SubNodes[nodeId].CmdProp.ValveCtrl != 0) &&
						(SubNodes[nodeId].CmdData.ValveCtrl == rxDataFrmPtr->DataBuf[bufLen+2]))
					{
						SubNodes[nodeId].CmdProp.ValveCtrl = 0;
						SubNodes[nodeId].CmdData.ValveCtrl = 0;
						meterBufPtr->CmdProp.ValveCtrl = 0;
						meterBufPtr->CmdData.ValveCtrl = 0;
						Eeprom_ReadWrite((uint8 *)meterBufPtr, nodeId * NODE_INFO_SIZE, meterDataLen, WRITE_ATTR);
					}
                }
				// 结算价格 : 操作命令(0x4) + 操作结果(0xAA或0xAB) + 2字节结算价格(低位在前，高位在后)
                else if( Meter_ACK_Settlement_Price == rxDataFrmPtr->DataBuf[bufLen] )
                {// 结算价格
					if( (SubNodes[nodeId].CmdProp.SettlementPrice != 0) &&
						(SubNodes[nodeId].CmdData.SettlementPriceLow == rxDataFrmPtr->DataBuf[bufLen+2]) &&
						(SubNodes[nodeId].CmdData.SettlementPriceHigh == rxDataFrmPtr->DataBuf[bufLen+3]))
					{
						SubNodes[nodeId].CmdProp.SettlementPrice = 0;
						SubNodes[nodeId].CmdData.SettlementPriceLow = 0;
						SubNodes[nodeId].CmdData.SettlementPriceHigh = 0;
						meterBufPtr->CmdProp.SettlementPrice = 0;
						meterBufPtr->CmdData.SettlementPriceLow = 0;
						meterBufPtr->CmdData.SettlementPriceHigh = 0;
						Eeprom_ReadWrite((uint8 *)meterBufPtr, nodeId * NODE_INFO_SIZE, meterDataLen, WRITE_ATTR);
					}
                }
				// 开关实时抄表 : 操作命令(0x5) + 操作结果(0xAA或0xAB) + 2字节表端状态(第二字节 bit3 为开关自动信道)
				else if( Meter_ACK_Control_RwMeter == rxDataFrmPtr->DataBuf[bufLen] )
                {// 开关实时抄表
					if( (SubNodes[nodeId].CmdProp.RealTimeMeterReading != 0) &&
						(((SubNodes[nodeId].CmdData.RealTimeMeterReading>>7)&0x1) == ((rxDataFrmPtr->DataBuf[bufLen+3]>>3)&0x1)))
					{
						SubNodes[nodeId].CmdProp.RealTimeMeterReading = 0;
						SubNodes[nodeId].CmdData.RealTimeMeterReading = 0;
						meterBufPtr->CmdProp.RealTimeMeterReading = 0;
						meterBufPtr->CmdData.RealTimeMeterReading = 0;
						Eeprom_ReadWrite((uint8 *)meterBufPtr, nodeId * NODE_INFO_SIZE, meterDataLen, WRITE_ATTR);
					}
                }
                OSMemPut(SmallMemoryPtr, meterBufPtr);
            }
			else if( 0xAB == rxDataFrmPtr->DataBuf[bufLen+1] )
			{
				Gprs_OutputDebugMsg(0, "----->>  0xAB  <<------");
			}
            OSMemPut(LargeMemoryPtr, rxDataFrmPtr);
        }
        break;
    }

    // 销毁本任务,此处必须先禁止任务调度,否则无法释放本任务占用的内存空间
    OSMboxDel(taskPtr->Mbox, OS_DEL_ALWAYS, &err);
    OSSchedLock();
    OSTaskDel(OS_PRIO_SELF);
    OSMemPut(LargeMemoryPtr, taskPtr->StkPtr);
    taskPtr->StkPtr = (void *)0;
    TaskRunStatus.DataForward = FALSE;
    OSSchedUnlock();
}

/************************************************************************************************
* Function Name: DataHandle_ActiveReportProc
* Decription   : 表具主动上报数据处理函数
* Input        : DataFrmPtr-接收到的数据指针
* Output       : 后续是否需要应答处理
* Others       : 用于处理主动上报定时定量数据,冻结数据.
************************************************************************************************/
bool DataHandle_ActiveReportProc(DATA_FRAME_STRUCT *DataFrmPtr)
{
    // 当节点不属于本集中器或者其他保存失败的情况下直接上传到服务器且不负责传输的结果
    if (ERROR == DataHandle_MeterDataSaveProc(DataFrmPtr)) {
#ifdef GIVEUP_ABNORMAL_DATA
        OSMemPut(LargeMemoryPtr, DataFrmPtr);
#else
        DataFrmPtr->PortNo = Uart_Gprs;
        DataFrmPtr->PkgProp = DataHandle_SetPkgProperty(DataFrmPtr->PkgProp.PkgXor, NONE_ACK, CMD_PKG, UP_DIR);
		if( XOR_CRC16 == DataFrmPtr->PkgProp.PkgXor){
			DataFrmPtr->PkgLength = DATA_FIXED_AREA_LENGTH_CRC16;
		}else{
			DataFrmPtr->PkgLength = DATA_FIXED_AREA_LENGTH_CRC8;
		}
        DataFrmPtr->PkgSn = PkgNo++;
        DataFrmPtr->DeviceType = Dev_Concentrator;
        DataFrmPtr->Life_Ack.Content = 0x0F;
        DataFrmPtr->RouteInfo.CurPos = 0;
        DataFrmPtr->RouteInfo.Level = 2;
        memcpy(DataFrmPtr->Route[0], Concentrator.LongAddr, LONG_ADDR_SIZE);
        memcpy(DataFrmPtr->Route[1], BroadcastAddrOut, LONG_ADDR_SIZE);
        DataHandle_SetPkgPath(DataFrmPtr, UNREVERSED);
        DataHandle_CreateTxData(DataFrmPtr, NULL);
#endif
    } else {
        uint8 length, version;
        uint16 nodeId;
        DATA_HANDLE_TASK *taskPtr;
        uint8 err;

    	length = DataFrmPtr->DataLen - 1;       // 减去数据格式编号字节
        version = DataFrmPtr->DataBuf[length];  // 表版本号
        nodeId = Data_FindNodeId(0, DataFrmPtr->Route[0]);
        if (NULL_U16_ID == nodeId) {
            OSMemPut(LargeMemoryPtr, DataFrmPtr);
            return NONE_ACK;
        }

        // 创建下行数据
        DataFrmPtr->DeviceType = Dev_Concentrator;
        DataFrmPtr->DataLen = 0;
        DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = 0xA0;	   // 特征字
        Rtc_Get((RTC_TIME *)(DataFrmPtr->DataBuf + DataFrmPtr->DataLen), Format_Bcd);
        DataFrmPtr->DataLen += 7;
        memset((uint8 *)(&DataFrmPtr->DataBuf[DataFrmPtr->DataLen]), 0, 12);
        DataFrmPtr->DataLen += 12;
        Data_GetTimeSlot(DataFrmPtr->Route[0], (uint8 *)(&DataFrmPtr->DataBuf[DataFrmPtr->DataLen]));
        DataFrmPtr->DataLen += 4;

        // 如果有下发命令，并且表版本号大于 3
        if( SubNodes[nodeId].CmdProp.Content != 0x0 && version > 0x3 ){
            DataFrmPtr->DataBuf[0] = 0xA1;	   // 特征字
            if( SubNodes[nodeId].CmdProp.SetChannel == 0x1 ){
                // 设置信道
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = Meter_ACK_Set_Channel; // 设置信道
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = 0x03;					// 数据长度
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = 0xDA;					// 两个字节的特征字 0xDA 0x26
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = 0x26;
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = SubNodes[nodeId].CmdData.SetChannel;	// 信道号，Bit7: 为1时设置信道，低7bit为信道号
            } else if ( SubNodes[nodeId].CmdProp.AutoChannel == 0x1 ){
                // 打开自动信道设置
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = Meter_ACK_Open_Auto_Channel; // 开关自动信道
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = 0x03;					// 数据长度
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = 0xDA;					// 两个字节的特征字 0xDA 0x26
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = 0x26;
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = SubNodes[nodeId].CmdData.AutoChannel;	// bit7：1开, 0关
            } else if ( SubNodes[nodeId].CmdProp.ValveCtrl == 0x1 ){
                // 阀控
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = Meter_ACK_Control_Valve; // 阀控
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = 0x03;					// 数据长度
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = 0xDA;					// 两个字节的特征字 0xDA 0x26
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = 0x26;
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = SubNodes[nodeId].CmdData.ValveCtrl;// 低四位：1开阀，2关阀
            } else if ( SubNodes[nodeId].CmdProp.RealTimeMeterReading == 0x1 ){
                // 开关实时抄表
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = Meter_ACK_Control_RwMeter; // 打开关闭实时抄表功能
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = 0x03;					// 数据长度
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = 0xDA;					// 两个字节的特征字 0xDA 0x26
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = 0x26;
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = SubNodes[nodeId].CmdData.RealTimeMeterReading;// bit7：1开, 0关
            } else if ( SubNodes[nodeId].CmdProp.SettlementPrice == 0x1 ){
                // 结算价格
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = Meter_ACK_Settlement_Price; // 结算价格
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = 0x04;					// 数据长度
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = 0xDA;					// 两个字节的特征字 0xDA 0x26
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = 0x26;
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = SubNodes[nodeId].CmdData.SettlementPriceLow;// 低位在前，高位在后
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = SubNodes[nodeId].CmdData.SettlementPriceHigh;// 低位在前，高位在后
            }

            // 如果需要处理应答消息,则创建应答处理任务
            if (NEED_ACK == DataFrmPtr->PkgProp.NeedAck) {
                if ((void *)0 == (taskPtr = DataHandle_GetEmptyTaskPtr())) {
                    goto DATA_FORWARD_FAILED;
                }
                if ((void *)0 == (taskPtr->StkPtr = (OS_STK *)OSMemGetOpt(LargeMemoryPtr, 10, TIME_DELAY_MS(50)))) {
                    goto DATA_FORWARD_FAILED;
                }
                taskPtr->Command = DataFrmPtr->Command;
                taskPtr->NodeId = nodeId;
                taskPtr->PkgSn = DataFrmPtr->PkgSn;
                taskPtr->RouteLevel = DataFrmPtr->RouteInfo.Level;
                taskPtr->PortNo = DataFrmPtr->PortNo;
                taskPtr->Mbox = OSMboxCreate((void *)0);
                //taskPtr->Msg = (uint8 *)rxDataFrmPtr;
                if (OS_ERR_NONE != OSTaskCreate(DataHandle_DataCommandResultTask, taskPtr, taskPtr->StkPtr + MEM_LARGE_BLOCK_LEN / sizeof(OS_STK) - 1, taskPtr->Prio)) {
                    OSMemPut(LargeMemoryPtr, taskPtr->StkPtr);
                    taskPtr->StkPtr = (void *)0;
                    OSMboxDel(taskPtr->Mbox, OS_DEL_ALWAYS, &err);
                    goto DATA_FORWARD_FAILED;
                }
            }

            DataFrmPtr->PkgProp = DataHandle_SetPkgProperty(DataFrmPtr->PkgProp.PkgXor, NEED_ACK, CMD_PKG, DOWN_DIR);
			DataHandle_SetPkgPath(DataFrmPtr, REVERSED);
			Test_DataHandle_CreateTxData(DataFrmPtr, version);
        }else{
            DataFrmPtr->PkgProp = DataHandle_SetPkgProperty(DataFrmPtr->PkgProp.PkgXor, NONE_ACK, ACK_PKG, DOWN_DIR);
			DataHandle_SetPkgPath(DataFrmPtr, REVERSED);
			DataHandle_CreateTxData(DataFrmPtr, version);
        }

        return NONE_ACK;

DATA_FORWARD_FAILED:
        if (NEED_ACK == DataFrmPtr->PkgProp.NeedAck) {
            DataFrmPtr->DataBuf[0] = OP_Failure;
            DataFrmPtr->DataLen = 1;
            return NEED_ACK;
        }
    }

    return NONE_ACK;
}

/************************************************************************************************
* Function Name: DataHandle_RTCTimingTask
* Decription   : 实时时钟校时处理任务
* Input        : *p_arg-参数指针
* Output       : 无
* Others       : 无
************************************************************************************************/
void DataHandle_RTCTimingTask(void *p_arg)
{
    uint8 err;
    DATA_HANDLE_TASK *taskPtr;
    DATA_FRAME_STRUCT *txDataFrmPtr, *rxDataFrmPtr;
	uint8 version = 0;

    // 创建上行校时数据包
    TaskRunStatus.RTCTiming = TRUE;
    if ((void *)0 != (txDataFrmPtr = OSMemGetOpt(LargeMemoryPtr, 10, TIME_DELAY_MS(50)))) {
        txDataFrmPtr->PortNo = Uart_Gprs;
		txDataFrmPtr->PkgProp = DataHandle_SetPkgProperty(XOR_CRC8, NEED_ACK, CMD_PKG, UP_DIR);
		if( XOR_CRC16 == txDataFrmPtr->PkgProp.PkgXor){
			txDataFrmPtr->PkgLength = DATA_FIXED_AREA_LENGTH_CRC16;
		}else{
			txDataFrmPtr->PkgLength = DATA_FIXED_AREA_LENGTH_CRC8;
		}
        txDataFrmPtr->PkgSn = PkgNo++;
        txDataFrmPtr->Command = CONC_RTC_Timing;
        txDataFrmPtr->DeviceType = Dev_Concentrator;
        txDataFrmPtr->Life_Ack.Content = 0x0F;
        txDataFrmPtr->RouteInfo.CurPos = 0;
        txDataFrmPtr->RouteInfo.Level = 2;
        memcpy(txDataFrmPtr->Route[0], Concentrator.LongAddr, LONG_ADDR_SIZE);
        memcpy(txDataFrmPtr->Route[1], BroadcastAddrOut, LONG_ADDR_SIZE);
        txDataFrmPtr->DataLen = 0;

        taskPtr = (DATA_HANDLE_TASK *)p_arg;
        taskPtr->Command = txDataFrmPtr->Command;
        taskPtr->NodeId = NULL_U16_ID;
        taskPtr->PkgSn = txDataFrmPtr->PkgSn;

        // 创建发送数据包
        DataHandle_SetPkgPath(txDataFrmPtr, UNREVERSED);
        DataHandle_CreateTxData(txDataFrmPtr, version);

        // 等待服务器的应答
        rxDataFrmPtr = OSMboxPend(taskPtr->Mbox, GPRS_WAIT_ACK_OVERTIME, &err);
        if ((void *)0 == rxDataFrmPtr) {
            RTCTimingTimer = 300;               // 如果超时则5分钟后重试
        } else {
            if (SUCCESS == Rtc_Set(*(RTC_TIME *)(rxDataFrmPtr->DataBuf), Format_Bcd)) {
                RTCTimingTimer = RTCTIMING_INTERVAL_TIME;
            } else {
                RTCTimingTimer = 5;             // 如果校时失败则5秒后重试
            }
            OSMemPut(LargeMemoryPtr, rxDataFrmPtr);
        }
    }

    // 销毁本任务,此处必须先禁止任务调度,否则无法释放本任务占用的内存空间
    OSMboxDel(taskPtr->Mbox, OS_DEL_ALWAYS, &err);
    OSSchedLock();
    OSTaskDel(OS_PRIO_SELF);
    OSMemPut(LargeMemoryPtr, taskPtr->StkPtr);
    taskPtr->StkPtr = (void *)0;
    TaskRunStatus.RTCTiming = FALSE;
    OSSchedUnlock();
}

/************************************************************************************************
* Function Name: DataHandle_RTCTimingProc
* Decription   : 集中器实时时钟主动校时处理函数
* Input        : 无
* Output       : 无
* Others       : 每隔一段时间就启动一次校时任务
************************************************************************************************/
void DataHandle_RTCTimingProc(void)
{
    uint8 err;
    DATA_HANDLE_TASK *taskPtr;

    // 检查Gprs是否在线或者任务是否正在运行中
    if (FALSE == Gprs.Online || TRUE == TaskRunStatus.RTCTiming) {
        RTCTimingTimer = 60;
        return;
    }

    if ((void *)0 == (taskPtr = DataHandle_GetEmptyTaskPtr())) {
        return;
    }
    if ((void *)0 == (taskPtr->StkPtr = (OS_STK *)OSMemGetOpt(LargeMemoryPtr, 10, TIME_DELAY_MS(50)))) {
        return;
    }
    taskPtr->Mbox = OSMboxCreate((void *)0);
    taskPtr->Msg = (void *)0;
    if (OS_ERR_NONE != OSTaskCreate(DataHandle_RTCTimingTask, taskPtr,
        taskPtr->StkPtr + MEM_LARGE_BLOCK_LEN / sizeof(OS_STK) - 1, taskPtr->Prio)) {
        OSMemPut(LargeMemoryPtr, taskPtr->StkPtr);
        taskPtr->StkPtr = (void *)0;
        OSMboxDel(taskPtr->Mbox, OS_DEL_ALWAYS, &err);
    }
}



/************************************************************************************************
* Function Name: DataHandle_ModifyScanChannelTask
* Decription   : 修改信道处理任务
* Input        : *p_arg-参数指针
* Output       : 无
* Others       : 无
************************************************************************************************/
void DataHandle_ModifyScanChannelTask(void *p_arg)
{
    uint8 err;
    DATA_HANDLE_TASK *taskPtr;
    DATA_FRAME_STRUCT *txDataFrmPtr, *rxDataFrmPtr;
    uint8 version = 0;

    // 创建上行修改信道数据包
    TaskRunStatus.ModifyScanChannel = TRUE;
    if ((void *)0 != (txDataFrmPtr = OSMemGetOpt(LargeMemoryPtr, 10, TIME_DELAY_MS(50)))) {
        txDataFrmPtr->PortNo = Usart_Rf;
		txDataFrmPtr->PkgProp = DataHandle_SetPkgProperty(XOR_CRC8, NEED_ACK, CMD_PKG, UP_DIR);
		if( XOR_CRC16 == txDataFrmPtr->PkgProp.PkgXor){
			txDataFrmPtr->PkgLength = DATA_FIXED_AREA_LENGTH_CRC16;
		}else{
			txDataFrmPtr->PkgLength = DATA_FIXED_AREA_LENGTH_CRC8;
		}
        txDataFrmPtr->PkgSn = PkgNo++;
        txDataFrmPtr->Command = Modify_Scan_Channel;
        txDataFrmPtr->DeviceType = Dev_Concentrator;
        txDataFrmPtr->Life_Ack.Content = 0x0F;
        txDataFrmPtr->RouteInfo.CurPos = 0;
        txDataFrmPtr->RouteInfo.Level = 2;
        memcpy(txDataFrmPtr->Route[0], Concentrator.LongAddr, LONG_ADDR_SIZE);
        memcpy(txDataFrmPtr->Route[1], BroadcastAddrOut, LONG_ADDR_SIZE);
        txDataFrmPtr->DataLen = 0;
        // 数据域包含
        txDataFrmPtr->DataBuf[0] = 0x82;// 0x2 读信道， 0x82 设置信道
        memcpy( &txDataFrmPtr->DataBuf[1], ModifyScanChannel_KEY, sizeof(ModifyScanChannel_KEY));
        txDataFrmPtr->DataLen += sizeof(ModifyScanChannel_KEY) + 1;
        // 高4位固定为3信道，低4位为需要设置的信道号
        txDataFrmPtr->DataBuf[txDataFrmPtr->DataLen++] = ((0x3 << 4)&0xF0) | (Concentrator.SecondChannel & 0xF);

        taskPtr = (DATA_HANDLE_TASK *)p_arg;
        taskPtr->Command = txDataFrmPtr->Command;
        taskPtr->NodeId = NULL_U16_ID;
        taskPtr->PkgSn = txDataFrmPtr->PkgSn;

        // 创建发送数据包
        DataHandle_SetPkgPath(txDataFrmPtr, UNREVERSED);
        DataHandle_CreateTxData(txDataFrmPtr, version);

        // 等待服务器的应答
        rxDataFrmPtr = OSMboxPend(taskPtr->Mbox, GPRS_WAIT_ACK_OVERTIME, &err);
        if ((void *)0 == rxDataFrmPtr) {
            ModifyScanChannel = 60;               // 如果超时则1分钟后重试
        } else {
            if ( ( 0x02 == rxDataFrmPtr->DataBuf[0] ) &&
                ( OP_Succeed == rxDataFrmPtr->DataBuf[1] ) &&
                ( Concentrator.SecondChannel == (rxDataFrmPtr->DataBuf[2]&0x0F) ) &&
                ( 0x30 == (rxDataFrmPtr->DataBuf[2]&0xF0) ) ) {
                ModifyScanChannel = 0;
                Concentrator.SaveSecondChannel = rxDataFrmPtr->DataBuf[2]&0x0F;
                Flash_SaveConcentratorInfo();
            } else {
                ModifyScanChannel = 5;             // 如果校时失败则5秒后重试
            }
            OSMemPut(LargeMemoryPtr, rxDataFrmPtr);
        }
    }

    // 销毁本任务,此处必须先禁止任务调度,否则无法释放本任务占用的内存空间
    OSMboxDel(taskPtr->Mbox, OS_DEL_ALWAYS, &err);
    OSSchedLock();
    OSTaskDel(OS_PRIO_SELF);
    OSMemPut(LargeMemoryPtr, taskPtr->StkPtr);
    taskPtr->StkPtr = (void *)0;
    TaskRunStatus.ModifyScanChannel = FALSE;
    OSSchedUnlock();
}

/************************************************************************************************
* Function Name: DataHandle_ModifyScanChannelProc
* Decription   : 集中器发送修改2e28扫描信道处理函数
* Input        :
* Others       : 无
************************************************************************************************/
void DataHandle_ModifyScanChannelProc(void)
{
	uint8 err;
	DATA_HANDLE_TASK *taskPtr;

	// 检查是否有其他任务正在运行中
	if (TRUE == TaskRunStatus.DataReplenish || TRUE == TaskRunStatus.DataForward || TRUE == TaskRunStatus.ModifyScanChannel) {
		ModifyScanChannel = 60;
		return;
	}

	if ((void *)0 == (taskPtr = DataHandle_GetEmptyTaskPtr())) {
		return;
	}
	if ((void *)0 == (taskPtr->StkPtr = (OS_STK *)OSMemGetOpt(LargeMemoryPtr, 10, TIME_DELAY_MS(50)))) {
		return;
	}
	taskPtr->Mbox = OSMboxCreate((void *)0);
	taskPtr->Msg = (void *)0;
	if (OS_ERR_NONE != OSTaskCreate(DataHandle_ModifyScanChannelTask, taskPtr,
		taskPtr->StkPtr + MEM_LARGE_BLOCK_LEN / sizeof(OS_STK) - 1, taskPtr->Prio)) {
		OSMemPut(LargeMemoryPtr, taskPtr->StkPtr);
		taskPtr->StkPtr = (void *)0;
		OSMboxDel(taskPtr->Mbox, OS_DEL_ALWAYS, &err);
	}
}




/************************************************************************************************
* Function Name: DataHandle_DataReplenishTask
* Decription   : 数据补抄处理任务
* Input        : *p_arg-参数指针
* Output       : 无
* Others       : 当到达补抄时间时,检查是否有未收到的数据,进行补抄,在超过设定的补抄轮次或者过零点
*                后,补抄自动停止
************************************************************************************************/
void DataHandle_DataReplenishTask(void *p_arg)
{
    uint8 err, retry, lastDay, version = 0;
    uint16 nodeId;
    uint32 dataReplenishDay, dayMask, RtcTemp, MeterDataTemp;
    RTC_TIME rtcTimer;
    DATA_HANDLE_TASK *taskPtr;
    DATA_FRAME_STRUCT *txDataFrmPtr, *rxDataFrmPtr;

    taskPtr = (DATA_HANDLE_TASK *)p_arg;
    TaskRunStatus.DataReplenish = TRUE;
    Rtc_Get(&rtcTimer, Format_Bcd);
    for (retry = 0; retry < 3; retry++) {
        for (nodeId = 0; nodeId < Concentrator.MaxNodeId; nodeId++) {
            DataReplenishTimer = DATAREPLENISH_INTERVAL_TIME;
            // 判断是否为表具,不是则略过
            if (0 == memcmp(SubNodes[nodeId].LongAddr, NullAddress, LONG_ADDR_SIZE)) {
                continue;
            }
            // 判断是否是计量表,不是则排除
            if ((SubNodes[nodeId].DevType & 0xF0) == 0xF0) {
                continue;
            }

			// 获取当前时间和节点记录时间，两者差值需要小于23小时
            MeterDataTemp = Byte4ToUint32(&SubNodes[nodeId].RxMeterDataTemp[0]);
			RtcTemp = RTC_GetCounter();

            // 数据是否已经抄到
            if ((SubNodes[nodeId].RxMeterDataDay == rtcTimer.Day) ||
                (RtcTemp - MeterDataTemp <= 23*60*60)) {
#if PRINT_INFO
                Gprs_OutputDebugMsg(0, "\n 已抄到: ");
                OSTimeDlyHMSM(0, 0, 0, 500);
                DebugOutputLength(SubNodes[nodeId].LongAddr, LONG_ADDR_SIZE);
                OSTimeDlyHMSM(0, 0, 0, 500);
#endif
                continue;
            }
            // 创建抄表数据包
            if ((void *)0 == (txDataFrmPtr = OSMemGetOpt(LargeMemoryPtr, 10, TIME_DELAY_MS(50)))) {
                break;
            }
            txDataFrmPtr->PortNo = Usart_Rf;
            txDataFrmPtr->PkgProp = DataHandle_SetPkgProperty(XOR_CRC8, NEED_ACK, CMD_PKG, DOWN_DIR);
			if( XOR_CRC16 == txDataFrmPtr->PkgProp.PkgXor){
				txDataFrmPtr->PkgLength = DATA_FIXED_AREA_LENGTH_CRC16;
			}else{
				txDataFrmPtr->PkgLength = DATA_FIXED_AREA_LENGTH_CRC8;
			}
            txDataFrmPtr->PkgSn = PkgNo++;
            txDataFrmPtr->DeviceType = Dev_Concentrator;
            txDataFrmPtr->Life_Ack.LifeCycle = 0x0F;
            txDataFrmPtr->Life_Ack.AckChannel = DEFAULT_RX_CHANNEL;
            txDataFrmPtr->RouteInfo.CurPos = 0;
            txDataFrmPtr->RouteInfo.Level = Data_GetRoute(nodeId, (uint8 *)(txDataFrmPtr->Route)) & 0x0F;
            txDataFrmPtr->DataLen = 0;
            txDataFrmPtr->Command = Read_Meter_Data;
            txDataFrmPtr->DataBuf[txDataFrmPtr->DataLen++] = (RealTimeDataMode == Concentrator.Param.WorkType) ? RealTimeData : FwFreezingData;
            Rtc_Get((RTC_TIME *)(txDataFrmPtr->DataBuf + txDataFrmPtr->DataLen), Format_Bcd);
            txDataFrmPtr->DataLen += sizeof(RTC_TIME) - 1;
            memset((uint8 *)(txDataFrmPtr->DataBuf + txDataFrmPtr->DataLen), 0, 12);
            txDataFrmPtr->DataLen += 12;
            Data_GetTimeSlot(SubNodes[nodeId].LongAddr, (uint8 *)(&txDataFrmPtr->DataBuf + txDataFrmPtr->DataLen));
            txDataFrmPtr->DataLen += 4;
			version = SubNodes[nodeId].RxMeterVersion;

            // 向任务中添加回调的标识
            taskPtr->Command = txDataFrmPtr->Command;
            taskPtr->NodeId = nodeId;
            taskPtr->PkgSn = txDataFrmPtr->PkgSn;
            taskPtr->RouteLevel = txDataFrmPtr->RouteInfo.Level;

            // 创建发送数据包,执行完毕后,该缓冲区将被释放或由别的指针获取
            DataHandle_SetPkgPath(txDataFrmPtr, UNREVERSED);
            DataHandle_CreateTxData(txDataFrmPtr, version);

            // 等待服务器的应答
            rxDataFrmPtr = OSMboxPend(taskPtr->Mbox, TIME_DELAY_MS((taskPtr->RouteLevel - 1) * DELAYTIME_ONE_LAYER * 2), &err);
            if ((void *)0 != rxDataFrmPtr) {
#if PRINT_INFO
                Gprs_OutputDebugMsg(0, "\n 应答: ");
                OSTimeDlyHMSM(0, 0, 0, 500);
                DebugOutputLength(SubNodes[nodeId].LongAddr, LONG_ADDR_SIZE);
                OSTimeDlyHMSM(0, 0, 0, 500);
#endif
                DataHandle_MeterDataSaveProc(rxDataFrmPtr);
                OSMemPut(LargeMemoryPtr, rxDataFrmPtr);
                OSTimeDlyHMSM(0, 0, 0, 300);
            } else {
#if PRINT_INFO
                Gprs_OutputDebugMsg(0, "\n 无应答: ");
                OSTimeDlyHMSM(0, 0, 0, 500);
                DebugOutputLength(SubNodes[nodeId].LongAddr, LONG_ADDR_SIZE);
                OSTimeDlyHMSM(0, 0, 0, 500);
#endif
                SubNodes[nodeId].Property.LastResult = 0;
                SubNodes[nodeId].Property.CurRouteNo = (SubNodes[nodeId].Property.CurRouteNo + 1) % MAX_CUSTOM_ROUTES;
            }

            // 是否继续补抄,在补抄过程中可以随时终止
            dataReplenishDay = (Concentrator.Param.DataReplenishDay[0] << 24) |
                               (Concentrator.Param.DataReplenishDay[1] << 16) |
                               (Concentrator.Param.DataReplenishDay[2] << 8)  |
                                Concentrator.Param.DataReplenishDay[3];
            Rtc_Get(&rtcTimer, Format_Bcd);
            lastDay = Rtc_LastDayofMonth(rtcTimer.Year, rtcTimer.Month);
            dayMask = 0x80000000;
            if (rtcTimer.Day >= lastDay) {
                dayMask |= (DayMaskTab[32 - BcdToBin(lastDay)] << 24);
            }
            if (0 == Concentrator.Param.DataReplenishCtrl ||
                rtcTimer.Hour < Concentrator.Param.DataReplenishHour ||
                0 == (dataReplenishDay & dayMask >> BcdToBin(rtcTimer.Day))) {
                retry = 100;
                break;
            }
            if (TRUE == TaskRunStatus.DataForward) {
                DataReplenishRound += 1;
                retry = 100;
                break;
            }
        }
    }

    // 销毁本任务,此处必须先禁止任务调度,否则无法释放本任务占用的内存空间
    DataReplenishTimer = DATAREPLENISH_INTERVAL_TIME;
    OSMboxDel(taskPtr->Mbox, OS_DEL_ALWAYS, &err);
    OSSchedLock();
    OSTaskDel(OS_PRIO_SELF);
    OSMemPut(LargeMemoryPtr, taskPtr->StkPtr);
    taskPtr->StkPtr = (void *)0;
    TaskRunStatus.DataReplenish = FALSE;
    OSSchedUnlock();
}

/************************************************************************************************
* Function Name: DataHandle_DataReplenishProc
* Decription   : 数据补抄处理函数
* Input        : 无
* Output       : 无
* Others       : 用于判断是否启用数据补抄功能
************************************************************************************************/
void DataHandle_DataReplenishProc(void)
{
    uint8 lastDay, err;
    uint32 dataReplenishDay, dayMask;
    RTC_TIME rtcTimer;
    DATA_HANDLE_TASK *taskPtr;

    DataReplenishTimer = DATAREPLENISH_INTERVAL_TIME;
    // 在未打开补抄功能的情况下不补抄
    if (0 == Concentrator.Param.DataReplenishCtrl ||
        TRUE == TaskRunStatus.DataReplenish || TRUE == TaskRunStatus.DataForward) {
        return;
    }

    // 检查是否到了数据补抄的时间点
    dataReplenishDay = (Concentrator.Param.DataReplenishDay[0] << 24) |
                       (Concentrator.Param.DataReplenishDay[1] << 16) |
                       (Concentrator.Param.DataReplenishDay[2] << 8)  |
                        Concentrator.Param.DataReplenishDay[3];
    Rtc_Get(&rtcTimer, Format_Bcd);
    lastDay = Rtc_LastDayofMonth(rtcTimer.Year, rtcTimer.Month);
    dayMask = 0x80000000;
    if (rtcTimer.Day >= lastDay) {
        dayMask |= (DayMaskTab[32 - BcdToBin(lastDay)] << 24);
    }
    if (rtcTimer.Hour < Concentrator.Param.DataReplenishHour ||
        0 == (dataReplenishDay & dayMask >> BcdToBin(rtcTimer.Day))) {
        DataReplenishRound = Concentrator.Param.DataReplenishCount;
        return;
    }
    if (0 == DataReplenishRound) {
        return;
    }

    // 搜索未被占用的空间,创建任务
    if ((void *)0 == (taskPtr = DataHandle_GetEmptyTaskPtr())) {
        return;
    }
    if ((void *)0 == (taskPtr->StkPtr = (OS_STK *)OSMemGetOpt(LargeMemoryPtr, 10, TIME_DELAY_MS(50)))) {
        return;
    }
    taskPtr->Mbox = OSMboxCreate((void *)0);
    taskPtr->Msg = (void *)0;
    DataReplenishRound--;
    if (OS_ERR_NONE != OSTaskCreate(DataHandle_DataReplenishTask, taskPtr,
        taskPtr->StkPtr + MEM_LARGE_BLOCK_LEN / sizeof(OS_STK) - 1, taskPtr->Prio)) {
        DataReplenishRound++;
        OSMemPut(LargeMemoryPtr, taskPtr->StkPtr);
        taskPtr->StkPtr = (void *)0;
        OSMboxDel(taskPtr->Mbox, OS_DEL_ALWAYS, &err);
    }
}

/************************************************************************************************
* Function Name: DataHandle_DataUploadTask
* Decription   : 数据上传处理任务
* Input        : *p_arg-参数指针
* Output       : 无
* Others       : 当需要上传的数据够一包时,或需实时上传时,或一天快结束时,启动数据上传服务器功能
************************************************************************************************/
void DataHandle_DataUploadTask(void *p_arg)
{
    COMMAND_TYPE cmd;
    uint8 i, retry, err, count, dataLen, meterDataLen, FreezeLen;
    uint16 nodeId, highPrioDataCount, rxMeterDataCount, uploadMaxCountOnePkg, *record;
    RTC_TIME rtcTime;
    DATA_HANDLE_TASK *taskPtr;
    DATA_FRAME_STRUCT *txDataFrmPtr, *rxDataFrmPtr;
    METER_DATA_SAVE_FORMAT *meterBufPtr;
    uint8 iStart, iLoop, RealTimeLen,FreezeStatus;
    int useVal = 0;
	uint8 version = 0;

    // 计算未上报的节点的数量
    taskPtr = (DATA_HANDLE_TASK *)p_arg;
    TaskRunStatus.DataUpload = TRUE;
    meterBufPtr = (void *)0;
    record = (void *)0;
    Rtc_Get(&rtcTime, Format_Bcd);

    retry = 5;
    while (retry-- && FALSE == TaskRunStatus.DataForward) {
        DataUploadTimer = DATAUPLOAD_INTERVAL_TIME;
        highPrioDataCount = 0;
        rxMeterDataCount = 0;
        for (nodeId = 0; nodeId < Concentrator.MaxNodeId; nodeId++) {
            if (0 == memcmp(SubNodes[nodeId].LongAddr, NullAddress, LONG_ADDR_SIZE)) {
                continue;
            }
            if ((SubNodes[nodeId].DevType & 0xF0) == 0xF0) {
                continue;
            }
            if (TRUE == SubNodes[nodeId].Property.UploadData) {
                continue;
            }
            if (HIGH == SubNodes[nodeId].Property.UploadPrio) {
                highPrioDataCount++;
            }
            rxMeterDataCount++;
        }

        // 如果没有数据上传则跳出
        if (0 == highPrioDataCount && (0 == rxMeterDataCount || 0 == Concentrator.Param.DataUploadCtrl)) {
            break;
        }

        // 根据工作类型判断各个参数值
        // 表端主动上报冻结数据，上传服务器冻结数据
        // 表端主动上报定时定量数据，上传服务器定时定量数据
        if( (0x0 == Concentrator.Param.DataUploadMode && RealTimeDataMode == Concentrator.Param.WorkType)
            || (0x1 == Concentrator.Param.DataUploadMode && FreezeDataMode == Concentrator.Param.WorkType) ){
            if (RealTimeDataMode == Concentrator.Param.WorkType) {
                uploadMaxCountOnePkg = (GPRS_DATA_MAX_DATA - DATA_FIXED_AREA_LENGTH_CRC8) / (LONG_ADDR_SIZE + REALTIME_DATA_AREA_SIZE);
                dataLen = REALTIME_DATA_AREA_SIZE;
                cmd = Upload_RealTime_Data;
            } else {
                uploadMaxCountOnePkg = (GPRS_DATA_MAX_DATA - DATA_FIXED_AREA_LENGTH_CRC8) / (LONG_ADDR_SIZE + FREEZE_DATA_AREA_SIZE);
                dataLen = FREEZE_DATA_AREA_SIZE;
                cmd = Upload_Freeze_Data;
            }
            meterDataLen = sizeof(METER_DATA_SAVE_FORMAT) - 1 + dataLen;


            // 有需要上传的数据但是没有达到一包(在设置时间到23点之间收到表具数据立即上传)
            if (0 == highPrioDataCount) {
                Rtc_Get(&rtcTime, Format_Bcd);
                if (rxMeterDataCount < uploadMaxCountOnePkg && (rtcTime.Hour < Concentrator.Param.DataUploadTime || rtcTime.Hour >= 0x23)) {
                    break;
                }
            }
            if ((void *)0 == meterBufPtr) {
                if ((void *)0 == (meterBufPtr = OSMemGetOpt(SmallMemoryPtr, 10, TIME_DELAY_MS(50)))) {
                    break;
                }
            }
            if ((void *)0 == record) {
                if ((void *)0 == (record = OSMemGetOpt(SmallMemoryPtr, 10, TIME_DELAY_MS(50)))) {
                    break;
                }
            }
            if ((void *)0 == (txDataFrmPtr = OSMemGetOpt(LargeMemoryPtr, 10, TIME_DELAY_MS(50)))) {
                break;
            }
            txDataFrmPtr->PortNo = Uart_Gprs;
			txDataFrmPtr->PkgProp = DataHandle_SetPkgProperty(XOR_CRC8, NEED_ACK, CMD_PKG, UP_DIR);
			if( XOR_CRC16 == txDataFrmPtr->PkgProp.PkgXor){
				txDataFrmPtr->PkgLength = DATA_FIXED_AREA_LENGTH_CRC16;
			}else{
				txDataFrmPtr->PkgLength = DATA_FIXED_AREA_LENGTH_CRC8;
			}
            txDataFrmPtr->PkgSn = PkgNo++;
            txDataFrmPtr->Command = cmd;
            txDataFrmPtr->DeviceType = Dev_Concentrator;
            txDataFrmPtr->Life_Ack.LifeCycle = 0x0F;
            txDataFrmPtr->Life_Ack.AckChannel = DEFAULT_RX_CHANNEL;
            txDataFrmPtr->RouteInfo.CurPos = 0;
            txDataFrmPtr->RouteInfo.Level = 2;
            memcpy(txDataFrmPtr->Route[0], Concentrator.LongAddr, LONG_ADDR_SIZE);
            memcpy(txDataFrmPtr->Route[1], BroadcastAddrOut, LONG_ADDR_SIZE);
            txDataFrmPtr->DataLen = 1;
            count = 0;
            for (nodeId = 0; nodeId < Concentrator.MaxNodeId; nodeId++) {
                if (0 == memcmp(SubNodes[nodeId].LongAddr, NullAddress, LONG_ADDR_SIZE)) {
                    continue;
                }
                if ((SubNodes[nodeId].DevType & 0xF0) == 0xF0) {
                    continue;
                }
                if (TRUE == SubNodes[nodeId].Property.UploadData) {
                    continue;
                }
                if ((highPrioDataCount > 0 && HIGH == SubNodes[nodeId].Property.UploadPrio) || 0 == highPrioDataCount) {
                    Eeprom_ReadWrite((uint8 *)meterBufPtr, nodeId * NODE_INFO_SIZE, meterDataLen, READ_ATTR);
                    if (0 != memcmp(meterBufPtr->Address, SubNodes[nodeId].LongAddr, LONG_ADDR_SIZE) ||
                        meterBufPtr->Crc8MeterData != CalCrc8(meterBufPtr->MeterData, dataLen)) {
                        Data_MeterDataInit(meterBufPtr, nodeId, meterDataLen);
                        Eeprom_ReadWrite((uint8 *)meterBufPtr, nodeId * NODE_INFO_SIZE, meterDataLen, WRITE_ATTR);
                        continue;
                    } else {
                        memcpy(txDataFrmPtr->DataBuf + txDataFrmPtr->DataLen, SubNodes[nodeId].LongAddr, LONG_ADDR_SIZE);
                        txDataFrmPtr->DataLen += LONG_ADDR_SIZE;
                        memcpy(txDataFrmPtr->DataBuf + txDataFrmPtr->DataLen, meterBufPtr->MeterData, dataLen);
                        txDataFrmPtr->DataLen += dataLen;
                        *(record + count++) = nodeId;
                        if (count >= uploadMaxCountOnePkg) {
                            break;
                        }
                    }
                }
            }
        }
        // 表端主动上报冻结数据，上传服务器定时定量数据
        else if(FreezeDataMode == Concentrator.Param.WorkType && 0x0 == Concentrator.Param.DataUploadMode) {
            uploadMaxCountOnePkg = (GPRS_DATA_MAX_DATA - DATA_FIXED_AREA_LENGTH_CRC8) / (LONG_ADDR_SIZE + REALTIME_DATA_AREA_SIZE);
            FreezeLen = FREEZE_DATA_AREA_SIZE;// 冻结数据长度
            dataLen = REALTIME_DATA_AREA_SIZE;// 定时定量数据长度
            cmd = Upload_RealTime_Data;

            //因为数据保存为冻结数据
            //从eeprom中读取出冻结数据后需要转成定时定量数据上次到服务器
            meterDataLen = sizeof(METER_DATA_SAVE_FORMAT) - 1 + FreezeLen;

            // 有需要上传的数据但是没有达到一包(在设置时间到23点之间收到表具数据立即上传)
            if (0 == highPrioDataCount) {
                Rtc_Get(&rtcTime, Format_Bcd);
                if (rxMeterDataCount < uploadMaxCountOnePkg && (rtcTime.Hour < Concentrator.Param.DataUploadTime || rtcTime.Hour >= 0x23)) {
                        break;
                }
            }
            if ((void *)0 == meterBufPtr) {
                if ((void *)0 == (meterBufPtr = OSMemGetOpt(SmallMemoryPtr, 10, TIME_DELAY_MS(50)))) {
                        break;
                }
            }
            if ((void *)0 == record) {
                if ((void *)0 == (record = OSMemGetOpt(SmallMemoryPtr, 10, TIME_DELAY_MS(50)))) {
                        break;
                }
            }
            if ((void *)0 == (txDataFrmPtr = OSMemGetOpt(LargeMemoryPtr, 10, TIME_DELAY_MS(50)))) {
                break;
            }
            txDataFrmPtr->PortNo = Uart_Gprs;
			txDataFrmPtr->PkgProp = DataHandle_SetPkgProperty(XOR_CRC8, NEED_ACK, CMD_PKG, UP_DIR);
			if( XOR_CRC16 == txDataFrmPtr->PkgProp.PkgXor){
				txDataFrmPtr->PkgLength = DATA_FIXED_AREA_LENGTH_CRC16;
			}else{
				txDataFrmPtr->PkgLength = DATA_FIXED_AREA_LENGTH_CRC8;
			}
            txDataFrmPtr->PkgSn = PkgNo++;
            txDataFrmPtr->Command = cmd;
            txDataFrmPtr->DeviceType = Dev_Concentrator;
            txDataFrmPtr->Life_Ack.LifeCycle = 0x0F;
            txDataFrmPtr->Life_Ack.AckChannel = DEFAULT_RX_CHANNEL;
            txDataFrmPtr->RouteInfo.CurPos = 0;
            txDataFrmPtr->RouteInfo.Level = 2;
            memcpy(txDataFrmPtr->Route[0], Concentrator.LongAddr, LONG_ADDR_SIZE);
            memcpy(txDataFrmPtr->Route[1], BroadcastAddrOut, LONG_ADDR_SIZE);
            txDataFrmPtr->DataLen = 1;
            count = 0;
            for (nodeId = 0; nodeId < Concentrator.MaxNodeId; nodeId++) {
                if (0 == memcmp(SubNodes[nodeId].LongAddr, NullAddress, LONG_ADDR_SIZE)) {
                    continue;
                }
                if ((SubNodes[nodeId].DevType & 0xF0) == 0xF0) {
                    continue;
                }
                if (TRUE == SubNodes[nodeId].Property.UploadData) {
                    continue;
                }
#if PRINT_INFO
                Gprs_OutputDebugMsg(0, "\n 上传表号: ");
                OSTimeDlyHMSM(0, 0, 0, 500);
                DebugOutputLength(SubNodes[nodeId].LongAddr, LONG_ADDR_SIZE);
                OSTimeDlyHMSM(0, 0, 0, 500);
#endif
                if ((highPrioDataCount > 0 && HIGH == SubNodes[nodeId].Property.UploadPrio) || 0 == highPrioDataCount) {
                    Eeprom_ReadWrite((uint8 *)meterBufPtr, nodeId * NODE_INFO_SIZE, meterDataLen, READ_ATTR);
                    if (0 != memcmp(meterBufPtr->Address, SubNodes[nodeId].LongAddr, LONG_ADDR_SIZE) ||
                        meterBufPtr->Crc8MeterData != CalCrc8(meterBufPtr->MeterData, FreezeLen)) {
                        Data_MeterDataInit(meterBufPtr, nodeId, meterDataLen);
                        Eeprom_ReadWrite((uint8 *)meterBufPtr, nodeId * NODE_INFO_SIZE, meterDataLen, WRITE_ATTR);
                        continue;
                    } else {
                        memcpy(txDataFrmPtr->DataBuf + txDataFrmPtr->DataLen, SubNodes[nodeId].LongAddr, LONG_ADDR_SIZE);
                        txDataFrmPtr->DataLen += LONG_ADDR_SIZE;
                        RealTimeLen = 0;
                        txDataFrmPtr->DataBuf[txDataFrmPtr->DataLen + RealTimeLen++] = (uint8)(rtcTime.Year);
                        txDataFrmPtr->DataBuf[txDataFrmPtr->DataLen + RealTimeLen++] = (uint8)(rtcTime.Year >> 8);
                        txDataFrmPtr->DataBuf[txDataFrmPtr->DataLen + RealTimeLen++] = meterBufPtr->MeterData[1];//rtcTimer.Month;
                        txDataFrmPtr->DataBuf[txDataFrmPtr->DataLen + RealTimeLen++] = meterBufPtr->MeterData[2];//rtcTimer.Day;
                        txDataFrmPtr->DataBuf[txDataFrmPtr->DataLen + RealTimeLen++] = 0x23;//meterBufPtr->MeterData[3];//rtcTimer.Hour;
                        //meterBufPtr->MeterData[4];//rtcTimer.Min;
                        iStart = 5;
                        useVal = 0;
                        for(iLoop = 0; iLoop < 4; iLoop ++){
                            useVal <<= 8;
                            useVal += meterBufPtr->MeterData[iStart + 4 - iLoop - 1];
                        }
                        FreezeStatus = SUCCESS;
                        if(useVal > 0xF0FFFFFF){
                            FreezeStatus = ERROR;
                        }
                        useVal *= 1000;
                        useVal += meterBufPtr->MeterData[iStart + 4] + meterBufPtr->MeterData[iStart + 5] * 256;
                        iStart += LONG_ADDR_SIZE;
                        for ( iLoop = 0; iLoop < 47; iLoop++){
                            useVal += meterBufPtr->MeterData[iStart++];
                            useVal += meterBufPtr->MeterData[iStart++] * 256;
                        }
                        meterBufPtr->MeterData[5] = (char)((useVal/1000) & 0xff);
                        meterBufPtr->MeterData[6] = (char)(((useVal/1000) >> 8) & 0xff);
                        meterBufPtr->MeterData[7] = (char)(((useVal/1000) >> 16) & 0xff);
                        meterBufPtr->MeterData[8] = (char)(((useVal/1000) >> 24) & 0xff);
                        meterBufPtr->MeterData[9] = (char)((useVal%1000) & 0xff);
                        meterBufPtr->MeterData[10] = (char)(((useVal%1000) >> 8) & 0xff);

                        memcpy(&txDataFrmPtr->DataBuf[txDataFrmPtr->DataLen + RealTimeLen], &meterBufPtr->MeterData[5], 6); // 正转用量[6]
                        RealTimeLen += 6;
                        memset(&txDataFrmPtr->DataBuf[txDataFrmPtr->DataLen + RealTimeLen], 0, 6); 						// 反转用量[6]
                        RealTimeLen += 6;
                        memcpy(&txDataFrmPtr->DataBuf[txDataFrmPtr->DataLen + RealTimeLen], &meterBufPtr->MeterData[1+104], 8+2);//其他数据[10]
                        *(record + count++) = nodeId;
                        if(ERROR == FreezeStatus ){
                            meterBufPtr->Property.UploadData = SubNodes[nodeId].Property.UploadData = TRUE;
                            count--;
                            txDataFrmPtr->DataLen -= LONG_ADDR_SIZE;
                            memset(&txDataFrmPtr->DataBuf[txDataFrmPtr->DataLen], 0, dataLen);
                        }else{
                            txDataFrmPtr->DataLen += dataLen;
                        }
                        if (count >= uploadMaxCountOnePkg) {
                            break;
                        }
                    }
                }
            }
        }else{
            Gprs_OutputDebugMsg(0,"\nERROR : 表端定时定量数据无法转化为冻结数据上传!!\n");
            break;
        }

        if (0 == count) {
            OSMemPut(LargeMemoryPtr, txDataFrmPtr);
            break;
        }
#if PRINT_INFO
        Gprs_OutputDebugMsg(0,"\n----数据上传处理任务----\n");
        OSTimeDlyHMSM(0, 0, 0, 500);
#endif
        txDataFrmPtr->DataBuf[0] = count;
        taskPtr->Command = txDataFrmPtr->Command;
        taskPtr->NodeId = NULL_U16_ID;
        taskPtr->PkgSn = txDataFrmPtr->PkgSn;

        // 创建发送数据包
        DataHandle_SetPkgPath(txDataFrmPtr, UNREVERSED);
        DataHandle_CreateTxData(txDataFrmPtr, version);

        // 等待服务器的应答
        rxDataFrmPtr = OSMboxPend(taskPtr->Mbox, GPRS_WAIT_ACK_OVERTIME, &err);
        if ((void *)0 != rxDataFrmPtr) {
            // 查找那个节点并更改状态保存起
            if (1 == rxDataFrmPtr->DataLen && OP_Succeed == rxDataFrmPtr->DataBuf[0]) {
                retry = 5;
                for (i = 0; i < count; i++) {
                    nodeId = *(record + i);
                    Eeprom_ReadWrite((uint8 *)meterBufPtr, nodeId * NODE_INFO_SIZE, sizeof(METER_DATA_SAVE_FORMAT) - 1, READ_ATTR);
                    if (0 == memcmp(meterBufPtr->Address, SubNodes[nodeId].LongAddr, LONG_ADDR_SIZE)) {
                        meterBufPtr->Property.UploadData = SubNodes[nodeId].Property.UploadData = TRUE;
                        meterBufPtr->Property.UploadPrio = SubNodes[nodeId].Property.UploadPrio = LOW;
                        Eeprom_ReadWrite((uint8 *)meterBufPtr, nodeId * NODE_INFO_SIZE, sizeof(METER_DATA_SAVE_FORMAT) - 1, WRITE_ATTR);
                    }
                }
            }
            OSMemPut(LargeMemoryPtr, rxDataFrmPtr);
        }
    }

    // 销毁本任务,此处必须先禁止任务调度,否则无法释放本任务占用的内存空间
    if ((void *)0 != meterBufPtr) {
        OSMemPut(SmallMemoryPtr, meterBufPtr);
    }
    if ((void *)0 != record) {
        OSMemPut(SmallMemoryPtr, record);
    }
    DataUploadTimer = DATAUPLOAD_INTERVAL_TIME;
    OSMboxDel(taskPtr->Mbox, OS_DEL_ALWAYS, &err);
    OSSchedLock();
    OSTaskDel(OS_PRIO_SELF);
    OSMemPut(LargeMemoryPtr, taskPtr->StkPtr);
    taskPtr->StkPtr = (void *)0;
    TaskRunStatus.DataUpload = FALSE;
    OSSchedUnlock();
}

/************************************************************************************************
* Function Name: DataHandle_DataUploadProc
* Decription   : 数据上传服务器处理函数
* Input        : 无
* Output       : 无
* Others       : 判断是否有数据需要上传并启动上传任务
************************************************************************************************/
void DataHandle_DataUploadProc(void)
{
    uint8 err;
    DATA_HANDLE_TASK *taskPtr;

    DataUploadTimer = DATAUPLOAD_INTERVAL_TIME;

    // Gprs必须在线,并且上传任务没有运行
    if (FALSE == Gprs.Online || TRUE == TaskRunStatus.DataUpload || TRUE == TaskRunStatus.DataForward) {
        return;
    }
    // 搜索未被占用的空间,创建数据上传任务
    if ((void *)0 == (taskPtr = DataHandle_GetEmptyTaskPtr())) {
        return;
    }
    if ((void *)0 == (taskPtr->StkPtr = (OS_STK *)OSMemGetOpt(LargeMemoryPtr, 10, TIME_DELAY_MS(50)))) {
        return;
    }
    taskPtr->Mbox = OSMboxCreate((void *)0);
    taskPtr->Msg = (void *)0;
    if (OS_ERR_NONE != OSTaskCreate(DataHandle_DataUploadTask, taskPtr,
        taskPtr->StkPtr + MEM_LARGE_BLOCK_LEN / sizeof(OS_STK) - 1, taskPtr->Prio)) {
        OSMemPut(LargeMemoryPtr, taskPtr->StkPtr);
        taskPtr->StkPtr = (void *)0;
        OSMboxDel(taskPtr->Mbox, OS_DEL_ALWAYS, &err);
    }
}




/************************************************************************************************
* Function Name: DataHandle_DataForwardTask
* Decription   : 数据转发处理任务
* Input        : p_arg-保存原来数据的指针
* Output       : 无
* Others       : 该函数用处理PC,串口,手持机发送数据后的等待任务
************************************************************************************************/
void DataHandle_DataForwardTask(void *p_arg)
{
    uint8 err;
    uint32 waitAckTime, startTime;
    DATA_HANDLE_TASK *taskPtr;
    DATA_FRAME_STRUCT *txDataFrmPtr, *rxDataFrmPtr;
	uint8 version = 0;

    taskPtr = (DATA_HANDLE_TASK *)p_arg;
    txDataFrmPtr = (DATA_FRAME_STRUCT *)(taskPtr->Msg);

    TaskRunStatus.DataForward = TRUE;
    startTime = Timer1ms;
    waitAckTime = (taskPtr->RouteLevel - 1) * DELAYTIME_ONE_LAYER * 2;
    while (waitAckTime > 0) {
        rxDataFrmPtr = OSMboxPend(taskPtr->Mbox, TIME_DELAY_MS(waitAckTime), &err);
        if ((void *)0 == rxDataFrmPtr) {
            txDataFrmPtr->DataBuf[7] = OP_OvertimeError;
            txDataFrmPtr->DataLen = 8;
            if (NULL_U16_ID != taskPtr->NodeId) {
                SubNodes[taskPtr->NodeId].Property.CurRouteNo = (SubNodes[taskPtr->NodeId].Property.CurRouteNo + 1) % MAX_CUSTOM_ROUTES;
            }
        } else {
            // 根据msg构建应答数据
            DataHandle_MeterDataSaveProc(rxDataFrmPtr);
            if (NULL_U16_ID == taskPtr->NodeId && 0 != memcmp(rxDataFrmPtr->Route[0], &txDataFrmPtr->DataBuf[1], LONG_ADDR_SIZE)) {
                OSMemPut(LargeMemoryPtr, rxDataFrmPtr);
                waitAckTime = (Timer1ms > startTime) ? (Timer1ms - startTime) : 0;
                continue;
            }
            txDataFrmPtr->PkgLength -= txDataFrmPtr->DataLen;
            txDataFrmPtr->DataLen = 1 + LONG_ADDR_SIZE;
            txDataFrmPtr->DataBuf[txDataFrmPtr->DataLen++] = OP_Succeed;
            memcpy(&txDataFrmPtr->DataBuf[txDataFrmPtr->DataLen], rxDataFrmPtr->DataBuf, rxDataFrmPtr->DataLen);
            txDataFrmPtr->DataLen += rxDataFrmPtr->DataLen;
            txDataFrmPtr->DataBuf[txDataFrmPtr->DataLen++] = rxDataFrmPtr->DownRssi;
            txDataFrmPtr->DataBuf[txDataFrmPtr->DataLen++] = rxDataFrmPtr->UpRssi;
            txDataFrmPtr->PkgLength += txDataFrmPtr->DataLen;
            OSMemPut(LargeMemoryPtr, rxDataFrmPtr);
        }
        break;
    }

    // 创建应答数据包
    txDataFrmPtr->PkgProp = DataHandle_SetPkgProperty(txDataFrmPtr->PkgProp.PkgXor, NONE_ACK, ACK_PKG, UP_DIR);
    DataHandle_SetPkgPath(txDataFrmPtr, REVERSED);
    DataHandle_CreateTxData(txDataFrmPtr, version);

    // 销毁本任务,此处必须先禁止任务调度,否则无法释放本任务占用的内存空间
    OSMboxDel(taskPtr->Mbox, OS_DEL_ALWAYS, &err);
    OSSchedLock();
    OSTaskDel(OS_PRIO_SELF);
    OSMemPut(LargeMemoryPtr, taskPtr->StkPtr);
    taskPtr->StkPtr = (void *)0;
    TaskRunStatus.DataForward = FALSE;
    OSSchedUnlock();
}

/************************************************************************************************
* Function Name: DataHandle_DataForwardProc
* Decription   : 数据转发处理任务,只处理接收事件
* Input        : DataFrmPtr-接收到的数据指针
* Output       : 是否需要后续处理
* Others       : 下行:命令字(1)+目标节点地址(6)+转发的数据(N) + 信道选择(1)
*                上行:命令字(1)+目标节点地址(6)+转发的结果(1)+转发的应答数据(N)
************************************************************************************************/
bool DataHandle_DataForwardProc(DATA_FRAME_STRUCT *DataFrmPtr)
{
    uint8 i, err;
    uint16 nodeId;
    DATA_HANDLE_TASK *taskPtr;
    DATA_FRAME_STRUCT *rxDataFrmPtr;
	uint8 version = 0;

    // 检查节点是否合法
    nodeId = Data_FindNodeId(0, &DataFrmPtr->DataBuf[1]);

    // 节点不存在集中器档案中或有任务在处理中
    if (TRUE == TaskRunStatus.DataForward) {
        if (NEED_ACK == DataFrmPtr->PkgProp.NeedAck) {
            DataFrmPtr->DataBuf[1 + LONG_ADDR_SIZE] = NULL_U16_ID == nodeId ? OP_ObjectNotExist : OP_Failure;
            DataFrmPtr->DataLen = 2 + LONG_ADDR_SIZE;
            return NEED_ACK;
        }
        OSMemPut(LargeMemoryPtr, DataFrmPtr);
        return NONE_ACK;
    }

    // 如果此时有数据补抄任务进行中,则等待数据补抄任务结束再进行转发任务
    if (TRUE == TaskRunStatus.DataReplenish) {
        i = 60;
        TaskRunStatus.DataForward = TRUE;
        while (--i && TRUE == TaskRunStatus.DataReplenish) {
            OSTimeDlyHMSM(0, 0, 1, 0);
        }
        TaskRunStatus.DataForward = FALSE;
    }

    // 申请空间保存当前的数据
    if ((void *)0 == (rxDataFrmPtr = OSMemGetOpt(LargeMemoryPtr, 10, TIME_DELAY_MS(50)))) {
        if (NEED_ACK == DataFrmPtr->PkgProp.NeedAck) {
            DataFrmPtr->DataBuf[1 + LONG_ADDR_SIZE] = OP_Failure;
            DataFrmPtr->DataLen = 2 + LONG_ADDR_SIZE;
            return NEED_ACK;
        }
        OSMemPut(LargeMemoryPtr, DataFrmPtr);
        return NONE_ACK;
    }
    memcpy(rxDataFrmPtr, DataFrmPtr, MEM_LARGE_BLOCK_LEN);

    // 节点在集中器档案中则创建转发
    DataFrmPtr->PortNo = Usart_Rf;
    DataFrmPtr->Command = (COMMAND_TYPE)(rxDataFrmPtr->DataBuf[0]);
    //DataFrmPtr->DeviceType = rxDataFrmPtr->DeviceType;
	DataFrmPtr->DeviceType = Dev_Concentrator;
    DataFrmPtr->PkgSn = PkgNo++;
    DataFrmPtr->Life_Ack.LifeCycle = 0x0F;
    DataFrmPtr->Life_Ack.AckChannel = DEFAULT_RX_CHANNEL;
    if (NULL_U16_ID == nodeId) {
        DataFrmPtr->RouteInfo.Level = 2;
        memcpy(DataFrmPtr->Route[0], Concentrator.LongAddr, LONG_ADDR_SIZE);
        memcpy(DataFrmPtr->Route[1], &DataFrmPtr->DataBuf[1], LONG_ADDR_SIZE);
    } else {
        DataFrmPtr->RouteInfo.Level = Data_GetRoute(nodeId, (uint8 *)(DataFrmPtr->Route)) & 0x0F;
    }
    DataFrmPtr->RouteInfo.CurPos = 0;
    for (i = 0; i < rxDataFrmPtr->DataLen - 1 - LONG_ADDR_SIZE; i++) {
        DataFrmPtr->DataBuf[i] = rxDataFrmPtr->DataBuf[i + 1 + LONG_ADDR_SIZE];
    }
    if( DataFrmPtr->DataBuf[i-1] == 0x44 && DataFrmPtr->DataBuf[i-2] == 0x44 ){
        version = 0x44;
    }
    DataFrmPtr->DataLen = i-2;

    // 如果需要应答,则创建应答任务
    if (NEED_ACK == rxDataFrmPtr->PkgProp.NeedAck) {
        if ((void *)0 == (taskPtr = DataHandle_GetEmptyTaskPtr())) {
            goto DATA_FORWARD_FAILED;
        }
        if ((void *)0 == (taskPtr->StkPtr = (OS_STK *)OSMemGetOpt(LargeMemoryPtr, 10, TIME_DELAY_MS(50)))) {
            goto DATA_FORWARD_FAILED;
        }
        taskPtr->Command = DataFrmPtr->Command;
        taskPtr->NodeId = nodeId;
        taskPtr->PkgSn = DataFrmPtr->PkgSn;
        taskPtr->RouteLevel = DataFrmPtr->RouteInfo.Level;
        taskPtr->PortNo = DataFrmPtr->PortNo;
        taskPtr->Mbox = OSMboxCreate((void *)0);
        taskPtr->Msg = (uint8 *)rxDataFrmPtr;
        if (OS_ERR_NONE != OSTaskCreate(DataHandle_DataForwardTask, taskPtr, taskPtr->StkPtr + MEM_LARGE_BLOCK_LEN / sizeof(OS_STK) - 1, taskPtr->Prio)) {
            OSMemPut(LargeMemoryPtr, taskPtr->StkPtr);
            taskPtr->StkPtr = (void *)0;
            OSMboxDel(taskPtr->Mbox, OS_DEL_ALWAYS, &err);
            goto DATA_FORWARD_FAILED;
        }
    } else {
        OSMemPut(LargeMemoryPtr, rxDataFrmPtr);
    }
    DataFrmPtr->PkgProp = DataHandle_SetPkgProperty(DataFrmPtr->PkgProp.PkgXor, NEED_ACK, CMD_PKG, DOWN_DIR);
    DataHandle_SetPkgPath(DataFrmPtr, UNREVERSED);
    DataHandle_CreateTxData(DataFrmPtr, version);
    return NONE_ACK;

DATA_FORWARD_FAILED:
    if (NEED_ACK == rxDataFrmPtr->PkgProp.NeedAck) {
        memcpy(DataFrmPtr, rxDataFrmPtr, MEM_LARGE_BLOCK_LEN);
        OSMemPut(LargeMemoryPtr, rxDataFrmPtr);
        DataFrmPtr->DataBuf[1 + LONG_ADDR_SIZE] = OP_Failure;
        DataFrmPtr->DataLen = 2 + LONG_ADDR_SIZE;
        return NEED_ACK;
    }
    OSMemPut(LargeMemoryPtr, rxDataFrmPtr);
    return NONE_ACK;
}

/************************************************************************************************
* Function Name: DataHandle_ReadSecondChannelTask
* Decription   : 读取第二扫描信道处理任务
* Input        : p_arg-保存原来数据的指针
* Output       : 无
* Others       : 该函数用处理PC,串口,手持机发送数据后的等待任务
************************************************************************************************/
void DataHandle_ReadSecondChannelTask(void *p_arg)
{
    uint8 err;
    uint32 waitAckTime, startTime;
    DATA_HANDLE_TASK *taskPtr;
    DATA_FRAME_STRUCT *txDataFrmPtr, *rxDataFrmPtr;
	uint8 version = 0;

    taskPtr = (DATA_HANDLE_TASK *)p_arg;
    txDataFrmPtr = (DATA_FRAME_STRUCT *)(taskPtr->Msg);

    TaskRunStatus.DataForward = TRUE;
    startTime = Timer1ms;
    waitAckTime = (taskPtr->RouteLevel - 1) * DELAYTIME_ONE_LAYER * 2;
    while (waitAckTime > 0) {
        rxDataFrmPtr = OSMboxPend(taskPtr->Mbox, TIME_DELAY_MS(waitAckTime), &err);
        if ((void *)0 == rxDataFrmPtr) {
            txDataFrmPtr->DataBuf[7] = OP_OvertimeError;
            txDataFrmPtr->DataLen = 8;
            if (NULL_U16_ID != taskPtr->NodeId) {
                SubNodes[taskPtr->NodeId].Property.CurRouteNo = (SubNodes[taskPtr->NodeId].Property.CurRouteNo + 1) % MAX_CUSTOM_ROUTES;
            }
        } else {
			if ( 0x02 == rxDataFrmPtr->DataBuf[0] && 0xAA == rxDataFrmPtr->DataBuf[1] ){
				if( (Concentrator.SaveSecondChannel != (rxDataFrmPtr->DataBuf[2] & 0xF)) &&
					((rxDataFrmPtr->DataBuf[2] & 0xF) >= 0x0) && ((rxDataFrmPtr->DataBuf[2] & 0xF) <= 0xF)){
					Concentrator.SaveSecondChannel = rxDataFrmPtr->DataBuf[2] & 0xF;
				}
			}
            if (NULL_U16_ID == taskPtr->NodeId && 0 != memcmp(rxDataFrmPtr->Route[0], &txDataFrmPtr->DataBuf[1], LONG_ADDR_SIZE)) {
                OSMemPut(LargeMemoryPtr, rxDataFrmPtr);
                waitAckTime = (Timer1ms > startTime) ? (Timer1ms - startTime) : 0;
                continue;
            }
            txDataFrmPtr->PkgLength -= txDataFrmPtr->DataLen;
            txDataFrmPtr->DataLen = 1 + LONG_ADDR_SIZE;
            txDataFrmPtr->DataBuf[txDataFrmPtr->DataLen++] = OP_Succeed;
            memcpy(&txDataFrmPtr->DataBuf[txDataFrmPtr->DataLen], rxDataFrmPtr->DataBuf, rxDataFrmPtr->DataLen);
            txDataFrmPtr->DataLen += rxDataFrmPtr->DataLen;
            txDataFrmPtr->DataBuf[txDataFrmPtr->DataLen++] = rxDataFrmPtr->DownRssi;
            txDataFrmPtr->DataBuf[txDataFrmPtr->DataLen++] = rxDataFrmPtr->UpRssi;
            txDataFrmPtr->PkgLength += txDataFrmPtr->DataLen;
            OSMemPut(LargeMemoryPtr, rxDataFrmPtr);
        }
        break;
    }

    // 创建应答数据包
    txDataFrmPtr->PkgProp = DataHandle_SetPkgProperty(txDataFrmPtr->PkgProp.PkgXor, NONE_ACK, ACK_PKG, UP_DIR);
    DataHandle_SetPkgPath(txDataFrmPtr, REVERSED);
    DataHandle_CreateTxData(txDataFrmPtr, version);

    // 销毁本任务,此处必须先禁止任务调度,否则无法释放本任务占用的内存空间
    OSMboxDel(taskPtr->Mbox, OS_DEL_ALWAYS, &err);
    OSSchedLock();
    OSTaskDel(OS_PRIO_SELF);
    OSMemPut(LargeMemoryPtr, taskPtr->StkPtr);
    taskPtr->StkPtr = (void *)0;
    TaskRunStatus.DataForward = FALSE;
    OSSchedUnlock();
}


/************************************************************************************************
* Function Name: DataHandle_ReadSecondChannelProc
* Decription   : 读取第二扫描信道
* Input        : DataFrmPtr-接收到的数据指针
* Output       : 是否需要后续处理
* Others       : 下行:命令字(1)+目标节点地址(6)+转发的数据(N)
*                上行:命令字(1)+目标节点地址(6)+转发的结果(1)+转发的应答数据(N)
************************************************************************************************/
bool DataHandle_ReadSecondChannelProc(DATA_FRAME_STRUCT *DataFrmPtr)
{
    uint8 i, err;
    uint16 nodeId;
    DATA_HANDLE_TASK *taskPtr;
    DATA_FRAME_STRUCT *rxDataFrmPtr;
	uint8 version = 0;

    // 检查节点是否合法
    nodeId = Data_FindNodeId(0, &DataFrmPtr->DataBuf[1]);

    // 节点不存在集中器档案中或有任务在处理中
    if (TRUE == TaskRunStatus.DataForward) {
        if (NEED_ACK == DataFrmPtr->PkgProp.NeedAck) {
            DataFrmPtr->DataBuf[1 + LONG_ADDR_SIZE] = NULL_U16_ID == nodeId ? OP_ObjectNotExist : OP_Failure;
            DataFrmPtr->DataLen = 2 + LONG_ADDR_SIZE;
            return NEED_ACK;
        }
        OSMemPut(LargeMemoryPtr, DataFrmPtr);
        return NONE_ACK;
    }

    // 如果此时有数据补抄任务进行中,则等待数据补抄任务结束再进行转发任务
    if (TRUE == TaskRunStatus.DataReplenish) {
        i = 60;
        TaskRunStatus.DataForward = TRUE;
        while (--i && TRUE == TaskRunStatus.DataReplenish) {
            OSTimeDlyHMSM(0, 0, 1, 0);
        }
        TaskRunStatus.DataForward = FALSE;
    }

    // 申请空间保存当前的数据
    if ((void *)0 == (rxDataFrmPtr = OSMemGetOpt(LargeMemoryPtr, 10, TIME_DELAY_MS(50)))) {
        if (NEED_ACK == DataFrmPtr->PkgProp.NeedAck) {
            DataFrmPtr->DataBuf[1 + LONG_ADDR_SIZE] = OP_Failure;
            DataFrmPtr->DataLen = 2 + LONG_ADDR_SIZE;
            return NEED_ACK;
        }
        OSMemPut(LargeMemoryPtr, DataFrmPtr);
        return NONE_ACK;
    }
    memcpy(rxDataFrmPtr, DataFrmPtr, MEM_LARGE_BLOCK_LEN);

    // 节点在集中器档案中则创建转发
    DataFrmPtr->PortNo = Usart_Rf;
    DataFrmPtr->Command = (COMMAND_TYPE)(rxDataFrmPtr->DataBuf[0]);
	DataFrmPtr->DeviceType = Dev_Concentrator;
    DataFrmPtr->PkgSn = PkgNo++;
    DataFrmPtr->Life_Ack.LifeCycle = 0x0F;
    DataFrmPtr->Life_Ack.AckChannel = DEFAULT_RX_CHANNEL;
    if (NULL_U16_ID == nodeId) {
        DataFrmPtr->RouteInfo.Level = 2;
        memcpy(DataFrmPtr->Route[0], Concentrator.LongAddr, LONG_ADDR_SIZE);
        memcpy(DataFrmPtr->Route[1], &DataFrmPtr->DataBuf[1], LONG_ADDR_SIZE);
    } else {
        DataFrmPtr->RouteInfo.Level = Data_GetRoute(nodeId, (uint8 *)(DataFrmPtr->Route)) & 0x0F;
    }
    DataFrmPtr->RouteInfo.CurPos = 0;
    for (i = 0; i < rxDataFrmPtr->DataLen - 1 - LONG_ADDR_SIZE; i++) {
        DataFrmPtr->DataBuf[i] = rxDataFrmPtr->DataBuf[i + 1 + LONG_ADDR_SIZE];
    }
    if( DataFrmPtr->DataBuf[i-1] == 0x44 && DataFrmPtr->DataBuf[i-2] == 0x44 ){
        version = 0x44;
    }

    // 如果需要应答,则创建应答任务
    if (NEED_ACK == rxDataFrmPtr->PkgProp.NeedAck) {
        if ((void *)0 == (taskPtr = DataHandle_GetEmptyTaskPtr())) {
            goto DATA_FORWARD_FAILED;
        }
        if ((void *)0 == (taskPtr->StkPtr = (OS_STK *)OSMemGetOpt(LargeMemoryPtr, 10, TIME_DELAY_MS(50)))) {
            goto DATA_FORWARD_FAILED;
        }
        taskPtr->Command = DataFrmPtr->Command;
        taskPtr->NodeId = nodeId;
        taskPtr->PkgSn = DataFrmPtr->PkgSn;
        taskPtr->RouteLevel = DataFrmPtr->RouteInfo.Level;
        taskPtr->PortNo = DataFrmPtr->PortNo;
        taskPtr->Mbox = OSMboxCreate((void *)0);
        taskPtr->Msg = (uint8 *)rxDataFrmPtr;
        if (OS_ERR_NONE != OSTaskCreate(DataHandle_ReadSecondChannelTask, taskPtr, taskPtr->StkPtr + MEM_LARGE_BLOCK_LEN / sizeof(OS_STK) - 1, taskPtr->Prio)) {
            OSMemPut(LargeMemoryPtr, taskPtr->StkPtr);
            taskPtr->StkPtr = (void *)0;
            OSMboxDel(taskPtr->Mbox, OS_DEL_ALWAYS, &err);
            goto DATA_FORWARD_FAILED;
        }
    } else {
        OSMemPut(LargeMemoryPtr, rxDataFrmPtr);
    }
	// 此处设置第二信道，2e28 暂时只支持 crc8 校验
    DataFrmPtr->PkgProp = DataHandle_SetPkgProperty(DataFrmPtr->PkgProp.PkgXor, NEED_ACK, CMD_PKG, DOWN_DIR);
    DataHandle_SetPkgPath(DataFrmPtr, UNREVERSED);
    DataHandle_CreateTxData(DataFrmPtr, version);
    return NONE_ACK;

DATA_FORWARD_FAILED:
    if (NEED_ACK == rxDataFrmPtr->PkgProp.NeedAck) {
        memcpy(DataFrmPtr, rxDataFrmPtr, MEM_LARGE_BLOCK_LEN);
        OSMemPut(LargeMemoryPtr, rxDataFrmPtr);
        DataFrmPtr->DataBuf[1 + LONG_ADDR_SIZE] = OP_Failure;
        DataFrmPtr->DataLen = 2 + LONG_ADDR_SIZE;
        return NEED_ACK;
    }
    OSMemPut(LargeMemoryPtr, rxDataFrmPtr);
    return NONE_ACK;
}


/************************************************************************************************
* Function Name: DataHandle_RxCmdProc
* Decription   : 数据处理任务,只处理接收到的命令事件
* Input        : DataFrmPtr-数据帧的指针
* Output       : 无
* Others       : 该函数处理来自表端或服务器或PC机或手持机发送过来的指令并根据指令来应答
************************************************************************************************/
void DataHandle_RxCmdProc(DATA_FRAME_STRUCT *DataFrmPtr)
{
    bool postHandle, reversePath;
    PKG_PROPERTY ackPkgProperty;
	uint8 version = 0;

    postHandle = DataFrmPtr->PkgProp.NeedAck;
    reversePath = REVERSED;
    ackPkgProperty = DataHandle_SetPkgProperty(DataFrmPtr->PkgProp.PkgXor, NONE_ACK, ACK_PKG, UP_DIR);
    switch (DataFrmPtr->Command) {
        // 表端主动上报定时定量或冻结数据 0x01
        case Meter_ActiveReport_Data:
            postHandle = DataHandle_ActiveReportProc(DataFrmPtr);
            break;

        // 读集中器版本信息 0x40
        case Read_CONC_Version:
            // 下行:空数据域
            // 上行:程序版本(2)+硬件版本(2)+协议版本(2)
            DataFrmPtr->DataLen = 0;
            DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = (uint8)(SW_VERSION >> 8);
            DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = (uint8)SW_VERSION;
            DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = (uint8)(HW_VERSION >> 8);
            DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = (uint8)HW_VERSION;
            DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = (uint8)(PT_VERSION >> 8);
            DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = (uint8)PT_VERSION;
            break;

        // 读集中器ID 0x41
        case Read_CONC_ID:
            // 下行:空数据域
            // 上行:集中器ID的BCD码(6)
            memcpy(DataFrmPtr->DataBuf, Concentrator.LongAddr, LONG_ADDR_SIZE);
            DataFrmPtr->DataLen = LONG_ADDR_SIZE;
            break;

        // 写集中器ID 0x42
        case Write_CONC_ID:
            Data_SetConcentratorAddr(DataFrmPtr);
            break;

        // 读集中器时钟 0x43
        case Read_CONC_RTC:
            // 下行:空数据域
            // 上行:集中器时钟(7)
            Rtc_Get((RTC_TIME *)DataFrmPtr->DataBuf, Format_Bcd);
            DataFrmPtr->DataLen = 7;
            break;

        // 写集中器时钟 0x44
        case Write_CONC_RTC:
            // 下行:集中器时钟(7)
            // 上行:操作状态(1)
            if (SUCCESS == Rtc_Set(*(RTC_TIME *)(DataFrmPtr->DataBuf), Format_Bcd)) {
                DataFrmPtr->DataBuf[0] = OP_Succeed;
                DataReplenishTimer = 30;
                DataUploadTimer = 10;
                DataReplenishRound = Concentrator.Param.DataReplenishCount;
                RTCTimingTimer = RTCTIMING_INTERVAL_TIME;
            } else {
                DataFrmPtr->DataBuf[0] = OP_TimeAbnormal;
            }
            DataFrmPtr->DataLen = 1;
            break;

        // 读Gprs参数 0x45
        case Read_GPRS_Param:
            Data_GprsParameter(DataFrmPtr);
            break;

        // 写Gprs参数 0x46
        case Write_GPRS_Param:
            Data_GprsParameter(DataFrmPtr);
            break;

        // 读Gprs信号强度 0x47
        case Read_GPRS_RSSI:
            // 下行:无
            // 上行:信号强度
            DataFrmPtr->DataBuf[0] = Gprs_GetCSQ();
            DataFrmPtr->DataBuf[1] = Gprs.Online ? 0x01 : 0x00;
            DataFrmPtr->DataLen = 2;
            DataFrmPtr->DataLen += Gprs_GetIMSI(&DataFrmPtr->DataBuf[DataFrmPtr->DataLen]);
            DataFrmPtr->DataLen += Gprs_GetGMM(&DataFrmPtr->DataBuf[DataFrmPtr->DataLen]);
            break;

        // 集中器初始化 0x48
        case Initial_CONC_Cmd:
            // 下行:操作类别
            // 上行:操作类别+操作状态
            DataFrmPtr->DataBuf[1] = OP_Succeed;
            if (0 == DataFrmPtr->DataBuf[0]) {
                Data_ClearDatabase();
            } else if(1 == DataFrmPtr->DataBuf[0]) {
                Data_ClearMeterData();
            } else {
                DataFrmPtr->DataBuf[1] = OP_Failure;
            }
            DataFrmPtr->DataLen = 2;
            break;

        // 读集中器的工作参数 0x49
        case Read_CONC_Work_Param:
            Data_RdWrConcentratorParam(DataFrmPtr);
            break;

        // 写集中器的工作参数 0x4A
        case Write_CONC_Work_Param:
            Data_RdWrConcentratorParam(DataFrmPtr);
            break;

        // 集中器重新启动 0x4C
        case Restart_CONC_Cmd:
            // 下行:无
            // 上行:操作状态
            DataFrmPtr->DataBuf[0] = OP_Succeed;
            DataFrmPtr->DataLen = 1;
            DevResetTimer = 5000;
            break;

        // 集中器数据转发指令 0x4D
        case Data_Forward_Cmd:
            postHandle = DataHandle_DataForwardProc(DataFrmPtr);
            break;

		// 设置集中器第二扫描信道 0x4E
		case Write_Second_Channel_Cmd:
			if( ((DataFrmPtr->DataBuf[0]&0xF) >= 0x0) && (DataFrmPtr->DataBuf[0] <= 0xF) ){
				Concentrator.SecondChannel = DataFrmPtr->DataBuf[0]&0xF;
				DataFrmPtr->DataBuf[0] = OP_Succeed;
				DataFrmPtr->DataLen = 1;
				ModifyScanChannel = 2;
			}else{
				DataFrmPtr->DataBuf[0] = OP_Failure;
				DataFrmPtr->DataLen = 1;
			}
			break;

		// 读集中器第二扫描信道 0x4F
		case Read_Second_Channel_Cmd:
            postHandle = DataHandle_ReadSecondChannelProc(DataFrmPtr);
			break;

        // 表具档案的总数量 0x50
        case Read_Meter_Total_Number:
            Data_ReadNodesCount(DataFrmPtr);
            break;

        // 读取表具档案信息 0x51
        case Read_Meters_Doc_Info:
            Data_ReadNodes(DataFrmPtr);
            break;

        // 写入表具档案信息 0x52
        case Write_Meters_Doc_Info:
            Data_WriteNodes(DataFrmPtr);
            break;

        // 删除表具档案信息 0x53
        case Delete_Meters_Doc_Info:
            Data_DeleteNodes(DataFrmPtr);
            break;

        // 修改表具档案信息 0x54
        case Modify_Meter_Doc_Info:
            Data_ModifyNodes(DataFrmPtr);
            break;

        // 读自定义路由信息 0x55
        case Read_Custom_Route_Info:
            Data_ReadCustomRoute(DataFrmPtr);
            break;

        // 写自定义路由信息 0x56
        case Write_Custom_Route_Info:
            Data_WriteCustomRoute(DataFrmPtr);
            break;

        // 批量读自定义路由信息 0x57
        case Batch_Read_Custom_Routes_Info:
            Data_BatchReadCustomRoutes(DataFrmPtr);
            break;

        // 批量写自定义路由信息 0x58
        case Batch_Write_Custom_Routes_Info:
            Data_BatchWriteCustomRoutes(DataFrmPtr);
            break;

        // 读集中器定时定量数据指令 0x63
        case Read_RealTime_Data:
            DataHandle_MeterDataLoadProc(DataFrmPtr);
            break;

        // 读集中器冻结数据指令 0x64
        case Read_Freeze_Data:
            DataHandle_MeterDataLoadProc(DataFrmPtr);
            break;

        // 批量读集中器定时定量数据指令 0x65
        case Batch_Read_RealTime_Data:
            DataHandle_MeterDataBatchLoadProc(DataFrmPtr);
            break;

         // 批量读集中器冻结数据指令 0x66
        case Batch_Read_Freeze_Data:
            DataHandle_MeterDataBatchLoadProc(DataFrmPtr);
            break;


        // 读集中器下发命令数据指令 0x67
        case Read_CmdDown_Data:
            DataHandle_ReadMeterCmdLoadProc(DataFrmPtr);
            break;

        // 批量读集中器下发命令数据指令 0x68
        case Batch_Read_CmdDown_Data:
            DataHandle_ReadMeterCmdBatchLoadProc(DataFrmPtr);
            break;

        // 批量写集中器下发命令数据指令 0x69
        case Batch_Wtite_CmdDown_Data:
            Data_BatchWriteMeterCmdLoadProc(DataFrmPtr);
            break;


        // 集中器程序升级 0xF1
        case Software_Update_Cmd:
            Data_SwUpdate(DataFrmPtr);
            break;

        // Eeprom检查 0xF3
        case Eeprom_Check_Cmd:
            Data_EepromCheckProc(DataFrmPtr);
            break;

        // 其他指令不支持
        default:
        //Gprs_OutputDebugMsg(TRUE, "--该指令暂不支持--\n");
            postHandle = NONE_ACK;
            OSMemPut(LargeMemoryPtr, DataFrmPtr);
            break;
    }

    if (NEED_ACK == postHandle) {
        DataFrmPtr->PkgProp = ackPkgProperty;
        DataHandle_SetPkgPath(DataFrmPtr, reversePath);
        DataHandle_CreateTxData(DataFrmPtr, version);
    }
}

/************************************************************************************************
* Function Name: DataHandle_RxAckProc
* Decription   : 数据处理任务,只处理接收到的应答事件
* Input        : DataBufPtr-命令数据指针
* Output       : 无
* Others       : 该函数处理来自表端或服务器或PC机或手持机发送过来的应答
************************************************************************************************/
void DataHandle_RxAckProc(DATA_FRAME_STRUCT *DataFrmPtr)
{
    uint8 i;
    uint16 nodeId;
    DATA_HANDLE_TASK *taskPtr;

    // 查找应答节点是否在本档案中
    nodeId = Data_FindNodeId(0, DataFrmPtr->Route[0]);

    // 判断该应答帧应该传递给谁
    for (i = 0; i < MAX_DATA_HANDLE_TASK_NUM; i++) {
        taskPtr = &DataHandle_TaskArry[i];
        if ((void *)0 != taskPtr->StkPtr &&
            taskPtr->NodeId == nodeId &&
            taskPtr->Command == DataFrmPtr->Command &&
            taskPtr->PkgSn == DataFrmPtr->PkgSn) {
            if (OS_ERR_NONE != OSMboxPost(taskPtr->Mbox, DataFrmPtr)) {
                OSMemPut(LargeMemoryPtr, DataFrmPtr);
            }
            return;
        }
    }
    OSMemPut(LargeMemoryPtr, DataFrmPtr);
}

/************************************************************************************************
* Function Name: DataHandle_PassProc
* Decription   : 透传处理任务
* Input        : DataFrmPtr-接收到的数据指针
* Output       : TRUE-已经处理,FALSE-没有处理
* Others       : 目标地址不是自己时,传递到下一个节点
************************************************************************************************/
bool DataHandle_PassProc(DATA_FRAME_STRUCT *DataFrmPtr)
{
    static uint8 lastInPort = End_Port;
	uint8 version = 0;
	uint16 nodeId = 0;
	// 如果是目标节点则跳至下一步处理
	if (DataFrmPtr->RouteInfo.CurPos == DataFrmPtr->RouteInfo.Level - 1) {
		return FALSE;
	}
	// 如果是上行且是倒数第二级则按照设备类型选择通讯端口,其他情况都用RF端口
	if (UP_DIR == DataFrmPtr->PkgProp.Direction &&
		DataFrmPtr->RouteInfo.CurPos == DataFrmPtr->RouteInfo.Level - 2) {
		DataFrmPtr->PortNo = lastInPort;
		DataFrmPtr->Life_Ack.AckChannel = DEFAULT_TX_CHANNEL;
	} else {
		lastInPort = DataFrmPtr->PortNo;
		DataFrmPtr->Life_Ack.AckChannel = DEFAULT_RX_CHANNEL;
		DataFrmPtr->PortNo = Usart_Rf;

		nodeId = Data_FindNodeId(0, DataFrmPtr->Route[DataFrmPtr->RouteInfo.Level - 1]);
		if( TRUE == SubNodes[nodeId].AutoChannelSwitch ){
			version = 0x44;
		}
	}

    DataHandle_CreateTxData(DataFrmPtr, version);
    return TRUE;
}


/************************************************************************************************
* Function Name: DataHandle_Task
* Decription   : 数据处理任务,只处理接收事件
* Input        : *p_arg-参数指针
* Output       : 无
* Others       : 无
************************************************************************************************/
void DataHandle_Task(void *p_arg)
{
    uint8 i, err, *dat;
    OS_FLAGS eventFlag;
    DATA_FRAME_STRUCT *dataFrmPtr;
    EXTRACT_DATA_RESULT ret;

    // 初始化参数
    (void)p_arg;
    PkgNo = CalCrc8(Concentrator.LongAddr, LONG_ADDR_SIZE);
    for (i = 0; i < MAX_DATA_HANDLE_TASK_NUM; i++) {
        DataHandle_TaskArry[i].Prio = TASK_DATAHANDLE_DATA_PRIO + i;
        DataHandle_TaskArry[i].StkPtr = (void *)0;
    }
    TaskRunStatus.DataForward = FALSE;
    TaskRunStatus.DataReplenish = FALSE;
    TaskRunStatus.DataUpload = FALSE;
    TaskRunStatus.RTCService = FALSE;
    TaskRunStatus.RTCTiming = FALSE;
    TaskRunStatus.ModifyScanChannel = FALSE;

    // 数据初始化
    Data_Init();

    while (TRUE) {
        // 获取发生的事件数据
        eventFlag = OSFlagPend(GlobalEventFlag, (OS_FLAGS)DATAHANDLE_EVENT_FILTER, (OS_FLAG_WAIT_SET_ANY | OS_FLAG_CONSUME), TIME_DELAY_MS(5000), &err);

        // 处理这些数据
        while (eventFlag != (OS_FLAGS)0) {
            dat = (void *)0;
            if (eventFlag & FLAG_USART_RF_RX) {
                // Rf模块收到了数据
                dat = OSMboxAccept(SerialPort.Port[Usart_Rf].MboxRx);
                eventFlag &= ~FLAG_USART_RF_RX;
            } else if (eventFlag & FLAG_GPRS_RX) {
                // Gprs模块收到了数据
                dat = OSMboxAccept(Gprs.MboxRx);
                eventFlag &= ~FLAG_GPRS_RX;
            } else if (eventFlag & FLAG_USB_RX) {
                // Usb端口收到了数据
                dat = OSMboxAccept(SerialPort.Port[Usb_Port].MboxRx);
                eventFlag &= ~FLAG_USB_RX;
            } else if (eventFlag & FLAG_USART_DEBUG_RX) {
                // Debug端口收到了数据
                dat = OSMboxAccept(SerialPort.Port[Usart_Debug].MboxRx);
                eventFlag &= ~FLAG_USART_DEBUG_RX;
            } else if (eventFlag & FLAG_UART_RS485_RX) {
                // 485端口收到了数据
                dat = OSMboxAccept(SerialPort.Port[Uart_Rs485].MboxRx);
                eventFlag &= ~FLAG_UART_RS485_RX;
            } else if (eventFlag & FLAG_USART_IR_RX) {
                // Ir端口收到了数据
                dat = OSMboxAccept(SerialPort.Port[Usart_Ir].MboxRx);
                eventFlag &= ~FLAG_USART_IR_RX;
            } else if (eventFlag & FLAG_DELAY_SAVE_TIMER) {
                // 数据延时保存
                eventFlag &= ~FLAG_DELAY_SAVE_TIMER;
                DataHandle_DataDelaySaveProc();
            } else if (eventFlag & FLAG_DATA_REPLENISH_TIMER) {
                // 数据补抄处理
                eventFlag &= ~FLAG_DATA_REPLENISH_TIMER;
                DataHandle_DataReplenishProc();
            } else if (eventFlag & FLAG_DATA_UPLOAD_TIMER) {
                // 数据上传处理
                eventFlag &= ~FLAG_DATA_UPLOAD_TIMER;
                DataHandle_DataUploadProc();
            } else if (eventFlag & FLAG_RTC_TIMING_TIMER) {
                // 时钟主动校时处理
                eventFlag &= ~FLAG_RTC_TIMING_TIMER;
                DataHandle_RTCTimingProc();
            } else if (eventFlag & FLAG_MODIFY_SCAN_CHANNEL) {
                // 修改表端主动上报信道号，通知 2E28 修改扫描信道
                eventFlag &= ~FLAG_MODIFY_SCAN_CHANNEL;
                DataHandle_ModifyScanChannelProc();
            }
            if ((void *)0 == dat) {
                continue;
            }

            // 从原数据中提取数据
            if (Ok_Data != (ret = DataHandle_ExtractData(dat))) {
                if (Error_DstAddress == ret) {
                    // 如果不是给自己的数据可以监听其他节点并将其加入对应的邻居表中
                }
				//else{
                    OSMemPut(LargeMemoryPtr, dat);
                    continue;
                //}
            }

            dataFrmPtr = (DATA_FRAME_STRUCT *)dat;
            // 确定监控信息上传的通道
            if (Usart_Debug == dataFrmPtr->PortNo || Usb_Port == dataFrmPtr->PortNo) {
                MonitorPort = (PORT_NO)(dataFrmPtr->PortNo);
            }

            // 如果目标地址不是自己则转发
            if (TRUE == DataHandle_PassProc(dataFrmPtr)) {
                continue;
            }

            // 分别处理命令帧和应答帧指令
            if (CMD_PKG == dataFrmPtr->PkgProp.PkgType) {
                // 如果是命令帧
                DataHandle_RxCmdProc(dataFrmPtr);
            } else {
                // 如果是应答帧
                DataHandle_RxAckProc(dataFrmPtr);
            }
        }

        OSTimeDlyHMSM(0, 0, 0, 50);
    }
}

/***************************************End of file*********************************************/


