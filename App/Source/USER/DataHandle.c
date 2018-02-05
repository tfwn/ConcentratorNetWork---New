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
PORT_NO MonitorPort = Usart_Debug;                      // ��ض˿�
uint8 SubNodesSaveDelayTimer = 0;                       // ������ʱ����ʱ��
uint8 DataReplenishRound;                               // ���ݲ����ִ�
uint16 DataReplenishTimer = 30;                         // ���ݲ�����ʱ��
uint16 DataUploadTimer = 60;                            // �����ϴ���ʱ��
uint16 ModifyScanChannel  = 0;                          // �޸�2E28ɨ���ŵ���ʱ��
uint16 RTCTimingTimer = 60;                             // RTCУʱ����������ʱ��
TASK_STATUS_STRUCT TaskRunStatus;                       // ��������״̬
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
* Decription   : ����������������յ�����ָ��
* Input        : ��
* Output       : �����ָ��
* Others       : ��
************************************************************************************************/
DATA_HANDLE_TASK *DataHandle_GetEmptyTaskPtr(void)
{
    uint8 i;

    // ����δ��ռ�õĿռ�,���������ϴ�����
    for (i = 0; i < MAX_DATA_HANDLE_TASK_NUM; i++) {
        if ((void *)0 == DataHandle_TaskArry[i].StkPtr) {
            return (&DataHandle_TaskArry[i]);
        }
    }

    // �������ȫ��ʹ�÷��ؿն���
    return ((void *)0);
}

/************************************************************************************************
* Function Name: DataHandle_SetPkgProperty
* Decription   : ���ð�����ֵ
* Input        : PkgXor-��������Ӫ�̱��벻����־: 0-����� 1-���
*                NeedAck-�Ƿ���Ҫ��ִ 0-�����ִ 1-��Ҫ��ִ
*                PkgType-֡���� 0-����֡ 1-Ӧ��֡
*                Dir-�����б�ʶ 0-���� 1-����
* Output       : ����ֵ
* Others       : ��
************************************************************************************************/
PKG_PROPERTY DataHandle_SetPkgProperty(bool PkgXor, bool NeedAck, bool PkgType, bool Dir)
{
    PKG_PROPERTY pkgProp;

    pkgProp.Content = 0;
    pkgProp.PkgXor = PkgXor;//�����ж�����־������bit�任Ϊ crc8 �� crc16 ���жϱ�־λ��
    pkgProp.NeedAck = NeedAck;
    pkgProp.Encrypt = Concentrator.Param.DataEncryptCtrl;
    pkgProp.PkgType = PkgType;
    pkgProp.Direction = Dir;
    return pkgProp;
}

/************************************************************************************************
* Function Name: DataHandle_SetPkgPath
* Decription   : �������ݰ���·��
* Input        : DataFrmPtr-����ָ��
*                ReversePath-�Ƿ���Ҫ��ת·��
* Output       : ��
* Others       : ��
************************************************************************************************/
void DataHandle_SetPkgPath(DATA_FRAME_STRUCT *DataFrmPtr, bool ReversePath)
{
    uint8 i, tmpBuf[LONG_ADDR_SIZE];

    if (0 == memcmp(BroadcastAddrIn, DataFrmPtr->Route[DataFrmPtr->RouteInfo.CurPos], LONG_ADDR_SIZE)) {
        memcpy(DataFrmPtr->Route[DataFrmPtr->RouteInfo.CurPos], Concentrator.LongAddr, LONG_ADDR_SIZE);
    }
    // ·���Ƿ�ת����
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
* Decription   : ��Э����ȡ�����ݲ��������ݵ���ȷ��
* Input        : BufPtr-ԭ����ָ��
* Output       : �ɹ������˵��
* Others       : ע��-�ɹ����ô˺�����BufPtrָ����ȡ���ݺ���ڴ�
************************************************************************************************/
EXTRACT_DATA_RESULT DataHandle_ExtractData(uint8 *BufPtr)
{
    uint8 i, *msg;
    uint16 tmp;
    PORT_BUF_FORMAT *portBufPtr;
    DATA_FRAME_STRUCT *dataFrmPtr;

    // ��Э���ʽ��ȡ��Ӧ������
    portBufPtr = (PORT_BUF_FORMAT *)BufPtr;
    if (FALSE == portBufPtr->Property.FilterDone) {
        return Error_Data;
    }
    // ����һ���ڴ����ڴ����ȡ�������
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

		// ���Crc16�Ƿ���ȷ
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

		// ���Crc8�Ƿ���ȷ
		if (dataFrmPtr->Crc8 != CalCrc8(portBufPtr->Buffer, dataFrmPtr->PkgLength - 2) || portBufPtr->Length < dataFrmPtr->PkgLength) {
			OSMemPut(LargeMemoryPtr, msg);
			return Error_DataCrcCheck;
		}
	}

    // ���������Ƿ��� 0x16
    if ( 0x16 != *(portBufPtr->Buffer + dataFrmPtr->PkgLength - 1)) {
        OSMemPut(LargeMemoryPtr, msg);
        return Error_Data;
    }

    // ����Ƿ�Ϊ�㲥��ַ�򱾻���ַ
    dataFrmPtr->RouteInfo.CurPos += 1;
    if ((0 == memcmp(Concentrator.LongAddr, dataFrmPtr->Route[dataFrmPtr->RouteInfo.CurPos], LONG_ADDR_SIZE) ||
        0 == memcmp(BroadcastAddrIn, dataFrmPtr->Route[dataFrmPtr->RouteInfo.CurPos], LONG_ADDR_SIZE)) &&
        dataFrmPtr->RouteInfo.CurPos < dataFrmPtr->RouteInfo.Level) {
        memcpy(BufPtr, msg, MEM_LARGE_BLOCK_LEN);
        OSMemPut(LargeMemoryPtr, msg);
        return Ok_Data;
    }

    // Ҫ���к�������,���Դ˴�����ȡ�������ݷ���
    memcpy(BufPtr, msg, MEM_LARGE_BLOCK_LEN);
    OSMemPut(LargeMemoryPtr, msg);
    return Error_DstAddress;
}


/************************************************************************************************
* Function Name: Test_DataHandle_CreateTxData
* Decription   : �����������ݰ�
* Input        : DataFrmPtr-�����͵�����
* Output       : �ɹ������
* Others       : �ú���ִ����Ϻ���ͷ�DataBufPtrָ��Ĵ洢��,���Ὣ·�����ĵ�ַ���з�ת
************************************************************************************************/
ErrorStatus Test_DataHandle_CreateTxData(DATA_FRAME_STRUCT *DataFrmPtr, uint8 version)
{
    uint8 err;
    uint16 tmp, nodeId;
    PORT_BUF_FORMAT *txPortBufPtr;
    uint8 ackChannel = 0;

    // ������һ���ڴ������м����ݴ���
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
    txPortBufPtr->Buffer[txPortBufPtr->Length++] = DataFrmPtr->PkgProp.Content;         // ���ı�ʶ
    txPortBufPtr->Buffer[txPortBufPtr->Length++] = DataFrmPtr->PkgSn;                   // �����
    txPortBufPtr->Buffer[txPortBufPtr->Length++] = DataFrmPtr->Command;                 // ������
    // Ϊ�˺��ѳ����ĵ�һ�������
    if (0 == DataFrmPtr->Life_Ack.AckChannel) {
        txPortBufPtr->Buffer[txPortBufPtr->Length++] = Dev_Server;                      // �豸����
    } else {
        txPortBufPtr->Buffer[txPortBufPtr->Length++] = DataFrmPtr->DeviceType;          // �豸����
    }
    txPortBufPtr->Buffer[txPortBufPtr->Length++] = DataFrmPtr->Life_Ack.Content;        // �������ں�Ӧ���ŵ�
    txPortBufPtr->Buffer[txPortBufPtr->Length++] = DataFrmPtr->RouteInfo.Content;       // ·����Ϣ
    memcpy(&txPortBufPtr->Buffer[txPortBufPtr->Length], DataFrmPtr->Route[0], DataFrmPtr->RouteInfo.Level * LONG_ADDR_SIZE);
    txPortBufPtr->Length += DataFrmPtr->RouteInfo.Level * LONG_ADDR_SIZE;
    memcpy(txPortBufPtr->Buffer + txPortBufPtr->Length, DataFrmPtr->DataBuf, DataFrmPtr->DataLen);      // ������
    txPortBufPtr->Length += DataFrmPtr->DataLen;
    txPortBufPtr->Buffer[txPortBufPtr->Length++] = 0x55;                                // �����ź�ǿ��
    txPortBufPtr->Buffer[txPortBufPtr->Length++] = 0x55;                                // �����ź�ǿ��
	if( XOR_CRC16 == DataFrmPtr->PkgProp.PkgXor){
		// crc16У�飬���ֽ���ǰ
	    uint16 crc16 = CalCrc16((uint8 *)(&txPortBufPtr->Buffer[tmp]), txPortBufPtr->Length-tmp);
	    txPortBufPtr->Buffer[txPortBufPtr->Length++] = (uint8)((crc16)&0xFF);
	    txPortBufPtr->Buffer[txPortBufPtr->Length++] = (uint8)((crc16 >> 8)&0xFF);
	} else {
		txPortBufPtr->Buffer[txPortBufPtr->Length] = CalCrc8((uint8 *)(&txPortBufPtr->Buffer[tmp]), txPortBufPtr->Length - tmp);	 // Crc8У��
		txPortBufPtr->Length += 1;
	}
    txPortBufPtr->Buffer[txPortBufPtr->Length++] = TAILBYTE;

	nodeId = Data_FindNodeId(0, DataFrmPtr->Route[DataFrmPtr->RouteInfo.Level - 1]);

    txPortBufPtr->Buffer[txPortBufPtr->Length++] = 0x00;
    // Ϊ�˺��ѳ����ĵ�һ�������
    if (0 == DataFrmPtr->Life_Ack.AckChannel) {
        if ( DataFrmPtr->RouteInfo.Level > 0x2 && version > 0x3){
            // �༶·������� , �Ұ汾�Ÿ��� 3.
            ackChannel = (DataFrmPtr->Route[DataFrmPtr->RouteInfo.CurPos+1][LONG_ADDR_SIZE-1] & 0x0F)/2 + 0x0B;
            txPortBufPtr->Buffer[txPortBufPtr->Length++] = ackChannel;
        } else {
            txPortBufPtr->Buffer[txPortBufPtr->Length++] = (DEFAULT_RX_CHANNEL + CHANNEL_OFFSET);
        }
		txPortBufPtr->Buffer[txPortBufPtr->Length++] = (DEFAULT_RX_CHANNEL + CHANNEL_OFFSET);
    } else {
        if ( DataFrmPtr->RouteInfo.Level > 0x2 && version > 0x3 && SubNodes[nodeId].AutoChannelSwitch == TRUE ){
            // �༶·������� , �Ұ汾�Ÿ��� 3.
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
* Decription   : �����������ݰ�
* Input        : DataFrmPtr-�����͵�����
* Output       : �ɹ������
* Others       : �ú���ִ����Ϻ���ͷ�DataBufPtrָ��Ĵ洢��,���Ὣ·�����ĵ�ַ���з�ת
************************************************************************************************/
ErrorStatus DataHandle_CreateTxData(DATA_FRAME_STRUCT *DataFrmPtr, uint8 version)
{
    uint8 err;
    uint16 tmp, nodeId;
    PORT_BUF_FORMAT *txPortBufPtr;
    uint8 ackChannel = 0;

    // ������һ���ڴ������м����ݴ���
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
    txPortBufPtr->Buffer[txPortBufPtr->Length++] = DataFrmPtr->PkgProp.Content;         // ���ı�ʶ
    txPortBufPtr->Buffer[txPortBufPtr->Length++] = DataFrmPtr->PkgSn;                   // �����
    txPortBufPtr->Buffer[txPortBufPtr->Length++] = DataFrmPtr->Command;                 // ������
    // Ϊ�˺��ѳ����ĵ�һ�������
    if (0 == DataFrmPtr->Life_Ack.AckChannel) {
        txPortBufPtr->Buffer[txPortBufPtr->Length++] = Dev_Server;                      // �豸����
    } else {
        txPortBufPtr->Buffer[txPortBufPtr->Length++] = DataFrmPtr->DeviceType;          // �豸����
    }
    txPortBufPtr->Buffer[txPortBufPtr->Length++] = DataFrmPtr->Life_Ack.Content;        // �������ں�Ӧ���ŵ�
    txPortBufPtr->Buffer[txPortBufPtr->Length++] = DataFrmPtr->RouteInfo.Content;       // ·����Ϣ
    memcpy(&txPortBufPtr->Buffer[txPortBufPtr->Length], DataFrmPtr->Route[0], DataFrmPtr->RouteInfo.Level * LONG_ADDR_SIZE);
    txPortBufPtr->Length += DataFrmPtr->RouteInfo.Level * LONG_ADDR_SIZE;
    memcpy(txPortBufPtr->Buffer + txPortBufPtr->Length, DataFrmPtr->DataBuf, DataFrmPtr->DataLen);      // ������
    txPortBufPtr->Length += DataFrmPtr->DataLen;
    txPortBufPtr->Buffer[txPortBufPtr->Length++] = 0x55;                                // �����ź�ǿ��
    txPortBufPtr->Buffer[txPortBufPtr->Length++] = 0x55;                                // �����ź�ǿ��
	if( XOR_CRC16 == DataFrmPtr->PkgProp.PkgXor){
		// crc16У�飬���ֽ���ǰ
	    uint16 crc16 = CalCrc16((uint8 *)(&txPortBufPtr->Buffer[tmp]), txPortBufPtr->Length-tmp);
	    txPortBufPtr->Buffer[txPortBufPtr->Length++] = (uint8)((crc16)&0xFF);
	    txPortBufPtr->Buffer[txPortBufPtr->Length++] = (uint8)((crc16 >> 8)&0xFF);
	} else {
		txPortBufPtr->Buffer[txPortBufPtr->Length] = CalCrc8((uint8 *)(&txPortBufPtr->Buffer[tmp]), txPortBufPtr->Length - tmp);	 // Crc8У��
		txPortBufPtr->Length += 1;
	}
    txPortBufPtr->Buffer[txPortBufPtr->Length++] = TAILBYTE;

	nodeId = Data_FindNodeId(0, DataFrmPtr->Route[DataFrmPtr->RouteInfo.Level - 1]);

    if (CMD_PKG == DataFrmPtr->PkgProp.PkgType) {
        txPortBufPtr->Buffer[txPortBufPtr->Length++] = 0x1E;
        if (DATA_CENTER_ID == nodeId || NULL_U16_ID == nodeId) {
            if ( version > 0x3 && SubNodes[nodeId].AutoChannelSwitch == TRUE ){
                // �༶·�������
                ackChannel = DataFrmPtr->Route[DataFrmPtr->RouteInfo.CurPos+1][LONG_ADDR_SIZE-1] & 0xF;
                txPortBufPtr->Buffer[txPortBufPtr->Length++] = ackChannel;
            }else{
                txPortBufPtr->Buffer[txPortBufPtr->Length++] = DEFAULT_TX_CHANNEL;
            }
        } else {
            if ( (version > 0x3 && SubNodes[nodeId].AutoChannelSwitch == TRUE) || version == 0x44 ){
                // �༶·�������
                ackChannel = DataFrmPtr->Route[DataFrmPtr->RouteInfo.CurPos+1][LONG_ADDR_SIZE-1] & 0xF;
                txPortBufPtr->Buffer[txPortBufPtr->Length++] = ackChannel;
            }else{
                txPortBufPtr->Buffer[txPortBufPtr->Length++] = SubNodes[nodeId].RxChannel;
            }
        }
        txPortBufPtr->Buffer[txPortBufPtr->Length++] = (DEFAULT_RX_CHANNEL + CHANNEL_OFFSET);
    } else {
        txPortBufPtr->Buffer[txPortBufPtr->Length++] = 0x00;
        // Ϊ�˺��ѳ����ĵ�һ�������
        if (0 == DataFrmPtr->Life_Ack.AckChannel) {
            if ( DataFrmPtr->RouteInfo.Level > 0x2 && version > 0x3){
                // �༶·������� , �Ұ汾�Ÿ��� 3.
                ackChannel = (DataFrmPtr->Route[DataFrmPtr->RouteInfo.CurPos+1][LONG_ADDR_SIZE-1] & 0x0F)/2 + 0x0B;
                txPortBufPtr->Buffer[txPortBufPtr->Length++] = ackChannel;
            } else {
                txPortBufPtr->Buffer[txPortBufPtr->Length++] = (DEFAULT_RX_CHANNEL + CHANNEL_OFFSET);
            }
        } else {
            if ( DataFrmPtr->RouteInfo.Level > 0x2 && version > 0x3 && SubNodes[nodeId].AutoChannelSwitch == TRUE ){
                // �༶·������� , �Ұ汾�Ÿ��� 3.
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

// ****���ڴ����ݵ���
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
// ****���ڴ����ݵ���

/************************************************************************************************
* Function Name: DataHandle_DataDelaySaveProc
* Decription   : ������ʱ���洦����
* Input        : ��
* Output       : ��
* Others       : ������������Ҫ����ʱ����һ����ʱ����,���ӳ�Flash������
************************************************************************************************/
void DataHandle_DataDelaySaveProc(void)
{
    SubNodesSaveDelayTimer = 0;
    Flash_SaveSubNodesInfo();
    Flash_SaveConcentratorInfo();
}

/************************************************************************************************
* Function Name: DataHandle_OutputMonitorMsg
* Decription   : ������������������Ϣ
* Input        : MsgType-��Ϣ������,MsgPtr-�����Ϣָ��,MsgLen-��Ϣ�ĳ���
* Output       : ��
* Others       : ��
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
* Decription   : �������ݱ��洦����
* Input        : DataFrmPtr-ָ������֡��ָ��
* Output       : �ɹ���ʧ��
* Others       : ���ڱ���ʵʱ/��ʱ/�����Ͷ������ݺ��������ź�ǿ��
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
    // ����ռ����ڱ��泭������
    if ((void *)0 == (msg = OSMemGetOpt(SmallMemoryPtr, 10, TIME_DELAY_MS(50)))) {
        return ERROR;
    }
    length = DataFrmPtr->DataLen - 1;       // ��ȥ���ݸ�ʽ����ֽ�
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

    // ������һ���ڴ�,���ڶ�ȡEeprom�е�����
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
    // �ýڵ���շ��ŵ�λ������ĩβ��4���ֽ�(Ҫ���ǵ��շ��ŵ�������)
    SubNodes[nodeId].RxChannel = (uint8)(*(msg + dataLen - UPDOWN_RSSI_SIZE - 2) >> 4);
    SubNodes[nodeId].TxChannel = (uint8)(*(msg + dataLen - UPDOWN_RSSI_SIZE - 2)&0x0F);

	// ��˰汾�ţ�λ������ĩβ�� 3 ���ֽ�(Ҫ���ǵ��շ��ŵ�������)
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

		// ��¼��ǰ�ڵ㳭��ʱ��
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
* Decription   : �������ݶ�ȡ������,�ý����Ӱ�켯���������ϴ�������
* Input        : DataFrmPtr-ָ������֡��ָ��
* Output       : ��
* Others       : ����:����ַ(6)
*                ������������:����״̬(1)+����ַ(6)+��������(N)
*                ʵʱ��������:����״̬(1)+����ַ(6)+����ʱ����(5)+����(N)
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

    // û�д˽ڵ����������������뼯�����Ĺ���ģʽ��һ�»������ڴ�ʧ�ܵ����
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

            memcpy(&dataBufPtr[dataLen], &meterBufPtr->MeterData[5], 6); // ��ת����[6]
            dataLen += 6;
            memset(&dataBufPtr[dataLen], 0, 6);							// ��ת����[6]
            dataLen += 6;
            memcpy(&dataBufPtr[dataLen], &meterBufPtr->MeterData[1+104], 8+2);//��������[10]
            dataLen += 10;

            //�����������̫��
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
* Decription   : ��������������ȡ������,�ý����Ӱ�켯���������ϴ�������
* Input        : DataFrmPtr-ָ������֡��ָ��
* Output       : ��
* Others       : ����:��ʼ�ڵ����(2)+��ȡ������(1)
*                ������������:�ڵ�������(2)+���η��ص�����N(1)+N*(����״̬(1)+����ַ(6)+��������(M))
*                ʵʱ��������:�ڵ�������(2)+���η��ص�����N(1)+N*(����״̬(1)+����ַ(6)+ʱ��(5)+����(M))
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

	// �����������е�������Ϣ���������ʱ����������
	if(Batch_Read_RealTime_Data == cmd && FreezeDataMode == Concentrator.Param.WorkType){
		FreezeLen = FREEZE_DATA_AREA_SIZE;		//�������ݳ���
		dataLen = REALTIME_DATA_AREA_SIZE;		//��ʱ�������ݳ���
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
                    //��Eeprom �ж�ȡ��������
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

                        memcpy(&dataBufPtr[RealTimeLen], &meterBufPtr->MeterData[5], 6); // ��ת����[6]
                        RealTimeLen += 6;
                        memset(&dataBufPtr[RealTimeLen], 0, 6); 						// ��ת����[6]
                        RealTimeLen += 6;
                        memcpy(&dataBufPtr[RealTimeLen], &meterBufPtr->MeterData[1+104], 8+2);//��������[10]
                        RealTimeLen += 10;
						//�����������̫��
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
* Decription   : �·������ȡ������
* Input        : DataFrmPtr-ָ������֡��ָ��
* Output       : ��
* Others       : ����:����ַ(6)
*                ����:����״̬(1)+����ַ(6)+�·���������(N)
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

    // û�д˽ڵ����������������뼯�����Ĺ���ģʽ��һ�»������ڴ�ʧ�ܵ����
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
* Decription   : �·�����������ȡ������
* Input        : DataFrmPtr-ָ������֡��ָ��
* Output       : ��
* Others       : ����:��ʼ�ڵ����(2)+��ȡ������(1)
*                ����:�ڵ�������(2)+���η��ص�����N(1)+N*(����״̬(1)+����ַ(6)+�·���������(M))
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
* Decription   : �����·������������
* Input        : p_arg-����ԭ�����ݵ�ָ��
* Output       : ��
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
		// ������ڶ��ֽ�Ϊ�������
            if( 0xAA == rxDataFrmPtr->DataBuf[bufLen+1] ){
                nodeId = Data_FindNodeId(0, rxDataFrmPtr->Route[0]);
                if (NULL_U16_ID == nodeId ) {
                    OSMemPut(LargeMemoryPtr, rxDataFrmPtr);
                    waitAckTime = (Timer1ms > startTime) ? (Timer1ms - startTime) : 0;
                    continue;
                }
                // ������һ���ڴ�,���ڶ�ȡEeprom�е�����
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
				// �����ŵ� : ��������(0x1) + �������(0xAA��0xAB) + �ŵ���(��4λΪ��������ϱ��ŵ�����4λΪ��ǰͨ���ŵ�)
                if( Meter_ACK_Set_Channel == rxDataFrmPtr->DataBuf[bufLen] )
                {// �����ŵ�
                	if( (SubNodes[nodeId].CmdProp.SetChannel != 0) &&
						((SubNodes[nodeId].CmdData.SetChannel&0x7F) == (rxDataFrmPtr->DataBuf[bufLen+2]&0xF)))
                	{
                		Concentrator.SecondChannel = SubNodes[nodeId].CmdData.SetChannel&0x7F;
						// ����⵽����Զ��ŵ��Ѿ��򿪣����Ҵ�ʱ���������ŵ������ֶ��޸� Eprom �б���� RX/TX ����
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
				// ���Զ��ŵ� : ��������(0x2) + �������(0xAA��0xAB) + 2�ֽڱ��״̬(�ڶ��ֽ� bit0 Ϊ�����Զ��ŵ�)
                else if( Meter_ACK_Open_Auto_Channel == rxDataFrmPtr->DataBuf[bufLen] )
                {// ���Զ��ŵ�����
                	if( (SubNodes[nodeId].CmdProp.AutoChannel != 0) &&
						(((SubNodes[nodeId].CmdData.AutoChannel>>7)&0x1) == ((rxDataFrmPtr->DataBuf[bufLen+3]>>0)&0x1)))
					{
	                    if(SubNodes[nodeId].CmdData.AutoChannel == 0x80){
	                       // ���Զ��ŵ���ʱ����˽����ŵ��Ǳ��β�ŵ����һλ�������ŵ����Ѿ����úõ��ŵ����������ݲ���֪�����߼�����ͳһ���ã���
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
				// ���� : ��������(0x3) + �������(0xAA��0xAB) + ��״̬(1������2�ط�)
                else if( Meter_ACK_Control_Valve == rxDataFrmPtr->DataBuf[bufLen] )
                {// ����   ����λ��1������2�ط�
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
				// ����۸� : ��������(0x4) + �������(0xAA��0xAB) + 2�ֽڽ���۸�(��λ��ǰ����λ�ں�)
                else if( Meter_ACK_Settlement_Price == rxDataFrmPtr->DataBuf[bufLen] )
                {// ����۸�
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
				// ����ʵʱ���� : ��������(0x5) + �������(0xAA��0xAB) + 2�ֽڱ��״̬(�ڶ��ֽ� bit3 Ϊ�����Զ��ŵ�)
				else if( Meter_ACK_Control_RwMeter == rxDataFrmPtr->DataBuf[bufLen] )
                {// ����ʵʱ����
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

    // ���ٱ�����,�˴������Ƚ�ֹ�������,�����޷��ͷű�����ռ�õ��ڴ�ռ�
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
* Decription   : ��������ϱ����ݴ�����
* Input        : DataFrmPtr-���յ�������ָ��
* Output       : �����Ƿ���ҪӦ����
* Others       : ���ڴ��������ϱ���ʱ��������,��������.
************************************************************************************************/
bool DataHandle_ActiveReportProc(DATA_FRAME_STRUCT *DataFrmPtr)
{
    // ���ڵ㲻���ڱ�������������������ʧ�ܵ������ֱ���ϴ����������Ҳ�������Ľ��
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

    	length = DataFrmPtr->DataLen - 1;       // ��ȥ���ݸ�ʽ����ֽ�
        version = DataFrmPtr->DataBuf[length];  // ��汾��
        nodeId = Data_FindNodeId(0, DataFrmPtr->Route[0]);
        if (NULL_U16_ID == nodeId) {
            OSMemPut(LargeMemoryPtr, DataFrmPtr);
            return NONE_ACK;
        }

        // ������������
        DataFrmPtr->DeviceType = Dev_Concentrator;
        DataFrmPtr->DataLen = 0;
        DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = 0xA0;	   // ������
        Rtc_Get((RTC_TIME *)(DataFrmPtr->DataBuf + DataFrmPtr->DataLen), Format_Bcd);
        DataFrmPtr->DataLen += 7;
        memset((uint8 *)(&DataFrmPtr->DataBuf[DataFrmPtr->DataLen]), 0, 12);
        DataFrmPtr->DataLen += 12;
        Data_GetTimeSlot(DataFrmPtr->Route[0], (uint8 *)(&DataFrmPtr->DataBuf[DataFrmPtr->DataLen]));
        DataFrmPtr->DataLen += 4;

        // ������·�������ұ�汾�Ŵ��� 3
        if( SubNodes[nodeId].CmdProp.Content != 0x0 && version > 0x3 ){
            DataFrmPtr->DataBuf[0] = 0xA1;	   // ������
            if( SubNodes[nodeId].CmdProp.SetChannel == 0x1 ){
                // �����ŵ�
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = Meter_ACK_Set_Channel; // �����ŵ�
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = 0x03;					// ���ݳ���
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = 0xDA;					// �����ֽڵ������� 0xDA 0x26
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = 0x26;
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = SubNodes[nodeId].CmdData.SetChannel;	// �ŵ��ţ�Bit7: Ϊ1ʱ�����ŵ�����7bitΪ�ŵ���
            } else if ( SubNodes[nodeId].CmdProp.AutoChannel == 0x1 ){
                // ���Զ��ŵ�����
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = Meter_ACK_Open_Auto_Channel; // �����Զ��ŵ�
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = 0x03;					// ���ݳ���
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = 0xDA;					// �����ֽڵ������� 0xDA 0x26
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = 0x26;
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = SubNodes[nodeId].CmdData.AutoChannel;	// bit7��1��, 0��
            } else if ( SubNodes[nodeId].CmdProp.ValveCtrl == 0x1 ){
                // ����
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = Meter_ACK_Control_Valve; // ����
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = 0x03;					// ���ݳ���
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = 0xDA;					// �����ֽڵ������� 0xDA 0x26
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = 0x26;
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = SubNodes[nodeId].CmdData.ValveCtrl;// ����λ��1������2�ط�
            } else if ( SubNodes[nodeId].CmdProp.RealTimeMeterReading == 0x1 ){
                // ����ʵʱ����
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = Meter_ACK_Control_RwMeter; // �򿪹ر�ʵʱ������
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = 0x03;					// ���ݳ���
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = 0xDA;					// �����ֽڵ������� 0xDA 0x26
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = 0x26;
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = SubNodes[nodeId].CmdData.RealTimeMeterReading;// bit7��1��, 0��
            } else if ( SubNodes[nodeId].CmdProp.SettlementPrice == 0x1 ){
                // ����۸�
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = Meter_ACK_Settlement_Price; // ����۸�
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = 0x04;					// ���ݳ���
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = 0xDA;					// �����ֽڵ������� 0xDA 0x26
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = 0x26;
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = SubNodes[nodeId].CmdData.SettlementPriceLow;// ��λ��ǰ����λ�ں�
                DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = SubNodes[nodeId].CmdData.SettlementPriceHigh;// ��λ��ǰ����λ�ں�
            }

            // �����Ҫ����Ӧ����Ϣ,�򴴽�Ӧ��������
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
* Decription   : ʵʱʱ��Уʱ��������
* Input        : *p_arg-����ָ��
* Output       : ��
* Others       : ��
************************************************************************************************/
void DataHandle_RTCTimingTask(void *p_arg)
{
    uint8 err;
    DATA_HANDLE_TASK *taskPtr;
    DATA_FRAME_STRUCT *txDataFrmPtr, *rxDataFrmPtr;
	uint8 version = 0;

    // ��������Уʱ���ݰ�
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

        // �����������ݰ�
        DataHandle_SetPkgPath(txDataFrmPtr, UNREVERSED);
        DataHandle_CreateTxData(txDataFrmPtr, version);

        // �ȴ���������Ӧ��
        rxDataFrmPtr = OSMboxPend(taskPtr->Mbox, GPRS_WAIT_ACK_OVERTIME, &err);
        if ((void *)0 == rxDataFrmPtr) {
            RTCTimingTimer = 300;               // �����ʱ��5���Ӻ�����
        } else {
            if (SUCCESS == Rtc_Set(*(RTC_TIME *)(rxDataFrmPtr->DataBuf), Format_Bcd)) {
                RTCTimingTimer = RTCTIMING_INTERVAL_TIME;
            } else {
                RTCTimingTimer = 5;             // ���Уʱʧ����5�������
            }
            OSMemPut(LargeMemoryPtr, rxDataFrmPtr);
        }
    }

    // ���ٱ�����,�˴������Ƚ�ֹ�������,�����޷��ͷű�����ռ�õ��ڴ�ռ�
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
* Decription   : ������ʵʱʱ������Уʱ������
* Input        : ��
* Output       : ��
* Others       : ÿ��һ��ʱ�������һ��Уʱ����
************************************************************************************************/
void DataHandle_RTCTimingProc(void)
{
    uint8 err;
    DATA_HANDLE_TASK *taskPtr;

    // ���Gprs�Ƿ����߻��������Ƿ�����������
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
* Decription   : �޸��ŵ���������
* Input        : *p_arg-����ָ��
* Output       : ��
* Others       : ��
************************************************************************************************/
void DataHandle_ModifyScanChannelTask(void *p_arg)
{
    uint8 err;
    DATA_HANDLE_TASK *taskPtr;
    DATA_FRAME_STRUCT *txDataFrmPtr, *rxDataFrmPtr;
    uint8 version = 0;

    // ���������޸��ŵ����ݰ�
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
        // ���������
        txDataFrmPtr->DataBuf[0] = 0x82;// 0x2 ���ŵ��� 0x82 �����ŵ�
        memcpy( &txDataFrmPtr->DataBuf[1], ModifyScanChannel_KEY, sizeof(ModifyScanChannel_KEY));
        txDataFrmPtr->DataLen += sizeof(ModifyScanChannel_KEY) + 1;
        // ��4λ�̶�Ϊ3�ŵ�����4λΪ��Ҫ���õ��ŵ���
        txDataFrmPtr->DataBuf[txDataFrmPtr->DataLen++] = ((0x3 << 4)&0xF0) | (Concentrator.SecondChannel & 0xF);

        taskPtr = (DATA_HANDLE_TASK *)p_arg;
        taskPtr->Command = txDataFrmPtr->Command;
        taskPtr->NodeId = NULL_U16_ID;
        taskPtr->PkgSn = txDataFrmPtr->PkgSn;

        // �����������ݰ�
        DataHandle_SetPkgPath(txDataFrmPtr, UNREVERSED);
        DataHandle_CreateTxData(txDataFrmPtr, version);

        // �ȴ���������Ӧ��
        rxDataFrmPtr = OSMboxPend(taskPtr->Mbox, GPRS_WAIT_ACK_OVERTIME, &err);
        if ((void *)0 == rxDataFrmPtr) {
            ModifyScanChannel = 60;               // �����ʱ��1���Ӻ�����
        } else {
            if ( ( 0x02 == rxDataFrmPtr->DataBuf[0] ) &&
                ( OP_Succeed == rxDataFrmPtr->DataBuf[1] ) &&
                ( Concentrator.SecondChannel == (rxDataFrmPtr->DataBuf[2]&0x0F) ) &&
                ( 0x30 == (rxDataFrmPtr->DataBuf[2]&0xF0) ) ) {
                ModifyScanChannel = 0;
                Concentrator.SaveSecondChannel = rxDataFrmPtr->DataBuf[2]&0x0F;
                Flash_SaveConcentratorInfo();
            } else {
                ModifyScanChannel = 5;             // ���Уʱʧ����5�������
            }
            OSMemPut(LargeMemoryPtr, rxDataFrmPtr);
        }
    }

    // ���ٱ�����,�˴������Ƚ�ֹ�������,�����޷��ͷű�����ռ�õ��ڴ�ռ�
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
* Decription   : �����������޸�2e28ɨ���ŵ�������
* Input        :
* Others       : ��
************************************************************************************************/
void DataHandle_ModifyScanChannelProc(void)
{
	uint8 err;
	DATA_HANDLE_TASK *taskPtr;

	// ����Ƿ���������������������
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
* Decription   : ���ݲ�����������
* Input        : *p_arg-����ָ��
* Output       : ��
* Others       : �����ﲹ��ʱ��ʱ,����Ƿ���δ�յ�������,���в���,�ڳ����趨�Ĳ����ִλ��߹����
*                ��,�����Զ�ֹͣ
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
            // �ж��Ƿ�Ϊ���,�������Թ�
            if (0 == memcmp(SubNodes[nodeId].LongAddr, NullAddress, LONG_ADDR_SIZE)) {
                continue;
            }
            // �ж��Ƿ��Ǽ�����,�������ų�
            if ((SubNodes[nodeId].DevType & 0xF0) == 0xF0) {
                continue;
            }

			// ��ȡ��ǰʱ��ͽڵ��¼ʱ�䣬���߲�ֵ��ҪС��23Сʱ
            MeterDataTemp = Byte4ToUint32(&SubNodes[nodeId].RxMeterDataTemp[0]);
			RtcTemp = RTC_GetCounter();

            // �����Ƿ��Ѿ�����
            if ((SubNodes[nodeId].RxMeterDataDay == rtcTimer.Day) ||
                (RtcTemp - MeterDataTemp <= 23*60*60)) {
#if PRINT_INFO
                Gprs_OutputDebugMsg(0, "\n �ѳ���: ");
                OSTimeDlyHMSM(0, 0, 0, 500);
                DebugOutputLength(SubNodes[nodeId].LongAddr, LONG_ADDR_SIZE);
                OSTimeDlyHMSM(0, 0, 0, 500);
#endif
                continue;
            }
            // �����������ݰ�
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

            // ����������ӻص��ı�ʶ
            taskPtr->Command = txDataFrmPtr->Command;
            taskPtr->NodeId = nodeId;
            taskPtr->PkgSn = txDataFrmPtr->PkgSn;
            taskPtr->RouteLevel = txDataFrmPtr->RouteInfo.Level;

            // �����������ݰ�,ִ����Ϻ�,�û����������ͷŻ��ɱ��ָ���ȡ
            DataHandle_SetPkgPath(txDataFrmPtr, UNREVERSED);
            DataHandle_CreateTxData(txDataFrmPtr, version);

            // �ȴ���������Ӧ��
            rxDataFrmPtr = OSMboxPend(taskPtr->Mbox, TIME_DELAY_MS((taskPtr->RouteLevel - 1) * DELAYTIME_ONE_LAYER * 2), &err);
            if ((void *)0 != rxDataFrmPtr) {
#if PRINT_INFO
                Gprs_OutputDebugMsg(0, "\n Ӧ��: ");
                OSTimeDlyHMSM(0, 0, 0, 500);
                DebugOutputLength(SubNodes[nodeId].LongAddr, LONG_ADDR_SIZE);
                OSTimeDlyHMSM(0, 0, 0, 500);
#endif
                DataHandle_MeterDataSaveProc(rxDataFrmPtr);
                OSMemPut(LargeMemoryPtr, rxDataFrmPtr);
                OSTimeDlyHMSM(0, 0, 0, 300);
            } else {
#if PRINT_INFO
                Gprs_OutputDebugMsg(0, "\n ��Ӧ��: ");
                OSTimeDlyHMSM(0, 0, 0, 500);
                DebugOutputLength(SubNodes[nodeId].LongAddr, LONG_ADDR_SIZE);
                OSTimeDlyHMSM(0, 0, 0, 500);
#endif
                SubNodes[nodeId].Property.LastResult = 0;
                SubNodes[nodeId].Property.CurRouteNo = (SubNodes[nodeId].Property.CurRouteNo + 1) % MAX_CUSTOM_ROUTES;
            }

            // �Ƿ��������,�ڲ��������п�����ʱ��ֹ
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

    // ���ٱ�����,�˴������Ƚ�ֹ�������,�����޷��ͷű�����ռ�õ��ڴ�ռ�
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
* Decription   : ���ݲ���������
* Input        : ��
* Output       : ��
* Others       : �����ж��Ƿ��������ݲ�������
************************************************************************************************/
void DataHandle_DataReplenishProc(void)
{
    uint8 lastDay, err;
    uint32 dataReplenishDay, dayMask;
    RTC_TIME rtcTimer;
    DATA_HANDLE_TASK *taskPtr;

    DataReplenishTimer = DATAREPLENISH_INTERVAL_TIME;
    // ��δ�򿪲������ܵ�����²�����
    if (0 == Concentrator.Param.DataReplenishCtrl ||
        TRUE == TaskRunStatus.DataReplenish || TRUE == TaskRunStatus.DataForward) {
        return;
    }

    // ����Ƿ������ݲ�����ʱ���
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

    // ����δ��ռ�õĿռ�,��������
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
* Decription   : �����ϴ���������
* Input        : *p_arg-����ָ��
* Output       : ��
* Others       : ����Ҫ�ϴ������ݹ�һ��ʱ,����ʵʱ�ϴ�ʱ,��һ������ʱ,���������ϴ�����������
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

    // ����δ�ϱ��Ľڵ������
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

        // ���û�������ϴ�������
        if (0 == highPrioDataCount && (0 == rxMeterDataCount || 0 == Concentrator.Param.DataUploadCtrl)) {
            break;
        }

        // ���ݹ��������жϸ�������ֵ
        // ��������ϱ��������ݣ��ϴ���������������
        // ��������ϱ���ʱ�������ݣ��ϴ���������ʱ��������
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


            // ����Ҫ�ϴ������ݵ���û�дﵽһ��(������ʱ�䵽23��֮���յ�������������ϴ�)
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
        // ��������ϱ��������ݣ��ϴ���������ʱ��������
        else if(FreezeDataMode == Concentrator.Param.WorkType && 0x0 == Concentrator.Param.DataUploadMode) {
            uploadMaxCountOnePkg = (GPRS_DATA_MAX_DATA - DATA_FIXED_AREA_LENGTH_CRC8) / (LONG_ADDR_SIZE + REALTIME_DATA_AREA_SIZE);
            FreezeLen = FREEZE_DATA_AREA_SIZE;// �������ݳ���
            dataLen = REALTIME_DATA_AREA_SIZE;// ��ʱ�������ݳ���
            cmd = Upload_RealTime_Data;

            //��Ϊ���ݱ���Ϊ��������
            //��eeprom�ж�ȡ���������ݺ���Ҫת�ɶ�ʱ���������ϴε�������
            meterDataLen = sizeof(METER_DATA_SAVE_FORMAT) - 1 + FreezeLen;

            // ����Ҫ�ϴ������ݵ���û�дﵽһ��(������ʱ�䵽23��֮���յ�������������ϴ�)
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
                Gprs_OutputDebugMsg(0, "\n �ϴ����: ");
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

                        memcpy(&txDataFrmPtr->DataBuf[txDataFrmPtr->DataLen + RealTimeLen], &meterBufPtr->MeterData[5], 6); // ��ת����[6]
                        RealTimeLen += 6;
                        memset(&txDataFrmPtr->DataBuf[txDataFrmPtr->DataLen + RealTimeLen], 0, 6); 						// ��ת����[6]
                        RealTimeLen += 6;
                        memcpy(&txDataFrmPtr->DataBuf[txDataFrmPtr->DataLen + RealTimeLen], &meterBufPtr->MeterData[1+104], 8+2);//��������[10]
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
            Gprs_OutputDebugMsg(0,"\nERROR : ��˶�ʱ���������޷�ת��Ϊ���������ϴ�!!\n");
            break;
        }

        if (0 == count) {
            OSMemPut(LargeMemoryPtr, txDataFrmPtr);
            break;
        }
#if PRINT_INFO
        Gprs_OutputDebugMsg(0,"\n----�����ϴ���������----\n");
        OSTimeDlyHMSM(0, 0, 0, 500);
#endif
        txDataFrmPtr->DataBuf[0] = count;
        taskPtr->Command = txDataFrmPtr->Command;
        taskPtr->NodeId = NULL_U16_ID;
        taskPtr->PkgSn = txDataFrmPtr->PkgSn;

        // �����������ݰ�
        DataHandle_SetPkgPath(txDataFrmPtr, UNREVERSED);
        DataHandle_CreateTxData(txDataFrmPtr, version);

        // �ȴ���������Ӧ��
        rxDataFrmPtr = OSMboxPend(taskPtr->Mbox, GPRS_WAIT_ACK_OVERTIME, &err);
        if ((void *)0 != rxDataFrmPtr) {
            // �����Ǹ��ڵ㲢����״̬������
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

    // ���ٱ�����,�˴������Ƚ�ֹ�������,�����޷��ͷű�����ռ�õ��ڴ�ռ�
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
* Decription   : �����ϴ�������������
* Input        : ��
* Output       : ��
* Others       : �ж��Ƿ���������Ҫ�ϴ��������ϴ�����
************************************************************************************************/
void DataHandle_DataUploadProc(void)
{
    uint8 err;
    DATA_HANDLE_TASK *taskPtr;

    DataUploadTimer = DATAUPLOAD_INTERVAL_TIME;

    // Gprs��������,�����ϴ�����û������
    if (FALSE == Gprs.Online || TRUE == TaskRunStatus.DataUpload || TRUE == TaskRunStatus.DataForward) {
        return;
    }
    // ����δ��ռ�õĿռ�,���������ϴ�����
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
* Decription   : ����ת����������
* Input        : p_arg-����ԭ�����ݵ�ָ��
* Output       : ��
* Others       : �ú����ô���PC,����,�ֳֻ��������ݺ�ĵȴ�����
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
            // ����msg����Ӧ������
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

    // ����Ӧ�����ݰ�
    txDataFrmPtr->PkgProp = DataHandle_SetPkgProperty(txDataFrmPtr->PkgProp.PkgXor, NONE_ACK, ACK_PKG, UP_DIR);
    DataHandle_SetPkgPath(txDataFrmPtr, REVERSED);
    DataHandle_CreateTxData(txDataFrmPtr, version);

    // ���ٱ�����,�˴������Ƚ�ֹ�������,�����޷��ͷű�����ռ�õ��ڴ�ռ�
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
* Decription   : ����ת����������,ֻ��������¼�
* Input        : DataFrmPtr-���յ�������ָ��
* Output       : �Ƿ���Ҫ��������
* Others       : ����:������(1)+Ŀ��ڵ��ַ(6)+ת��������(N) + �ŵ�ѡ��(1)
*                ����:������(1)+Ŀ��ڵ��ַ(6)+ת���Ľ��(1)+ת����Ӧ������(N)
************************************************************************************************/
bool DataHandle_DataForwardProc(DATA_FRAME_STRUCT *DataFrmPtr)
{
    uint8 i, err;
    uint16 nodeId;
    DATA_HANDLE_TASK *taskPtr;
    DATA_FRAME_STRUCT *rxDataFrmPtr;
	uint8 version = 0;

    // ���ڵ��Ƿ�Ϸ�
    nodeId = Data_FindNodeId(0, &DataFrmPtr->DataBuf[1]);

    // �ڵ㲻���ڼ����������л��������ڴ�����
    if (TRUE == TaskRunStatus.DataForward) {
        if (NEED_ACK == DataFrmPtr->PkgProp.NeedAck) {
            DataFrmPtr->DataBuf[1 + LONG_ADDR_SIZE] = NULL_U16_ID == nodeId ? OP_ObjectNotExist : OP_Failure;
            DataFrmPtr->DataLen = 2 + LONG_ADDR_SIZE;
            return NEED_ACK;
        }
        OSMemPut(LargeMemoryPtr, DataFrmPtr);
        return NONE_ACK;
    }

    // �����ʱ�����ݲ������������,��ȴ����ݲ�����������ٽ���ת������
    if (TRUE == TaskRunStatus.DataReplenish) {
        i = 60;
        TaskRunStatus.DataForward = TRUE;
        while (--i && TRUE == TaskRunStatus.DataReplenish) {
            OSTimeDlyHMSM(0, 0, 1, 0);
        }
        TaskRunStatus.DataForward = FALSE;
    }

    // ����ռ䱣�浱ǰ������
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

    // �ڵ��ڼ������������򴴽�ת��
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

    // �����ҪӦ��,�򴴽�Ӧ������
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
* Decription   : ��ȡ�ڶ�ɨ���ŵ���������
* Input        : p_arg-����ԭ�����ݵ�ָ��
* Output       : ��
* Others       : �ú����ô���PC,����,�ֳֻ��������ݺ�ĵȴ�����
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

    // ����Ӧ�����ݰ�
    txDataFrmPtr->PkgProp = DataHandle_SetPkgProperty(txDataFrmPtr->PkgProp.PkgXor, NONE_ACK, ACK_PKG, UP_DIR);
    DataHandle_SetPkgPath(txDataFrmPtr, REVERSED);
    DataHandle_CreateTxData(txDataFrmPtr, version);

    // ���ٱ�����,�˴������Ƚ�ֹ�������,�����޷��ͷű�����ռ�õ��ڴ�ռ�
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
* Decription   : ��ȡ�ڶ�ɨ���ŵ�
* Input        : DataFrmPtr-���յ�������ָ��
* Output       : �Ƿ���Ҫ��������
* Others       : ����:������(1)+Ŀ��ڵ��ַ(6)+ת��������(N)
*                ����:������(1)+Ŀ��ڵ��ַ(6)+ת���Ľ��(1)+ת����Ӧ������(N)
************************************************************************************************/
bool DataHandle_ReadSecondChannelProc(DATA_FRAME_STRUCT *DataFrmPtr)
{
    uint8 i, err;
    uint16 nodeId;
    DATA_HANDLE_TASK *taskPtr;
    DATA_FRAME_STRUCT *rxDataFrmPtr;
	uint8 version = 0;

    // ���ڵ��Ƿ�Ϸ�
    nodeId = Data_FindNodeId(0, &DataFrmPtr->DataBuf[1]);

    // �ڵ㲻���ڼ����������л��������ڴ�����
    if (TRUE == TaskRunStatus.DataForward) {
        if (NEED_ACK == DataFrmPtr->PkgProp.NeedAck) {
            DataFrmPtr->DataBuf[1 + LONG_ADDR_SIZE] = NULL_U16_ID == nodeId ? OP_ObjectNotExist : OP_Failure;
            DataFrmPtr->DataLen = 2 + LONG_ADDR_SIZE;
            return NEED_ACK;
        }
        OSMemPut(LargeMemoryPtr, DataFrmPtr);
        return NONE_ACK;
    }

    // �����ʱ�����ݲ������������,��ȴ����ݲ�����������ٽ���ת������
    if (TRUE == TaskRunStatus.DataReplenish) {
        i = 60;
        TaskRunStatus.DataForward = TRUE;
        while (--i && TRUE == TaskRunStatus.DataReplenish) {
            OSTimeDlyHMSM(0, 0, 1, 0);
        }
        TaskRunStatus.DataForward = FALSE;
    }

    // ����ռ䱣�浱ǰ������
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

    // �ڵ��ڼ������������򴴽�ת��
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

    // �����ҪӦ��,�򴴽�Ӧ������
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
	// �˴����õڶ��ŵ���2e28 ��ʱֻ֧�� crc8 У��
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
* Decription   : ���ݴ�������,ֻ������յ��������¼�
* Input        : DataFrmPtr-����֡��ָ��
* Output       : ��
* Others       : �ú����������Ա�˻��������PC�����ֳֻ����͹�����ָ�����ָ����Ӧ��
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
        // ��������ϱ���ʱ�����򶳽����� 0x01
        case Meter_ActiveReport_Data:
            postHandle = DataHandle_ActiveReportProc(DataFrmPtr);
            break;

        // ���������汾��Ϣ 0x40
        case Read_CONC_Version:
            // ����:��������
            // ����:����汾(2)+Ӳ���汾(2)+Э��汾(2)
            DataFrmPtr->DataLen = 0;
            DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = (uint8)(SW_VERSION >> 8);
            DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = (uint8)SW_VERSION;
            DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = (uint8)(HW_VERSION >> 8);
            DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = (uint8)HW_VERSION;
            DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = (uint8)(PT_VERSION >> 8);
            DataFrmPtr->DataBuf[DataFrmPtr->DataLen++] = (uint8)PT_VERSION;
            break;

        // ��������ID 0x41
        case Read_CONC_ID:
            // ����:��������
            // ����:������ID��BCD��(6)
            memcpy(DataFrmPtr->DataBuf, Concentrator.LongAddr, LONG_ADDR_SIZE);
            DataFrmPtr->DataLen = LONG_ADDR_SIZE;
            break;

        // д������ID 0x42
        case Write_CONC_ID:
            Data_SetConcentratorAddr(DataFrmPtr);
            break;

        // ��������ʱ�� 0x43
        case Read_CONC_RTC:
            // ����:��������
            // ����:������ʱ��(7)
            Rtc_Get((RTC_TIME *)DataFrmPtr->DataBuf, Format_Bcd);
            DataFrmPtr->DataLen = 7;
            break;

        // д������ʱ�� 0x44
        case Write_CONC_RTC:
            // ����:������ʱ��(7)
            // ����:����״̬(1)
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

        // ��Gprs���� 0x45
        case Read_GPRS_Param:
            Data_GprsParameter(DataFrmPtr);
            break;

        // дGprs���� 0x46
        case Write_GPRS_Param:
            Data_GprsParameter(DataFrmPtr);
            break;

        // ��Gprs�ź�ǿ�� 0x47
        case Read_GPRS_RSSI:
            // ����:��
            // ����:�ź�ǿ��
            DataFrmPtr->DataBuf[0] = Gprs_GetCSQ();
            DataFrmPtr->DataBuf[1] = Gprs.Online ? 0x01 : 0x00;
            DataFrmPtr->DataLen = 2;
            DataFrmPtr->DataLen += Gprs_GetIMSI(&DataFrmPtr->DataBuf[DataFrmPtr->DataLen]);
            DataFrmPtr->DataLen += Gprs_GetGMM(&DataFrmPtr->DataBuf[DataFrmPtr->DataLen]);
            break;

        // ��������ʼ�� 0x48
        case Initial_CONC_Cmd:
            // ����:�������
            // ����:�������+����״̬
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

        // ���������Ĺ������� 0x49
        case Read_CONC_Work_Param:
            Data_RdWrConcentratorParam(DataFrmPtr);
            break;

        // д�������Ĺ������� 0x4A
        case Write_CONC_Work_Param:
            Data_RdWrConcentratorParam(DataFrmPtr);
            break;

        // �������������� 0x4C
        case Restart_CONC_Cmd:
            // ����:��
            // ����:����״̬
            DataFrmPtr->DataBuf[0] = OP_Succeed;
            DataFrmPtr->DataLen = 1;
            DevResetTimer = 5000;
            break;

        // ����������ת��ָ�� 0x4D
        case Data_Forward_Cmd:
            postHandle = DataHandle_DataForwardProc(DataFrmPtr);
            break;

		// ���ü������ڶ�ɨ���ŵ� 0x4E
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

		// ���������ڶ�ɨ���ŵ� 0x4F
		case Read_Second_Channel_Cmd:
            postHandle = DataHandle_ReadSecondChannelProc(DataFrmPtr);
			break;

        // ��ߵ����������� 0x50
        case Read_Meter_Total_Number:
            Data_ReadNodesCount(DataFrmPtr);
            break;

        // ��ȡ��ߵ�����Ϣ 0x51
        case Read_Meters_Doc_Info:
            Data_ReadNodes(DataFrmPtr);
            break;

        // д���ߵ�����Ϣ 0x52
        case Write_Meters_Doc_Info:
            Data_WriteNodes(DataFrmPtr);
            break;

        // ɾ����ߵ�����Ϣ 0x53
        case Delete_Meters_Doc_Info:
            Data_DeleteNodes(DataFrmPtr);
            break;

        // �޸ı�ߵ�����Ϣ 0x54
        case Modify_Meter_Doc_Info:
            Data_ModifyNodes(DataFrmPtr);
            break;

        // ���Զ���·����Ϣ 0x55
        case Read_Custom_Route_Info:
            Data_ReadCustomRoute(DataFrmPtr);
            break;

        // д�Զ���·����Ϣ 0x56
        case Write_Custom_Route_Info:
            Data_WriteCustomRoute(DataFrmPtr);
            break;

        // �������Զ���·����Ϣ 0x57
        case Batch_Read_Custom_Routes_Info:
            Data_BatchReadCustomRoutes(DataFrmPtr);
            break;

        // ����д�Զ���·����Ϣ 0x58
        case Batch_Write_Custom_Routes_Info:
            Data_BatchWriteCustomRoutes(DataFrmPtr);
            break;

        // ����������ʱ��������ָ�� 0x63
        case Read_RealTime_Data:
            DataHandle_MeterDataLoadProc(DataFrmPtr);
            break;

        // ����������������ָ�� 0x64
        case Read_Freeze_Data:
            DataHandle_MeterDataLoadProc(DataFrmPtr);
            break;

        // ��������������ʱ��������ָ�� 0x65
        case Batch_Read_RealTime_Data:
            DataHandle_MeterDataBatchLoadProc(DataFrmPtr);
            break;

         // ��������������������ָ�� 0x66
        case Batch_Read_Freeze_Data:
            DataHandle_MeterDataBatchLoadProc(DataFrmPtr);
            break;


        // ���������·���������ָ�� 0x67
        case Read_CmdDown_Data:
            DataHandle_ReadMeterCmdLoadProc(DataFrmPtr);
            break;

        // �������������·���������ָ�� 0x68
        case Batch_Read_CmdDown_Data:
            DataHandle_ReadMeterCmdBatchLoadProc(DataFrmPtr);
            break;

        // ����д�������·���������ָ�� 0x69
        case Batch_Wtite_CmdDown_Data:
            Data_BatchWriteMeterCmdLoadProc(DataFrmPtr);
            break;


        // �������������� 0xF1
        case Software_Update_Cmd:
            Data_SwUpdate(DataFrmPtr);
            break;

        // Eeprom��� 0xF3
        case Eeprom_Check_Cmd:
            Data_EepromCheckProc(DataFrmPtr);
            break;

        // ����ָ�֧��
        default:
        //Gprs_OutputDebugMsg(TRUE, "--��ָ���ݲ�֧��--\n");
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
* Decription   : ���ݴ�������,ֻ������յ���Ӧ���¼�
* Input        : DataBufPtr-��������ָ��
* Output       : ��
* Others       : �ú����������Ա�˻��������PC�����ֳֻ����͹�����Ӧ��
************************************************************************************************/
void DataHandle_RxAckProc(DATA_FRAME_STRUCT *DataFrmPtr)
{
    uint8 i;
    uint16 nodeId;
    DATA_HANDLE_TASK *taskPtr;

    // ����Ӧ��ڵ��Ƿ��ڱ�������
    nodeId = Data_FindNodeId(0, DataFrmPtr->Route[0]);

    // �жϸ�Ӧ��֡Ӧ�ô��ݸ�˭
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
* Decription   : ͸����������
* Input        : DataFrmPtr-���յ�������ָ��
* Output       : TRUE-�Ѿ�����,FALSE-û�д���
* Others       : Ŀ���ַ�����Լ�ʱ,���ݵ���һ���ڵ�
************************************************************************************************/
bool DataHandle_PassProc(DATA_FRAME_STRUCT *DataFrmPtr)
{
    static uint8 lastInPort = End_Port;
	uint8 version = 0;
	uint16 nodeId = 0;
	// �����Ŀ��ڵ���������һ������
	if (DataFrmPtr->RouteInfo.CurPos == DataFrmPtr->RouteInfo.Level - 1) {
		return FALSE;
	}
	// ������������ǵ����ڶ��������豸����ѡ��ͨѶ�˿�,�����������RF�˿�
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
* Decription   : ���ݴ�������,ֻ��������¼�
* Input        : *p_arg-����ָ��
* Output       : ��
* Others       : ��
************************************************************************************************/
void DataHandle_Task(void *p_arg)
{
    uint8 i, err, *dat;
    OS_FLAGS eventFlag;
    DATA_FRAME_STRUCT *dataFrmPtr;
    EXTRACT_DATA_RESULT ret;

    // ��ʼ������
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

    // ���ݳ�ʼ��
    Data_Init();

    while (TRUE) {
        // ��ȡ�������¼�����
        eventFlag = OSFlagPend(GlobalEventFlag, (OS_FLAGS)DATAHANDLE_EVENT_FILTER, (OS_FLAG_WAIT_SET_ANY | OS_FLAG_CONSUME), TIME_DELAY_MS(5000), &err);

        // ������Щ����
        while (eventFlag != (OS_FLAGS)0) {
            dat = (void *)0;
            if (eventFlag & FLAG_USART_RF_RX) {
                // Rfģ���յ�������
                dat = OSMboxAccept(SerialPort.Port[Usart_Rf].MboxRx);
                eventFlag &= ~FLAG_USART_RF_RX;
            } else if (eventFlag & FLAG_GPRS_RX) {
                // Gprsģ���յ�������
                dat = OSMboxAccept(Gprs.MboxRx);
                eventFlag &= ~FLAG_GPRS_RX;
            } else if (eventFlag & FLAG_USB_RX) {
                // Usb�˿��յ�������
                dat = OSMboxAccept(SerialPort.Port[Usb_Port].MboxRx);
                eventFlag &= ~FLAG_USB_RX;
            } else if (eventFlag & FLAG_USART_DEBUG_RX) {
                // Debug�˿��յ�������
                dat = OSMboxAccept(SerialPort.Port[Usart_Debug].MboxRx);
                eventFlag &= ~FLAG_USART_DEBUG_RX;
            } else if (eventFlag & FLAG_UART_RS485_RX) {
                // 485�˿��յ�������
                dat = OSMboxAccept(SerialPort.Port[Uart_Rs485].MboxRx);
                eventFlag &= ~FLAG_UART_RS485_RX;
            } else if (eventFlag & FLAG_USART_IR_RX) {
                // Ir�˿��յ�������
                dat = OSMboxAccept(SerialPort.Port[Usart_Ir].MboxRx);
                eventFlag &= ~FLAG_USART_IR_RX;
            } else if (eventFlag & FLAG_DELAY_SAVE_TIMER) {
                // ������ʱ����
                eventFlag &= ~FLAG_DELAY_SAVE_TIMER;
                DataHandle_DataDelaySaveProc();
            } else if (eventFlag & FLAG_DATA_REPLENISH_TIMER) {
                // ���ݲ�������
                eventFlag &= ~FLAG_DATA_REPLENISH_TIMER;
                DataHandle_DataReplenishProc();
            } else if (eventFlag & FLAG_DATA_UPLOAD_TIMER) {
                // �����ϴ�����
                eventFlag &= ~FLAG_DATA_UPLOAD_TIMER;
                DataHandle_DataUploadProc();
            } else if (eventFlag & FLAG_RTC_TIMING_TIMER) {
                // ʱ������Уʱ����
                eventFlag &= ~FLAG_RTC_TIMING_TIMER;
                DataHandle_RTCTimingProc();
            } else if (eventFlag & FLAG_MODIFY_SCAN_CHANNEL) {
                // �޸ı�������ϱ��ŵ��ţ�֪ͨ 2E28 �޸�ɨ���ŵ�
                eventFlag &= ~FLAG_MODIFY_SCAN_CHANNEL;
                DataHandle_ModifyScanChannelProc();
            }
            if ((void *)0 == dat) {
                continue;
            }

            // ��ԭ��������ȡ����
            if (Ok_Data != (ret = DataHandle_ExtractData(dat))) {
                if (Error_DstAddress == ret) {
                    // ������Ǹ��Լ������ݿ��Լ��������ڵ㲢��������Ӧ���ھӱ���
                }
				//else{
                    OSMemPut(LargeMemoryPtr, dat);
                    continue;
                //}
            }

            dataFrmPtr = (DATA_FRAME_STRUCT *)dat;
            // ȷ�������Ϣ�ϴ���ͨ��
            if (Usart_Debug == dataFrmPtr->PortNo || Usb_Port == dataFrmPtr->PortNo) {
                MonitorPort = (PORT_NO)(dataFrmPtr->PortNo);
            }

            // ���Ŀ���ַ�����Լ���ת��
            if (TRUE == DataHandle_PassProc(dataFrmPtr)) {
                continue;
            }

            // �ֱ�������֡��Ӧ��ָ֡��
            if (CMD_PKG == dataFrmPtr->PkgProp.PkgType) {
                // ���������֡
                DataHandle_RxCmdProc(dataFrmPtr);
            } else {
                // �����Ӧ��֡
                DataHandle_RxAckProc(dataFrmPtr);
            }
        }

        OSTimeDlyHMSM(0, 0, 0, 50);
    }
}

/***************************************End of file*********************************************/


