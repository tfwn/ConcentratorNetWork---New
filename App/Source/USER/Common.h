/************************************************************************************************
*                                   SRWF-6009
*    (c) Copyright 2015, Software Department, Sunray Technology Co.Ltd
*                               All Rights Reserved
*
* FileName     : Common.h
* Description  :
* Version      :
* Function List:
*------------------------------Revision History--------------------------------------------------
* No.   Version     Date            Revised By      Item            Description
* 1     V1.1        07/16/2015      Zhangxp         SRWF-6009       Original Version
************************************************************************************************/

#ifndef  COMMON_H
#define  COMMON_H


#ifdef   COMMON_GLOBALS
#define  COMMON_EXT
#else
#define  COMMON_EXT  extern
#endif

/************************************************************************************************
*                             Include File Section
************************************************************************************************/
#include "Version.h"

/**************************************************************************************************
*                          Pubilc Macro Define Section
**************************************************************************************************/
#define TRUE                                    1
#define FALSE                                   0

#define ON                                      1
#define OFF                                     0

#define HIGH                                    1
#define LOW                                     0

#define NULL                                    0

#define READ_ATTR                               0           // 读属性
#define WRITE_ATTR                              1           // 写属性
#define DELETE_ATTR                             2           // 删除属性

#define BIT(x)                                  (1 << (x))

#define BROADCAST_CRC16                         0xBECF
#define LONG_ADDR_SIZE                          6
#define SHORT_ADDR_SIZE                         2
#define PREAMBLE                                0x55
#define SYNCWORD1                               0xD3
#define SYNCWORD2                               0x91
#define TAILBYTE                                0x16
#define TAGGENWORD1								0xDA
#define TAGGENWORD2								0x26

/************************************************************************************************
*                                Typedef Section
************************************************************************************************/
typedef unsigned char       uint8;                          // 无符号8位整型变量
typedef signed char         int8;                           // 有符号8位整型变量
typedef unsigned short      uint16;                         // 无符号16位整型变量
typedef signed short        int16;                          // 有符号16位整型变量
typedef unsigned long       uint32;                         // 无符号32位整型变量
typedef signed long         int32;                          // 有符号32位整型变量
typedef float               fp32;                           // 单精度浮点数（32位长度）
typedef double              fp64;                           // 双精度浮点数（64位长度）
typedef unsigned char       bool;                           // 布尔类型变量


/************************************************************************************************
*                                  Struct Define Section
************************************************************************************************/
// 操作结果
typedef enum
{
    OP_Succeed = 0xAA,                                      // 操作成功
    OP_Failure = 0xAB,                                      // 操作失败
    OP_CommunicateError = 0xAC,                             // 通讯失败
    OP_CommunicateOK = 0xAD,                                // 通讯成功
    OP_FormatError = 0xAE,                                  // 格式错误
    OP_TimeAbnormal = 0xAF,                                 // 时间异常
    OP_ObjectNotExist = 0xBA,                               // 对象不存在
    OP_ObjectRepetitive = 0xBB,                             // 对象重复
    OP_Objectoverflow = 0xBC,                               // 对象已满
    OP_ParameterError = 0xBD,                               // 参数错误
    OP_OvertimeError = 0xCC,                                // 超时错误
    OP_SRunOvertimeError = 0xCD,                            // 单轮运行超时错误
    OP_Executing = 0xCE,                                    // 正在执行
    OP_HadDealed = 0xCF,                                    // 操作已处理
    OP_HadAck = 0xD0,                                       // 已应答
    OP_ErrorMeterData = 0xD1,                               // 抄表数据有错误
    OP_NoFunction = 0xD2                                    // 没有此项功能
} OP_RESULT;

/************************************************************************************************
*                                  Union Define Section
************************************************************************************************/

/************************************************************************************************
*                                  Struct Define Section
************************************************************************************************/

/************************************************************************************************
*                        Global Variable Declare Section
************************************************************************************************/
COMMON_EXT uint8 BroadcastAddrIn[LONG_ADDR_SIZE];
COMMON_EXT uint8 BroadcastAddrOut[LONG_ADDR_SIZE];
COMMON_EXT uint8 NullAddress[LONG_ADDR_SIZE];

/************************************************************************************************
*                            Function Declare Section
************************************************************************************************/
COMMON_EXT uint8 CalCrc8(const uint8 *DataBuf, uint16 DataLen);
COMMON_EXT uint16 CalCrc16(const uint8 *DataBuf, uint32 DataLen);
COMMON_EXT void Uint16ToString(uint16 Src, uint8 *DstPtr);
COMMON_EXT void Uint8ToString(uint8 Src, uint8 *DstPtr);
COMMON_EXT void StringToByte(uint8 *SrcPtr, uint8 *DstPtr, uint8 Len);
COMMON_EXT uint8 BinToBcd(uint8 Val);
COMMON_EXT uint8 BcdToBin(uint8 Val);
COMMON_EXT uint16 BcdToAscii(uint8 *SrcPtr, uint8 *DstPtr, uint8 SrcLength, uint8 LenMul);

#endif
/***************************************End of file*********************************************/

