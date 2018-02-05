/************************************************************************************************
*                                   SRWF-6009
*    (c) Copyright 2015, Software Department, Sunray Technology Co.Ltd
*                               All Rights Reserved
*
* FileName     : Common.c
* Description  :
* Version      :
* Function List:
*------------------------------Revision History--------------------------------------------------
* No.   Version     Date            Revised By      Item            Description
* 1     V1.1        07/16/2015      Zhangxp         SRWF-6009       Original Version
************************************************************************************************/

#define COMMON_GLOBALS

/************************************************************************************************
*                             Include File Section
************************************************************************************************/
#include "ucos_ii.h"
#include "Main.h"
#include "SerialPort.h"
#include "DataHandle.h"
#include <string.h>

/************************************************************************************************
*                        Global Variable Declare Section
************************************************************************************************/
uint8 BroadcastAddrIn[LONG_ADDR_SIZE] = {0xD5, 0xD5, 0xD5, 0xD5, 0xD5, 0xD5};
uint8 BroadcastAddrOut[LONG_ADDR_SIZE] = {0xD4, 0xD4, 0xD4, 0xD4, 0xD4, 0xD4};
uint8 NullAddress[LONG_ADDR_SIZE] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

/************************************************************************************************
*                           Prototype Declare Section
************************************************************************************************/

/************************************************************************************************
*                           Function Declare Section
************************************************************************************************/

/************************************************************************************************
* Function Name: Uint16ToString
* Decription   : 将16位长度的数转换成字符串,如0x1234转换成"4660"
* Input        : Src-16位长度的数,DstPtr-指向转换后的缓冲区指针
* Output       : 无
* Others       : 无
************************************************************************************************/
void Uint16ToString(uint16 Src, uint8 *DstPtr)
{
    uint8 i, result, flag;
    uint16 div = 10000;

    flag = 0;
    for (i = 0; i < 4; i++) {
        result = (uint8)(Src / div);
        if (result > 0 || 1 == flag) {
            *DstPtr++ = result + '0';
            flag = 1;
        }
        Src %= div;
        div /= 10;
    }
    *DstPtr++ = Src + '0';
    *DstPtr = 0;
}

/************************************************************************************************
* Function Name: Uint8ToString
* Decription   : 将8位长度的数转换成字符串,如0x34转换成"52"
* Input        : Src-8位长度的数,DstPtr-指向转换后的缓冲区指针
* Output       : 无
* Others       : 无
************************************************************************************************/
void Uint8ToString(uint8 Src, uint8 *DstPtr)
{
    uint8 i, result, flag;
    uint16 div = 100;

    flag = 0;
    for (i = 0; i < 2; i++) {
        result = Src / div;
        if (result > 0 || 1 == flag) {
            *DstPtr++ = result + '0';
            flag = 1;
        }
        Src %= div;
        div /= 10;
    }
    *DstPtr++ = Src + '0';
    *DstPtr = 0;
}

/************************************************************************************************
* Function Name: StringToByte
* Decription   : 将字符串转换成数字,如"1234"转换成0x1234
* Input        : SrcPtr-指向待转换字符串的指针,DstPtr-指向转换后的缓冲区指针,SrcLen-待转换数据长度
* Output       : 无
* Others       : 无
************************************************************************************************/
void StringToByte(uint8 *SrcPtr, uint8 *DstPtr, uint8 SrcLen)
{
    uint8 a, b;

    SrcLen /= 2;
    while (SrcLen--) {
        a = *SrcPtr++ - '0';
        b = *SrcPtr++ - '0';
        *DstPtr++ = a << 4 & 0xF0 | b;
    }
}

/************************************************************************************************
* Function Name: BinToBcd
* Decription   : 将Bin转换成BCD编码
* Input        : Val-待转换字节
* Output       : 转换后BCD格式的值
* Others       : 无
************************************************************************************************/
uint8 BinToBcd(uint8 Val)
{
    uint8 bcd = Val % 10;

    bcd = bcd | Val / 10 << 4;
    return bcd;
}

/************************************************************************************************
* Function Name: BcdToBin
* Decription   : BCD格式转换成Bin
* Input        : Val-待转换字节
* Output       : 转换后的值
* Others       : 无
************************************************************************************************/
uint8 BcdToBin(uint8 Val)
{
    uint8 hex;

    hex = (Val >> 4 & 0x0F) * 10 + (Val & 0x0F);
    return hex;
}

/************************************************************************************************
* Function Name: BcdToAscii
* Decription   : BCD格式转换成字节
* Input        : SrcPtr-待转换数据的指针,DstPtr-保存转换后的数据指针,SrcLength-数据长度
* Output       : 转换后的长度,为转换之前长度的 LenMul 倍
* Others       : 无
************************************************************************************************/
uint16 BcdToAscii(uint8 *SrcPtr, uint8 *DstPtr, uint8 SrcLength, uint8 LenMul)
{
    uint8 i, msb, lsb;

    for (i = 0; i < SrcLength; i++) {
        msb = *SrcPtr >> 4 & 0x0F;
        lsb = *SrcPtr & 0x0F;
        *DstPtr++ = (msb <= 9) ? (msb + '0') : (msb + 'A' - 10);
        *DstPtr++ = (lsb <= 9) ? (lsb + '0') : (lsb + 'A' - 10);
		if(LenMul == 3){
			*DstPtr++ = ' ';
		}
        SrcPtr++;
    }
    return (SrcLength * LenMul);
}

/************************************************************************************************
* Function Name: CalCrc8
* Decription   : Crc8校验计算函数
* Input        : DataBuf-指向数据的指针,DataLen-要校验的数据长度
* Output       : 返回校验值
* Others       : 无
************************************************************************************************/
uint8 CalCrc8(const uint8 *DataBuf, uint16 DataLen)
{
    uint8 i, crc8=0;

    while (DataLen--) {
        crc8 ^= *DataBuf;
        for (i = 0; i < 8; i++) {
            if (crc8 & 0x01) {
                crc8 >>= 1;
                crc8 ^= 0x8C;
            } else {
                crc8 >>= 1;
            }
        }
        DataBuf++;
    }
    return crc8;
}

/************************************************************************************************
* Function Name: CalCrc16
* Decription   : CalCrc16校验计算函数
* Input        : DataBuf-指向数据的指针,DataLen-要校验的数据长度
* Output       : 返回校验值
* Others       : 无
************************************************************************************************/
uint16 CalCrc16(const uint8 *DataBuf, uint32 DataLen)
{
    #define CRC_POLY 0x8408
    uint16 crc16 = 0xFFFF;
    uint8 i;

    while (DataLen--) {
        crc16 ^= *DataBuf++;
        for (i = 0; i < 8; i++) {
            if (crc16 & 0x0001) {
                crc16 >>= 1;
                crc16 ^= CRC_POLY;
            } else {
                crc16 >>= 1;
            }
        }
    }
    crc16 ^= 0xFFFF;

    return crc16;
}

/***************************************End of file*********************************************/

