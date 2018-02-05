/************************************************************************************************
*                                   SRWF-6009-BOOT
*    (c) Copyright 2015, Software Department, Sunray Technology Co.Ltd
*                               All Rights Reserved
*
* FileName     : Common.c
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
#include "Timer.h"
#include "Hal.h"
#include "stm32f10x.h"
#include "Core_cm3.h"


/************************************************************************************************
*                        Global Variable Declare Section
************************************************************************************************/


/************************************************************************************************
* Function Name: DataCopy
* Decription   : 数据拷贝函数
* Input        : Src - 源数据缓冲区指针, Dst - 目标数据缓冲区指针, Len - 拷贝数据的长度
* Output       : 实际拷贝数据的数量
* Others       : None
************************************************************************************************/
uint16 DataCopy(uint8 *Dst, uint8 *Src, uint16 Len)
{
    uint16 i;

    for (i = 0; i < Len; i++)
    {
        FEED_WATCHDOG
        * Dst++ = * Src++;
    }
    return i;
}

/************************************************************************************************
* Function Name: DataCompare
* Decription   : 数据比较函数
* Input        : Buf1 - 数据缓冲区1指针, Buf2 - 数据缓冲区2指针, Len - 比较数据的长度
* Output       : TRUE - 相同, FALSE - 不同
* Others       : None
************************************************************************************************/
bool DataCompare(uint8 *Buf1, uint8 *Buf2, uint16 Len)
{
    uint16 i;

    for (i = 0; i < Len; i++)
    {
        FEED_WATCHDOG
        if (*(Buf1 + i) != *(Buf2 + i))
        {
            return FALSE;
        }
    }
    return TRUE;
}

/************************************************************************************************
* Function Name: Cal_Crc
* Decription   : CRC校验函数
* Input        : Seed,被校验数据的Buf指针,被校验数据长度Len
* Output       : 计算出来的校验值
* Others       : None
************************************************************************************************/
uint16 Cal_Crc(uint16 Seed, uint8 *Buf, uint32 Len)
{
    uint16 crc = 0xFFFF;
    uint8 i;

    while (Len--)
    {
        FEED_WATCHDOG
        crc ^= * Buf++;
        for (i = 0; i < 8; i ++)
        {
            if (crc & 0x0001)
            {
                crc >>= 1;
                crc ^= Seed;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    crc ^= 0xFFFF;

    return crc;
}

/************************************************************************************************
* Function Name: SoftReset
* Decription   : 系统进行软件复位
* Input        : None
* Output       : None
* Others       :
************************************************************************************************/
void SoftReset(void)
{
    NVIC_SystemReset();
}

/**************************************End of file**********************************************/


