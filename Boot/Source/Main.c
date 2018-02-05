/************************************************************************************************
*                                   SRWF-6009-BOOT
*    (c) Copyright 2015, Software Department, Sunray Technology Co.Ltd
*                               All Rights Reserved
*
* FileName     : Main.c
* Description  :
* Version      :
* Function List:
*------------------------------Revision History--------------------------------------------------
* No.   Version     Date        Revised By      Item            Description
* 1     V1.1        10/08/2015  Zhangxp         SRWF-6009-BOOT
************************************************************************************************/

/************************************************************************************************
*                             Include File Section
************************************************************************************************/
#include "Stm32f10x.h"
#include "System_stm32f10x.h"
#include "Main.h"
#include "Uart.h"
#include "Timer.h"
#include "Flash.h"
#include "Hal.h"
#include "Uart_Update.h"


/************************************************************************************************
*                        Global Variable Declare Section
************************************************************************************************/
uint8 Buffer[FLASH_PAGE_SIZE];
__root const uint8 *Boot_Key = "6009*Boot";


/************************************************************************************************
*                           Prototype Declare Section
************************************************************************************************/
static void App_UpdateProcess(void);


/************************************************************************************************
* Function Name: MCU_Init
* Decription   : 单片机初始化函数
* Input        : None
* Output       : None
* Others       : None
************************************************************************************************/
static void MCU_Init(void)
{
    DISABLE_INTERRUPT;
    Hal_InitMcu();
    Timer_Init();               // 必须在协议栈初始化之前调用
    Uart_Init();
    ENABLE_INTERRUPT;
}

/************************************************************************************************
* Function Name: main
* Decription   : 主函数
* Input        : None
* Output       : None
* Others       : None
************************************************************************************************/
void main(void)
{
    uint32 JumpAdr;
    pFunction JumpToApplication;

    // 系统初始化
    MCU_Init();

    // 进入UART升级进程
    Uart_UpdateProcess();

    // 进入APP升级进程
    App_UpdateProcess();

    // 跳转到APP应用程序
    JumpAdr = *(__IO uint32_t *)(FLASH_APPCODE_START_ADDR + 4);
    JumpToApplication = (pFunction)JumpAdr;
    __set_MSP(*(__IO uint32_t *)FLASH_APPCODE_START_ADDR);    // Initialize user application's Stack Pointer
    JumpToApplication();                                      // Jump to user application

    while(1);
}

/************************************************************************************************
* Function Name: App_UpdateProcess
* Decription   : 判断是否是APP升级
* Input        : None
* Output       : None
* Others       : None
************************************************************************************************/
static void App_UpdateProcess(void)
{
    uint8 i;
    uint16 crc, fcs, len;
    uint32 count, srcAdr, dstAdr, codeLen;

    // 升级信息保存格式: Crc16(2)+升级文件保存位置(4)+升级代码总长度(4)+Crc16(2)
    DataCopy(Buffer, (uint8 *)FLASH_UPGRADE_INFO_START, 12);
    if (((uint16 *)(&Buffer[10]))[0] != Cal_Crc(CRC_SEED, Buffer, 10))
    {
        return;
    }
    crc = ((uint16 *)Buffer)[0];
    srcAdr = ((uint32 *)(&Buffer[2]))[0];
    codeLen = ((uint32 *)(&Buffer[6]))[0];
    dstAdr = FLASH_APPCODE_START_ADDR;
    fcs = Cal_Crc(CRC_SEED, (uint8 *)srcAdr, codeLen);
    if (crc != fcs)
    {
        return;
    }

    // 开始升级
    count = 0;
    do
    {
        len = (codeLen - count > sizeof(Buffer)) ? sizeof(Buffer) : (codeLen - count);
        DataCopy(Buffer, (uint8 *)srcAdr, len);
        for (i = 0; i < 8; i++)
        {
            Flash_Erase(dstAdr, 1);
            Flash_Write(Buffer, len, dstAdr);
            if (TRUE == DataCompare((uint8 *)srcAdr, (uint8 *)dstAdr, len))
            {
                srcAdr += len;
                dstAdr += len;
                count += len;
                break;
            }
        }
    } while (count < codeLen && i < 8);

    // 擦除升级信息
    if (count >= codeLen)
    {
        Flash_Erase(FLASH_UPGRADE_INFO_START, 1);
    }
    while (1)
    {
        SoftReset();
    }
}

/**************************************End of file**********************************************/


