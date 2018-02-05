
/************************************************************************************************
*                                   SRWF-6009
*    (c) Copyright 2015, Software Department, Sunray Technology Co.Ltd
*                               All Rights Reserved
*
* FileName     : Main.c
* Description  :
* Version      :
* Function List:
*------------------------------Revision History--------------------------------------------------
* No.   Version     Date            Revised By      Item            Description
* 1     V1.0        07/14/2015      Zhangxp         SRWF-6009       Original Version
************************************************************************************************/

#define MAIN_GLOBALS

/************************************************************************************************
*                             Include File Section
************************************************************************************************/
#include "Stm32f10x_conf.h"
#include "Bsp.h"
#include "ucos_ii.h"
#include "Main.h"
#include "Rtc.h"
#include "SerialPort.h"
#include "Gprs.h"
#include "Led.h"
#include "Eeprom.h"
#include "Flash.h"
#include "DataHandle.h"
#include "Database.h"

/************************************************************************************************
*                        Global Variable Declare Section
************************************************************************************************/
__root const uint8 SWInfo[] = SOFTWARE_VERSION_INFO;


/************************************************************************************************
*                           Prototype Declare Section
************************************************************************************************/

/************************************************************************************************
*                           Function Declare Section
************************************************************************************************/

/************************************************************************************************
* Function Name: Main_Task
* Decription   : 主任务函数
* Input        : 无
* Output       : 无
* Others       : 无
************************************************************************************************/
void Main_Task(void *p_arg)
{
    uint8 i, err;

    (void)p_arg;
    OS_ENTER_CRITICAL();
    SysTick_Init();
    OS_EXIT_CRITICAL();
#if OS_TASK_STAT_EN > 0u
    OSStatInit();
#endif

    // 创建内存
    SmallMemoryPtr = OSMemCreate(MemorySmallBlock, TOTAL_SMALL_BLOCK, MEM_SMALL_BLOCK_LEN, &err);
    LargeMemoryPtr = OSMemCreate(MemoryLargeBlock, TOTAL_LARGE_BLOCK, MEM_LARGE_BLOCK_LEN, &err);

    // 创建信号量集
    GlobalEventFlag = OSFlagCreate(0, &err);

    // 创建邮箱
    for (i = Port_Start; i < Port_Total; i++) {
        SerialPort.Port[i].MboxTx = OSMboxCreate((void *)0);
        SerialPort.Port[i].MboxRx = OSMboxCreate((void *)0);
    }
    Gprs.MboxTx = OSMboxCreate((void *)0);
    Gprs.MboxRx = OSMboxCreate((void *)0);

    // 创建互斥信号量
    EepromAccessMutex = OSMutexCreate(TASK_EEPROM_PRIO, &err);

    // 创建串行数据管理任务
    OSTaskCreate(SerialPort_Task,
                 (void *)0,
                 &SerialPort.Task_Stk[TASK_SERIALPORT_STK_SIZE - 1],
                 TASK_SERIAL_PRIO);

    // 创建GPRS连接和数据管理任务
    OSTaskCreate(Gprs_Task,
                 (void *)0,
                 &Gprs.Task_Stk[TASK_GPRS_STK_SIZE - 1],
                 TASK_GPRS_PRIO);

    // 创建数据处理任务
    OSTaskCreate(DataHandle_Task,
                 (void *)0,
                 &Task_DataHandle_Stk[TASK_DATAHANDLE_STK_SIZE - 1],
                 TASK_DATAHANDLE_PRIO);

    // 创建LED控制管理任务
    OSTaskCreate(Led_Task,
                 (void *)0,
                 &Led.Task_Stk[TASK_LED_STK_SIZE - 1],
                 TASK_LED_PRIO);

    // 复位RF模块
    GPIO_ResetBits(GPIOB, GPIO_Pin_13);
    OSTimeDlyHMSM(0, 0, 0, 800);
    GPIO_SetBits(GPIOB, GPIO_Pin_13);

    while (TRUE) {
        // 延时保存
        if (SubNodesSaveDelayTimer) {
            if (0 == --SubNodesSaveDelayTimer) {
                OSFlagPost(GlobalEventFlag, (OS_FLAGS)FLAG_DELAY_SAVE_TIMER, OS_FLAG_SET, &err);
            }
        }

        // 数据补抄
        if (DataReplenishTimer) {
            if (0 == --DataReplenishTimer) {
                OSFlagPost(GlobalEventFlag, (OS_FLAGS)FLAG_DATA_REPLENISH_TIMER, OS_FLAG_SET, &err);
            }
        }

        // 数据上传
        if (DataUploadTimer) {
            if (0 == --DataUploadTimer) {
                OSFlagPost(GlobalEventFlag, (OS_FLAGS)FLAG_DATA_UPLOAD_TIMER, OS_FLAG_SET, &err);
            }
        }

        // 修改 2e28 扫描信道
        if (ModifyScanChannel) {
            if (0 == --ModifyScanChannel) {
                OSFlagPost(GlobalEventFlag, (OS_FLAGS)FLAG_MODIFY_SCAN_CHANNEL, OS_FLAG_SET, &err);
            }
        }

        // 时钟校时
        if (RTCTimingTimer) {
            if (0 == --RTCTimingTimer) {
                OSFlagPost(GlobalEventFlag, (OS_FLAGS)FLAG_RTC_TIMING_TIMER, OS_FLAG_SET, &err);
            }
        }

        // 心跳间隔
        if (Gprs.HeartbeatInterval) {
            if (0 == --Gprs.HeartbeatInterval) {
                OSFlagPost(GlobalEventFlag, (OS_FLAGS)FLAG_HEATBEAT_TIMER, OS_FLAG_SET, &err);
            }
        }

        // 喂狗,注意下面延时的时间一定要小于喂狗时间
        IWDG_ReloadCounter();
        OSTimeDlyHMSM(0, 0, 1, 0);
    }
}

/************************************************************************************************
* Function Name: main
* Decription   : 主函数
* Input        : 无
* Output       : 无
* Others       : 无
************************************************************************************************/
void main(void)
{
    BSP_Init();
    OSInit();
    OSTaskCreate(Main_Task,
                 (void *)0,
                 &MainTask_Stk[TASK_MAIN_STK_SIZE - 1],
                 TASK_MAIN_PRIO);
    OSStart();
    return;
}

/***************************************End of file*********************************************/

