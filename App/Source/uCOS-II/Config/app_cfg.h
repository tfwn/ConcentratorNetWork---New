/************************************************************************************************
*                                   SRWF-6009
*    (c) Copyright 2015, Software Department, Sunray Technology Co.Ltd
*                               All Rights Reserved
*
* FileName     : App_cfg.h
* Description  :
* Version      :
* Function List:
*------------------------------Revision History--------------------------------------------------
* No.   Version     Date            Revised By      Item            Description
* 1     V1.0        07/15/2015      Zhangxp         SRWF-6009       Original Version
************************************************************************************************/

#ifndef  APP_CFG_H
#define  APP_CFG_H

/************************************************************************************************
*                               Pubilc Macro Define Section
************************************************************************************************/
// 共享内存空间参数定义
#define TOTAL_LARGE_BLOCK                       12          // 大内存块数
#define TOTAL_SMALL_BLOCK                       5           // 小内存块数


// 固定内存空间参数定义,该空间为某个任务专用,不释放
#define TASK_MAIN_STK_SIZE                      128
#define TASK_SERIALPORT_STK_SIZE                128
#define TASK_GPRS_STK_SIZE                      128
#define TASK_DATAHANDLE_STK_SIZE                168
#define TASK_LED_STK_SIZE                       88

// 延时时间定义,此处定义要和Os_cfg.h文件中的OS_TICKS_PER_SEC定义一起更改,最小值为10
#define TIME_DELAY_MS(x)                        ((x) * OS_TICKS_PER_SEC / 1000)

// 任务优先级别定义
#define TASK_MAIN_PRIO                          4           // 主任务
#define TASK_SERIAL_PRIO                        6           // 串行数据任务
#define TASK_EEPROM_PRIO                        7           // Eeprom访问互斥信号量
#define TASK_GPRS_PRIO                          8           // Gprs任务

#define TASK_DATAHANDLE_RF_PRIO                 9           // Rf串口数据处理任务
#define TASK_DATAHANDLE_GPRS_PRIO               10

#define TASK_DATAHANDLE_DATA_PRIO               12          // 抄表数据处理,以下5个优先级必须为空

#define TASK_DATAHANDLE_PRIO                    20          // 数据处理任务

#define TASK_LED_PRIO                           25          // Led任务

/************************************************************************************************
*                                   Enum Define Section
************************************************************************************************/

/************************************************************************************************
*                                  Struct Define Section
************************************************************************************************/

/************************************************************************************************
*                                 Variable Declare Section
************************************************************************************************/

/************************************************************************************************
*                            Function Declare Section
************************************************************************************************/

#endif

/***************************************End of file*********************************************/

