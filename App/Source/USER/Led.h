/************************************************************************************************
*                                   SRWF-6009
*    (c) Copyright 2015, Software Department, Sunray Technology Co.Ltd
*                               All Rights Reserved
*
* FileName     : Led.h
* Description  :
* Version      :
* Function List:
*------------------------------Revision History--------------------------------------------------
* No.   Version     Date            Revised By      Item            Description
* 1     V1.0        08/13/2015      Zhangxp         SRWF-6009       Original Version
************************************************************************************************/

#ifndef  LED_H
#define  LED_H


#ifdef   LED_GLOBALS
#define  LED_EXT
#else
#define  LED_EXT  extern
#endif

/************************************************************************************************
*                               Pubilc Macro Define Section
************************************************************************************************/
#define FlashOnTime(x)                  ((x) >> 4 & 0x0F)
#define FlashOffTime(x)                 ((x) & 0x0F)
#define FLASHDELAYTIME                  600

/************************************************************************************************
*                                   Enum Define Section
************************************************************************************************/
// Led灯枚举变量
typedef enum {
    Led1 = 0,
    Led2,
    TotalLed
} LED_TYPE;

// Led闪烁的时间定义
typedef enum
{
    AlwaysOff = 0,              // 如果AlwayOff被选中，则等常灭，它的优先级最高
    Delay50ms,
    Delay100ms,
    Delay200ms,
    Delay300ms,
    Delay500ms,
    Delay800ms,
    Delay1000ms,
    Delay1200ms,
    Delay1500ms,
    Delay1800ms,
    Delay2000ms,
    Delay2500ms,
    Delay3000ms,
    Delay5000ms,
    AlwaysOn,
    EndDelayTime
} FLASH_TIME;

/************************************************************************************************
*                                  Struct Define Section
************************************************************************************************/
// Led控制参数变量定义
typedef struct {
    uint8 RunTime: 7;                               // Led运行计数器
    uint8 Status: 1;                                // Led运行状态
    uint8 FlashCtrl;                                // Led闪烁控制
    uint8 CurLedStatus;                             // Led当前闪烁状态
    uint8 DelayTimer;                               // Led延时定时器
} LED_CONTROL;

// Led参数结构体变量定义
typedef struct {
    LED_CONTROL Control[TotalLed];                  // Led控制器
    OS_STK Task_Stk[TASK_LED_STK_SIZE];             // Led任务堆栈
} LED_PARAM;

/************************************************************************************************
*                        Global Variable Declare Section
************************************************************************************************/
LED_EXT LED_PARAM Led;


/************************************************************************************************
*                            Function Declare Section
************************************************************************************************/
LED_EXT void Led_Init(void);
LED_EXT void Led_Task(void *p_arg);
LED_EXT void Led_FlashTime(LED_TYPE LedType, FLASH_TIME OnTime, FLASH_TIME OffTime, bool Effect);


#endif
/***************************************End of file*********************************************/


