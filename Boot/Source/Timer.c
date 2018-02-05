/************************************************************************************************
*                                   SRWF-6009-BOOT
*    (c) Copyright 2015, Software Department, Sunray Technology Co.Ltd
*                               All Rights Reserved
*
* FileName     : Timer.c
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
#include "Uart.h"
#include "Hal.h"


/************************************************************************************************
*                        Global Variable Declare Section
************************************************************************************************/
TIMEREVENT_STRUCT TimerEvent[MAX_TIMER_AMOUNT];
uint32 Timer1ms;


/************************************************************************************************
*                           Function Declare Section
************************************************************************************************/

/************************************************************************************************
* Function Name: Timer_Init
* Decription   : 定时器初始化函数
* Input        : None
* Output       : None
* Others       : Timer RA 定时器，定时时间为1ms
************************************************************************************************/
void Timer_Init(void)
{
    uint8 i;

    for (i = 0; i < MAX_TIMER_AMOUNT; i ++)
    {
        TimerEvent[i].Time = INACTIVE_TIMER;
    }
}

/************************************************************************************************
* Function Name: Timer_Handler
* Decription   : 定时器事件处理函数
* Input        : None
* Output       : None
* Others       : Timer RA 定时器，定时时间为1ms
************************************************************************************************/
void Timer_Handler(void)
{
    uint8 i;

    for (i = 0; i < MAX_TIMER_AMOUNT; i++)
    {
        // 如果定时的时间到则运行对应的事件处理函数
        if (0 == TimerEvent[i].Time)
        {
            TimerEvent[i].Time = INACTIVE_TIMER;
            (*TimerEvent[i].Event)();
        }
        FEED_WATCHDOG
    }
}

/************************************************************************************************
* Function Name: Timer_ActiveEvent
* Decription   : 定时器事件激活函数
* Input        : msTime - 定时的时间(ms), Event - 时间到时所执行的函数, Mode - 工作模式(保持计时或重新计时)
* Output       : 定时器事件的时间序号
* Others       : Timer RA 定时器，定时时间为1ms
************************************************************************************************/
uint8 Timer_ActiveEvent(uint32 msTime, void (*Event)(), TIMEREVENT_ACTIVEMODE_ENUM Mode)
{
    uint8 i;

    // 判断这个定时器事件是否已经激活,如果已经激活则根据Mode来判断是继续计时还是重新计时
    for (i = 0; i < MAX_TIMER_AMOUNT; i++)
    {
        if ((INACTIVE_TIMER != TimerEvent[i].Time) && (Event == TimerEvent[i].Event))
        {
            if (Timer_ActiveResetMode == Mode)
            {
                TimerEvent[i].Time = msTime;
            }
            return i;
        }
    }

    // 如果这个定时器事件没有被激活,则激活这个定时器事件
    for (i = 0; i < MAX_TIMER_AMOUNT; i++)
    {
        if (INACTIVE_TIMER == TimerEvent[i].Time)
        {
            TimerEvent[i].Time = msTime;
            TimerEvent[i].Event = Event;
            return i;
        }
    }

    return TIMER_ERROR;
}

/************************************************************************************************
* Function Name: Timer2_ISR
* Decription   : 定时器中断处理函数
* Input        : None
* Output       : None
* Others       : 本函数由定时器2中断处理函数调用
************************************************************************************************/
void Timer2_ISR(void)
{
    uint8 i;

    Timer1ms++;

    // 处理1ms的中断事件
    if (Uart1.RxTimer > 0)
    {
        Uart1.RxTimer--;
    }

    // 定时器事件时钟计时
    for (i = 0; i < MAX_TIMER_AMOUNT; i++)
    {
        if (INACTIVE_TIMER != TimerEvent[i].Time && TimerEvent[i].Time > 0)
        {
            TimerEvent[i].Time--;
        }
    }
}

/**************************************End of file**********************************************/


