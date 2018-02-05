/************************************************************************************************
*                                   SRWF-6009-BOOT
*    (c) Copyright 2015, Software Department, Sunray Technology Co.Ltd
*                               All Rights Reserved
*
* FileName     : Timer.h
* Description  :
* Version      :
* Function List:
*------------------------------Revision History--------------------------------------------------
* No.   Version     Date            Revised By      Item            Description
* 1     V1.0        10/08/2015      Zhangxp         SRWF-6009-BOOT  Original Version
************************************************************************************************/
#ifndef  _TIMER_H_
#define  _TIMER_H_

/************************************************************************************************
*                          Pubilc Macro Define Section
************************************************************************************************/
#define MAX_TIMER_AMOUNT                                    2
#define INACTIVE_TIMER                                      0xFFFFFFFF
#define TIMER_ERROR                                         0xFF

/************************************************************************************************
*                               Enum Define Section
************************************************************************************************/
typedef enum
{
    Timer_ActiveResetMode = 0,                                      // 定时器重新开始计时
    Timer_ActiveKeepMode                                            // 定时器保持原先的计时
}TIMEREVENT_ACTIVEMODE_ENUM;

/************************************************************************************************
*                                   Struct Define Section
************************************************************************************************/
// 定时器事件结构体定义
typedef struct
{
    uint32 Time;
    void (* Event)(void);
}TIMEREVENT_STRUCT;

/************************************************************************************************
*                            Variable Declare Section
************************************************************************************************/
extern uint32 Timer1ms;

/************************************************************************************************
*                            Function Declare Section
************************************************************************************************/
extern void Timer_Init(void);
extern void Timer_Handler(void);
extern uint8 Timer_ActiveEvent(uint32 msTime, void (* Event)(), TIMEREVENT_ACTIVEMODE_ENUM Mode);
extern void Timer2_ISR(void);


#endif

/**************************************End of file**********************************************/


