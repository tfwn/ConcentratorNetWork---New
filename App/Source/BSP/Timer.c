/************************************************************************************************
*                                   SRWF-6009
*    (c) Copyright 2015, Software Department, Sunray Technology Co.Ltd
*                               All Rights Reserved
*
* FileName     : Timer.c
* Description  :
* Version      :
* Function List:
*------------------------------Revision History--------------------------------------------------
* No.   Version     Date            Revised By      Item            Description
* 1     V1.0        07/20/2015      Zhangxp         SRWF-6009       Original Version
************************************************************************************************/

#define TIMER_GLOBALS

/************************************************************************************************
*                             Include File Section
************************************************************************************************/
#include "Stm32f10x_conf.h"
#include "ucos_ii.h"
#include "Main.h"
#include "Bsp.h"
#include "Flash.h"
#include "Timer.h"
#include "SerialPort.h"
#include "DataHandle.h"
#include "Gprs.h"

/************************************************************************************************
*                        Global Variable Declare Section
************************************************************************************************/

/************************************************************************************************
*                           Prototype Declare Section
************************************************************************************************/

/************************************************************************************************
*                           Function Declare Section
************************************************************************************************/

/************************************************************************************************
* Function Name: Timer2_Init
* Decription   : 定时器初始化
* Input        : 无
* Output       : 无
* Others       : 无
************************************************************************************************/
void Timer2_Init(void)
{
    uint32 tmp;
    RCC_ClocksTypeDef RCC_ClocksStatus;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    TIM_DeInit(TIM2);
    RCC_GetClocksFreq(&RCC_ClocksStatus);

    // Time Base configuration
    tmp = RCC_ClocksStatus.PCLK1_Frequency / 1000000;
    TIM_TimeBaseStructure.TIM_Prescaler         = (uint16)(tmp - 1);        // 1MHz
    TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period            = 999;                      // 1MS
    TIM_TimeBaseStructure.TIM_ClockDivision     = 0x0;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;

    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    // Enable the TIM2 Update Interrupt
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    // TIM1 counter enable
    TIM_Cmd(TIM2, ENABLE);
    Timer1ms = 0;
}

/************************************************************************************************
* Function Name: TIM2_IRQHandler
* Decription   : 定时器2中断服务程序
* Input        : 无
* Output       : 无
* Others       : 无
************************************************************************************************/
void TIM2_IRQHandler(void)
{
    uint8 err;

    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    OSIntEnter();
    Timer1ms++;
    Gprs.WorkTime++;
    SerialPort_DelayProc();

    // 软件延时复位
    if (0xFFFFFFFF != DevResetTimer) {
        if (2000 == DevResetTimer) {
            OSFlagPost(GlobalEventFlag, (OS_FLAGS)FLAG_LOGOFF_EVENT, OS_FLAG_SET, &err);
        }
        if (0 == DevResetTimer--) {
            SoftReset();
        }
    }
    OSIntExit();
}

/***************************************End of file*********************************************/

