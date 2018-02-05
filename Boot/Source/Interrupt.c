/************************************************************************************************
*                                   SRWF-6009-BOOT
*    (c) Copyright 2015, Software Department, Sunray Technology Co.Ltd
*                               All Rights Reserved
*
* FileName     : Interrupt.c
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
#include "stm32f10x_gpio.h"
#include "Interrupt.h"
#include "Hal.h"
#include "Uart.h"
#include "Timer.h"


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
* Function Name: NMI_Handler
* Decription   : NMI指令中断处理函数
* Input        : None
* Output       : None
* Others       : None
************************************************************************************************/
void NMI_Handler(void)
{
}


/************************************************************************************************
* Function Name: HardFault_Handler
* Decription   : 硬件错误中断处理函数
* Input        : None
* Output       : None
* Others       : None
************************************************************************************************/
void HardFault_Handler(void)
{
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1)
    {
    }
}

/************************************************************************************************
* Function Name: MemManage_Handler
* Decription   : 存储管理中断处理函数
* Input        : None
* Output       : None
* Others       : None
************************************************************************************************/
void MemManage_Handler(void)
{
    /* Go to infinite loop when Memory Manage exception occurs */
    while (1)
    {
    }
}

/************************************************************************************************
* Function Name: BusFault_Handler
* Decription   : 总线错误中断处理函数
* Input        : None
* Output       : None
* Others       : None
************************************************************************************************/
void BusFault_Handler(void)
{
    /* Go to infinite loop when Bus Fault exception occurs */
    while (1)
    {
    }
}

/************************************************************************************************
* Function Name: UsageFault_Handler
* Decription   : 错误应用中断处理函数
* Input        : None
* Output       : None
* Others       : None
************************************************************************************************/
void UsageFault_Handler(void)
{
    /* Go to infinite loop when Usage Fault exception occurs */
    while (1)
    {
    }
}

/************************************************************************************************
* Function Name: SVC_Handler
* Decription   : 通过SWI指令的系统服务调用
* Input        : None
* Output       : None
* Others       : None
************************************************************************************************/
void SVC_Handler(void)
{
}

/************************************************************************************************
* Function Name: DebugMon_Handler
* Decription   : 调试监控器
* Input        : None
* Output       : None
* Others       : None
************************************************************************************************/
void DebugMon_Handler(void)
{
}

/************************************************************************************************
* Function Name: PendSV_Handler
* Decription   : 可挂起的系统服务
* Input        : None
* Output       : None
* Others       : None
************************************************************************************************/
void PendSV_Handler(void)
{
}

/************************************************************************************************
* Function Name: WWDG_IRQHandler
* Decription   : 窗口定时器中断
* Input        : None
* Output       : None
* Others       : None
************************************************************************************************/
void WWDG_IRQHandler(void)
{
#if ENABLE_WWDG
    // Update WWDG counter
    WWDG_SetCounter(0x7F);

    // Clear EWI flag
    WWDG_ClearFlag();
#endif
}

/************************************************************************************************
* Function Name: TIM2_IRQHandler
* Decription   : Time2中断处理函数
* Input        : None
* Output       : None
* Others       : 该函数中断周期为1ms
************************************************************************************************/
void TIM2_IRQHandler(void)
{
    ENABLE_INTERRUPT;
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    Timer2_ISR();
}

/************************************************************************************************
* Function Name: TIM3_IRQHandler
* Decription   : Time3中断处理函数
* Input        : None
* Output       : None
* Others       : 该函数中断周期为10ms
************************************************************************************************/
void TIM3_IRQHandler(void)
{
    // Clear the TIM3 Update pending bit
    ENABLE_INTERRUPT;
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
}

/************************************************************************************************
* Function Name: USART1_IRQHandler
* Decription   : 串口1中断处理函数
* Input        : None
* Output       : None
* Others       :
************************************************************************************************/
void USART1_IRQHandler(void)
{
    ENABLE_INTERRUPT;
    Uart_IntHandler();
}

/************************************************************************************************
* Function Name: EXTI2_IRQHandler
* Decription   : EXTI线2中断处理函数
* Input        : None
* Output       : None
* Others       :
************************************************************************************************/
void EXTI2_IRQHandler(void)
{
}

/**************************************End of file**********************************************/


