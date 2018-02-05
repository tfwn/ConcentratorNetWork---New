/************************************************************************************************
*                                   SRWF-6009-BOOT
*    (c) Copyright 2015, Software Department, Sunray Technology Co.Ltd
*                               All Rights Reserved
*
* FileName     : Hal.h
* Description  :
* Version      :
* Function List:
*------------------------------Revision History--------------------------------------------------
* No.   Version     Date            Revised By      Item            Description
* 1     V1.0        10/08/2015      Zhangxp         SRWF-6009-BOOT  Original Version
************************************************************************************************/
#ifndef _HAL_H_
#define _HAL_H_

/************************************************************************************************
*                             Include File Section
************************************************************************************************/
#include "stm32f10x.h"

/************************************************************************************************
*                                   Enum Define Section
************************************************************************************************/
typedef enum
{
    GPIO_INPUT,
    GPIO_OUTPUT,
}GPIO_MODE_ENUM;

/************************************************************************************************
*                          Pubilc Macro Define Section
************************************************************************************************/
// 管脚输入输出功能宏定义( PortNum: A~G, BitNum: 0~15 )
//-------------------------------------------------------------------------------------------------
// 输出管脚，可置0或1
#define GPIO_OUTPUT_PIN(PortNum, BitNum)                    *((volatile uint8 *)((PortNum##_BASE + 12 - 0x40000000) * 32 + 0x42000000 + 4 * BitNum))

// 推挽输出
#define GPIO_MODE_OUTPUT(PortNum, BitNum)                   Hal_ConfigureGpioMode(PortNum, BitNum, GPIO_OUTPUT)

// 输入管脚，可读取0或1
#define GPIO_INPUT_PIN(PortNum, BitNum)                     * ((volatile uint8 *)((PortNum##_BASE + 8 - 0x40000000) * 32 + 0x42000000 + 4 * BitNum))

// 浮空输入
#define GPIO_MODE_INPUT(PortNum, BitNum)                    Hal_ConfigureGpioMode(PortNum, BitNum, GPIO_INPUT)


// 外部中断宏定义
//-------------------------------------------------------------------------------------------------
// Enable Rising external interrupt in line(IntNum: 0~15)
#define EXTI_MODE_RISING_EDGE(IntNum)                       do {EXTI->RTSR |= (uint32)((IntNum) << 1);     \
                                                                EXTI->FTSR &= ~(uint32)((IntNum) << 1);    \
                                                            } while(0)

// Enable Falling external interrupt in line(IntNum:0~15)
#define EXTI_MODE_FALLING_EDGE(IntNum)                      do {EXTI->RTSR &= ~(uint32)((IntNum) << 1);    \
                                                                EXTI->FTSR |= (uint32)((IntNum) << 1);     \
                                                            } while(0)

#define ENABLE_EXTI(IntNum)                                 EXTI->IMR |= (uint32)(1 << (IntNum))
#define DISABLE_EXTI(IntNum)                                EXTI->IMR &= ~(uint32)(1 << (IntNum))

#define ENABLE_INTERRUPT                                    __enable_interrupt()
#define DISABLE_INTERRUPT                                   __disable_interrupt()


// 定时器3中断控制定义
//-------------------------------------------------------------------------------------------------
#define TIMER3_INT_ENABLE                                   TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE)
#define TIMER3_INT_DISABLE                                  TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE)

#define TMR_ENTER_CRITICAL()                                TIMER3_INT_DISABLE
#define TMR_EXIT_CRITICAL()                                 TIMER3_INT_ENABLE


// 看门狗控制管脚定义
//-------------------------------------------------------------------------------------------------
#define WATCHDOG_PIN                                        GPIO_Pin_4  // PC4
#define WATCHDOG_MODE_OUTPUT                                GPIO_MODE_OUTPUT(GPIOC, 4)
#define WATCHDOG_OUTPUT_PIN                                 GPIO_OUTPUT_PIN(GPIOC, 4)
#define WATCHDOG_MODE_INPUT                                 GPIO_MODE_INPUT(GPIOC, 4)
#define FEED_WATCHDOG                                       {IWDG_ReloadCounter(); WATCHDOG_OUTPUT_PIN ^= 1;}
#define FEED_WATCHDOG_STOP                                  WATCHDOG_MODE_INPUT


/************************************************************************************************
*                            Function Declare Section
************************************************************************************************/
extern void Hal_InitMcu(void);
extern void Hal_ConfigureGpioMode(GPIO_TypeDef * GPIOx, uint8 BitNum, GPIO_MODE_ENUM GpioMode);


#endif

/**************************************End of file**********************************************/

