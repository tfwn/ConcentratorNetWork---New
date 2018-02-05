/************************************************************************************************
*                                   SRWF-6009
*    (c) Copyright 2015, Software Department, Sunray Technology Co.Ltd
*                               All Rights Reserved
*
* FileName     : Bsp.c
* Description  :
* Version      :
* Function List:
*------------------------------Revision History--------------------------------------------------
* No.   Version     Date            Revised By      Item            Description
* 1     V1.0        07/14/2015      Zhangxp         SRWF-6009       Original Version
************************************************************************************************/

#define BSP_GLOBALS

/************************************************************************************************
*                             Include File Section
************************************************************************************************/
#include "Stm32f10x_conf.h"
#include "ucos_ii.h"
#include "Bsp.h"
#include "SerialPort.h"
#include "Timer.h"
#include "Gprs.h"
#include "Rtc.h"
#include "Led.h"
#include "Eeprom.h"
#include "Flash.h"
#include "DataHandle.h"
#include "Database.h"

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
* Function Name: SysTick_Init
* Decription   : 心跳定时器初始化
* Input        : 无
* Output       : 无
* Others       : 无
************************************************************************************************/
void SysTick_Init(void)
{
    RCC_ClocksTypeDef rcc_clocks;

    RCC_GetClocksFreq(&rcc_clocks);
    SysTick_Config(rcc_clocks.HCLK_Frequency / OS_TICKS_PER_SEC);
}

/************************************************************************************************
* Function Name: Rcc_Init
* Decription   : 时钟初始化函数
* Input        : 无
* Output       : 无
* Others       : 无
************************************************************************************************/
void Rcc_Init(void)
{
    /* RCC system reset(for debug purpose) */
    RCC_DeInit();

#if defined (STM32F10X_HD) || (defined STM32F10X_XL) || (defined STM32F10X_HD_VL)
#ifdef DATA_IN_ExtSRAM
    SystemInit_ExtMemCtl();
#endif /* DATA_IN_ExtSRAM */
#endif

    /* Configure the System clock frequency, HCLK, PCLK2 and PCLK1 prescalers */
    /* Configure the Flash Latency cycles and enable prefetch buffer */
    RCC_HSICmd(ENABLE);                                             // 使用内部8MHz晶振
    while (RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET) {
    }

    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
    FLASH_SetLatency(FLASH_Latency_2);

    RCC_HCLKConfig(RCC_SYSCLK_Div2);                                // 为SYSCLK二分频
    RCC_PCLK2Config(RCC_HCLK_Div1);                                 // PCLK2时钟不分频
    RCC_PCLK1Config(RCC_HCLK_Div1);                                 // PCLK1时钟不分频

    // 设置 PLL 时钟源及倍频系数
    RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_12);           // 48MHz

    // 使能或者失能 PLL,这个参数可以取:ENABLE或者DISABLE,如果PLL被用于系统时钟,那么它不能被失能
    RCC_PLLCmd(ENABLE);

    // 等待指定的 RCC 标志位设置成功 等待PLL初始化成功
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) {

    }

    // 设置系统时钟（SYSCLK） 设置PLL为系统时钟源
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    // 等待PLL成功用作于系统时钟的时钟源
    //  0x00：HSI 作为系统时钟
    //  0x04：HSE作为系统时钟
    //  0x08：PLL作为系统时钟
    while (RCC_GetSYSCLKSource() != 0x08) {

    }

#ifdef VECT_TAB_SRAM
    //     SCB->VTOR = SRAM_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM. */
#else
    //     SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal FLASH. */
#endif

    /* Enable GPIOA, GPIOB, GPIOC and AFIO clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD| RCC_APB2Periph_AFIO , ENABLE);

    /* Enable TIM2 Clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    /* Enable WWDG clock */
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG | RCC_APB1Periph_SPI2, ENABLE);

    /* Enable DMA clock */
//    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    // 设置USB时钟
    RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_Div1);  //USBclk=PLLclk/1=48Mhz
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
}

/************************************************************************************************
* Function Name: Gpio_Init
* Decription   : Gpio引脚初始化
* Input        : 无
* Output       : 无
* Others       : 无
************************************************************************************************/
void Gpio_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);

    // PG引脚初始化
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStruct);

    // D+引脚拉低(PA18),必须放在USB初始化之前,目的是让PC机重新枚举USB端口
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    GPIO_ResetBits(GPIOA, GPIO_Pin_12);

    // RF多余引脚设置为输入状态
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/************************************************************************************************
* Function Name: Nvic_Init
* Decription   : 中断向量初始化
* Input        : 无
* Output       : 无
* Others       : 无
************************************************************************************************/
static void Nvic_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Configure the NVIC Preemption Priority Bits */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

    /* Enable the TIM2 UP Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                   = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

     /* Enable the USB interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Enable the USB Wake-up interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USBWakeUp_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_Init(&NVIC_InitStructure);
}

/**
  \brief   System Reset
  \details Initiates a system reset request to reset the MCU.
 */
__STATIC_INLINE void __NVIC_SystemReset(void)
{
  __DSB();                                                          /* Ensure all outstanding memory accesses included
                                                                       buffered write are completed before reset */
  SCB->AIRCR  = (uint32_t)((0x5FAUL << SCB_AIRCR_VECTKEY_Pos)    |
                           (SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) |
                            SCB_AIRCR_SYSRESETREQ_Msk    );         /* Keep priority group unchanged */
  __DSB();                                                          /* Ensure completion of memory access */

  for(;;)                                                           /* wait until reset */
  {
    __NOP();
  }
}

/************************************************************************************************
* Function Name: SoftReset
* Decription   : 软件复位
* Input        : 无
* Output       : 无
* Others       : 无
************************************************************************************************/
void SoftReset(void)
{
    __NVIC_SystemReset();
    //NVIC_SystemReset();
}

/************************************************************************************************
* Function Name: Iwdg_Init
* Decription   : 独立看门狗初始化
* Input        : 无
* Output       : 无
* Others       : 无
************************************************************************************************/
static void Iwdg_Init(void)
{
    // 当进入调试模式时看门狗停止工作
    DBGMCU->CR |= DBGMCU_CR_DBG_IWDG_STOP;

    // 解除写保护
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

    // 设置分频系数,时间计算(大概):Tout=((4*2^prer)*rlr)/40 (ms) =((4*2^5)*1600)/40=5S
    IWDG_SetPrescaler(IWDG_Prescaler_128);
    IWDG_SetReload(1600);

    // 重装一次
    IWDG_ReloadCounter();

    // 启动看门狗
    IWDG_Enable();

    RCC_LSICmd(ENABLE);                                     //打开LSI,因为独立看门狗使用的是LSI，所以最好程序启动的时候，使时钟源稳定:
    while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY)==RESET);       //等待直到LSI稳定
}

/************************************************************************************************
* Function Name: BSP_Init
* Decription   : 板级支持包初始化函数
* Input        : 无
* Output       : 无
* Others       : 无
************************************************************************************************/
void BSP_Init(void)
{
    CriticalSecCntr = 0;
    DevResetTimer = MAIN_BOARD_RESTART_PERIOD;
    OS_ENTER_CRITICAL();
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, (FLASH_APPCODE_START_ADDR - NVIC_VectTab_FLASH));
    Rcc_Init();
    Gpio_Init();
    Timer2_Init();
    Rtc_Init();
    SerialPort_Init();
    Gprs_Init();
    Nvic_Init();
    Eeprom_Init();
    Led_Init();
    Iwdg_Init();
    OS_EXIT_CRITICAL();
}

/***************************************End of file*********************************************/

