/************************************************************************************************
*                                   SRWF-6009-BOOT
*    (c) Copyright 2015, Software Department, Sunray Technology Co.Ltd
*                               All Rights Reserved
*
* FileName     : Stm32f10x.c
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
#include "Hal.h"


/************************************************************************************************
*                        Global Variable Declare Section
************************************************************************************************/

/************************************************************************************************
*                           Prototype Declare Section
************************************************************************************************/
static void Hal_RccInit(void);
static void Hal_GpioInit(void);
static void Hal_NvicInit(void);
static void Hal_IwdgInit(void);
static void Hal_TimerInit(void);


/************************************************************************************************
*                           Function Declare Section
************************************************************************************************/

/************************************************************************************************
* Function Name: Hal_InitMcu
* Decription   : 单片机初始化函数
* Input        : None
* Output       : None
* Others       : None
************************************************************************************************/
void Hal_InitMcu(void)
{
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0000);
    Hal_RccInit();
    Hal_GpioInit();
    Hal_NvicInit();
    Hal_IwdgInit();
    Hal_TimerInit();
}

/************************************************************************************************
* Function Name: Hal_RccInit
* Decription   : 时钟初始化函数
* Input        : None
* Output       : None
* Others       : None
************************************************************************************************/
static void Hal_RccInit(void)
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

    /* Enable GPIOA, GPIOB, GPIOC and AFIO clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO , ENABLE);

    /* Enable TIM2 and TIM3 Clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 , ENABLE);

    /* Enable WWDG clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG | RCC_APB1Periph_SPI2, ENABLE);

    /* Enable DMA clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
}

/************************************************************************************************
* Function Name: Hal_GpioInit
* Decription   : GPIO初始化
* Input        : None
* Output       : None
* Others       : None
************************************************************************************************/
static void Hal_GpioInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // D+引脚拉低(PA18),必须放在USB初始化之前,目的是让PC机重新枚举USB端口
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOA, GPIO_Pin_12);
}

/************************************************************************************************
* Function Name: Hal_ConfigureGpioMode
* Decription   : 配置GPIO管脚工作模式
* Input        : None
* Output       : None
* Others       : None
************************************************************************************************/
void Hal_ConfigureGpioMode(GPIO_TypeDef *GPIOx, uint8 BitNum, GPIO_MODE_ENUM GpioMode)
{
    if (GPIO_INPUT == GpioMode)       //floating Input
    {
        if (BitNum > 7)
        {
            GPIOx->CRH =(GPIOx->CRH & (~(0x0000000F << 4 * (BitNum - 8))) | (0x00000004 << 4 * (BitNum - 8)));
        }
        else
        {
            GPIOx->CRL = (GPIOx->CRL & (~(0x0000000F << 4 * BitNum)) | (0x00000004 << 4 * BitNum));
        }
    }
    else if (GPIO_OUTPUT == GpioMode)  //Push-pull Output
    {
        if (BitNum > 7)
        {
            GPIOx->CRH = (GPIOx->CRH & (~(0x0000000F << 4 * (BitNum - 8))) | (0x00000003 << 4 * (BitNum - 8)));
        }
        else
        {
            GPIOx->CRL = (GPIOx->CRL & (~(0x0000000F << 4 * BitNum)) | (0x00000003 << 4 * BitNum));
        }
    }
}

/************************************************************************************************
* Function Name: Hal_NvicInit
* Decription   : 配置neste中断向量控制器
* Input        : None
* Output       : None
* Others       : None
************************************************************************************************/
static void Hal_NvicInit(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Configure the NVIC Preemption Priority Bits */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

    /* Enable the TIM2 UP Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                   = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/************************************************************************************************
* Function Name: Hal_IwdgInit
* Decription   : 配置独立看门狗
* Input        : None
* Output       : None
* Others       : None
************************************************************************************************/
static void Hal_IwdgInit(void)
{
    // 当进入调试模式时看门狗停止工作
    DBGMCU->CR |= DBGMCU_CR_DBG_IWDG_STOP;

    // 解除写保护
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

    // 设置分频系数,时间计算(大概):Tout=((4*2^prer)*rlr)/40 (ms) =((4*2^3)*2500)/40=2S
    // 一定要大于硬件开门狗的时间(约1.6S)
    IWDG_SetPrescaler(IWDG_Prescaler_32);
    IWDG_SetReload(2500);

    // 重装一次
    IWDG_ReloadCounter();

    // 启动看门狗
    IWDG_Enable();
}

/************************************************************************************************
* Function Name: Hal_TimerInit
* Decription   : 配置定时器
* Input        : None
* Output       : None
* Others       : None
************************************************************************************************/
static void Hal_TimerInit(void)
{
    uint32 tmp;
    RCC_ClocksTypeDef RCC_ClocksStatus;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    TIM_DeInit(TIM2);
    RCC_GetClocksFreq(&RCC_ClocksStatus);

    // Time Base configuration
    tmp = RCC_ClocksStatus.PCLK2_Frequency / 1000000;
    TIM_TimeBaseStructure.TIM_Prescaler         = (uint16)(tmp - 1);        // 1MHz
    TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period            = 999;                      // 1MS
    TIM_TimeBaseStructure.TIM_ClockDivision     = 0x0;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;

    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    // Enable the TIM1 Update Interrupt
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    // TIM1 counter enable
    TIM_Cmd(TIM2, ENABLE);
}

/**************************************End of file**********************************************/

