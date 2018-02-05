/************************************************************************************************
*                                   SRWF-6009
*    (c) Copyright 2015, Software Department, Sunray Technology Co.Ltd
*                               All Rights Reserved
*
* FileName     : Eeprom.c
* Description  : 根据Stm32_eval_i2c_ee.c改写,启用DMA
* Version      :
* Function List:
*------------------------------Revision History--------------------------------------------------
* No.   Version     Date            Revised By      Item            Description
* 1     V1.1        08/03/2015      Zhangxp         SRWF-6009       Original Version
************************************************************************************************/

#define EEPROM_GLOBALS

/************************************************************************************************
*                             Include File Section
************************************************************************************************/
#include "Stm32f10x_conf.h"
#include "ucos_ii.h"
#include "Eeprom.h"

/************************************************************************************************
*                        Global Variable Declare Section
************************************************************************************************/
static uint8 EepromDMADone;             // 0-进行中 1-完成 2-出现错误


/************************************************************************************************
*                           Prototype Declare Section
************************************************************************************************/

/************************************************************************************************
*                           Function Declare Section
************************************************************************************************/

/************************************************************************************************
* Function Name: DMA1_Channel6_IRQHandler
* Decription   : Eeprom写操作DMA中断处理函数
* Input        : 无
* Output       : 无
* Others       : 无
************************************************************************************************/
void DMA1_Channel6_IRQHandler(void)
{
    uint32 timeOut;

    OSIntEnter();

    // 检查DMA传输是否结束
    if (DMA_GetFlagStatus(DMA1_IT_TC6) != RESET) {

        // 禁能DMA发送通道并清除中断标识位
        DMA_Cmd(DMA1_Channel6, DISABLE);
        DMA_ClearFlag(DMA1_IT_GL6);
        EepromDMADone = 1;

        // 等待直到数据都被物理传送到总线上,若发生错误,在中断外处理
        timeOut = EEPROM_LONG_TIMEOUT;
        while(!I2C_GetFlagStatus(I2C1, I2C_FLAG_BTF)) {
            if ((timeOut--) == 0) {
                EepromDMADone = 2;
            }
        }

        // 发送停止条件
        I2C_GenerateSTOP(I2C1, ENABLE);

        // 运行SR1和SR2的读操作以便清除事件位
        (void)I2C1->SR1;
        (void)I2C1->SR2;
    }

    OSIntExit();
}

/************************************************************************************************
* Function Name: DMA1_Channel7_IRQHandler
* Decription   : Eeprom读操作DMA中断处理函数
* Input        : 无
* Output       : 无
* Others       : 无
************************************************************************************************/
void DMA1_Channel7_IRQHandler(void)
{
    OSIntEnter();

    // 检查DMA传输是否结束
    if (DMA_GetFlagStatus(DMA1_IT_TC7) != RESET) {

        // 发送结束条件
        I2C_GenerateSTOP(I2C1, ENABLE);

        // 禁能DMA并清除中断标志位
        DMA_Cmd(DMA1_Channel7, DISABLE);
        DMA_ClearFlag(DMA1_IT_GL7);

        EepromDMADone = 1;
    }

    OSIntExit();
}

/************************************************************************************************
* Function Name: Eeprom_Timeout_UserCallback
* Decription   : Eeprom操作超时后处理函数
* Input        : 无
* Output       : 无
* Others       : 无
************************************************************************************************/
static void Eeprom_Timeout_UserCallback(void)
{
    uint8 i, delay;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 复位从器件方法:SCL对应的端口为OD输出,再强制向SCL发送9个脉冲
    for (i = 0; i < 10; i++) {
        GPIO_SetBits(GPIOB, GPIO_Pin_6);
        for (delay = 0; delay < 100; delay++);
        GPIO_ResetBits(GPIOB, GPIO_Pin_6);
        for (delay = 0; delay < 100; delay++);
    }
    GPIO_SetBits(GPIOB, GPIO_Pin_6);                // 此句必须有,否则无效

    // 复位STM32的I2C总线(SCLK,SDA必须为高电平才有效)
    I2C_SoftwareResetCmd(I2C1, ENABLE);

    // 重新初始化I2C
    Eeprom_Init();
}

/************************************************************************************************
* Function Name: Eeprom_LowLevel_DMAConfig
* Decription   : Eeprom数据传输DMA低级配置
* Input        : BufPtr-保存数据的指针值,BufSize-保存数据的数量,AccessMode-访问期间的模式
* Output       : 无
* Others       : 无
************************************************************************************************/
static void Eeprom_LowLevel_DMAConfig(uint32 BufPtr, uint32 BufSize, uint8 AccessMode)
{
    DMA_InitTypeDef eprom_DMA_InitStructure;

    if (AccessMode == WRITE_ATTR) {
        // 配置DMA发送通道参数
        eprom_DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)I2C1_DR_Address;
        eprom_DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)BufPtr;
        eprom_DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
        eprom_DMA_InitStructure.DMA_BufferSize = (uint32_t)BufSize;
        eprom_DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        eprom_DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
        eprom_DMA_InitStructure.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte;
        eprom_DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        eprom_DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
        eprom_DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
        eprom_DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
        DMA_Init(DMA1_Channel6, &eprom_DMA_InitStructure);
    } else {
        // 配置DMA接收通道参数
        eprom_DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)I2C1_DR_Address;
        eprom_DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)BufPtr;
        eprom_DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
        eprom_DMA_InitStructure.DMA_BufferSize = (uint32_t)BufSize;
        eprom_DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        eprom_DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
        eprom_DMA_InitStructure.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte;
        eprom_DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        eprom_DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
        eprom_DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
        eprom_DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
        DMA_Init(DMA1_Channel7, &eprom_DMA_InitStructure);
    }
}

/************************************************************************************************
* Function Name: Eeprom_AccessProc
* Decription   : Eeprom读写访问函数
* Input        : DevAddr-Eeprom芯片的地址
*                SubAddr-Eeprom芯片的内部地址,即读写地址
*                BufPtr-保存读出数据的指针或写入数据的缓冲区指针
*                Count-读写数据的数量
*                AccessMode-Eeprom访问方式,读或者写
* Output       : 成功或者失败
* Others       : 采用DMA模式传输
************************************************************************************************/
static bool Eeprom_AccessProc(uint8 DevAddr, uint32 SubAddr, uint8 *BufPtr, uint16 Count, uint8 AccessMode)
{
    uint32 timeOut;

    // 检查I2C总线是否忙
    timeOut = EEPROM_LONG_TIMEOUT;
    while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)) {
        if ((timeOut--) == 0) {
            goto EEPROM_TIMEOUT_HANDLE;
        }
    }

    // 发送起始条件并检查是否正确执行
    I2C_GenerateSTART(I2C1, ENABLE);
    timeOut = EEPROM_FLAG_TIMEOUT;
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) {
        if ((timeOut--) == 0) {
            goto EEPROM_TIMEOUT_HANDLE;
        }
    }

    // 发送芯片地址(写模式)并检查是否正确执行
    if (SubAddr > 0xFFFF) {
        DevAddr |= 0x02;
    }
    I2C_Send7bitAddress(I2C1, DevAddr, I2C_Direction_Transmitter);
    timeOut = EEPROM_LONG_TIMEOUT;
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
        if ((timeOut--) == 0) {
            goto EEPROM_TIMEOUT_HANDLE;
        }
    }

    // 发送芯片内部从地址高字节并检查是否正确执行
    I2C_SendData(I2C1, (uint8)(SubAddr >> 8));
    timeOut = EEPROM_FLAG_TIMEOUT;
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
        if ((timeOut--) == 0) {
            goto EEPROM_TIMEOUT_HANDLE;
        }
    }

    // 发送芯片内部从地址低字节并检查是否正确执行
    I2C_SendData(I2C1, (uint8)SubAddr);
    timeOut = EEPROM_FLAG_TIMEOUT;
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
        if ((timeOut--) == 0) {
            goto EEPROM_TIMEOUT_HANDLE;
        }
    }

    if (WRITE_ATTR == AccessMode) {
        // 如果是写模式,则配置DMA发送通道,参数为缓冲区指针和缓冲区大小并使能DMA通道
        Eeprom_LowLevel_DMAConfig((uint32)BufPtr, Count, AccessMode);
        DMA_Cmd(DMA1_Channel6, ENABLE);
        EepromDMADone = 0;
    } else {
        // 如果是读模式,再次发送起始条件并检查是否正确执行
        I2C_GenerateSTART(I2C1, ENABLE);
        timeOut = EEPROM_FLAG_TIMEOUT;
        while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) {
            if ((timeOut--) == 0) {
                goto EEPROM_TIMEOUT_HANDLE;
            }
        }

        // 发送芯片地址(读模式)
        I2C_Send7bitAddress(I2C1, DevAddr, I2C_Direction_Receiver);

        // 如果读取的字节数少于2个字节则直接读取,不启用DMA
        if (Count < 2) {
            // 检查发送芯片地址操作是否成功
            timeOut = EEPROM_FLAG_TIMEOUT;
            while (I2C_GetFlagStatus(I2C1, I2C_FLAG_ADDR) == RESET) {
                if ((timeOut--) == 0) {
                    goto EEPROM_TIMEOUT_HANDLE;
                }
            }

            // 不发送应答位NACK
            I2C_AcknowledgeConfig(I2C1, DISABLE);

            // 关中断
            OS_ENTER_CRITICAL();

            // 通过读取SR1和SR2来清除ADDR寄存器(SR1已经读过了)
            (void)I2C1->SR2;

            // 发送停止条件
            I2C_GenerateSTOP(I2C1, ENABLE);

            // 开中断
            OS_EXIT_CRITICAL();

            // 等待收到的数据
            timeOut = EEPROM_FLAG_TIMEOUT;
            while (I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE) == RESET) {
                if ((timeOut--) == 0) {
                    goto EEPROM_TIMEOUT_HANDLE;
                }
            }

            // 读取从Eeprom中收到的数据
            *BufPtr = I2C_ReceiveData(I2C1);

            // 检查停止位是否执行完毕
            timeOut = EEPROM_FLAG_TIMEOUT;
            while (I2C1->CR1 & I2C_CR1_STOP) {
                if ((timeOut--) == 0) {
                    goto EEPROM_TIMEOUT_HANDLE;
                }
            }

            // 启动响应位以便下次接收数据
            I2C_AcknowledgeConfig(I2C1, ENABLE);

            EepromDMADone = 1;
        } else {
            // 检查发送地址是否执行完毕
            timeOut = EEPROM_FLAG_TIMEOUT;
            while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
                if ((timeOut--) == 0) {
                    goto EEPROM_TIMEOUT_HANDLE;
                }
            }

            // 配置DMA接收通道,参数为地址指针和读取的数量
            Eeprom_LowLevel_DMAConfig((uint32)BufPtr, Count, READ_ATTR);

            // 通知DMA下一个停止条件即是最后一个
            I2C_DMALastTransferCmd(I2C1, ENABLE);

            // 使能DMA通道
            DMA_Cmd(DMA1_Channel7, ENABLE);
        }
    }

    return SUCCESS;

EEPROM_TIMEOUT_HANDLE:              // Eeprom读写操作超时处理
    Eeprom_Timeout_UserCallback();
    EepromDMADone = 1;

    return ERROR;

}

/************************************************************************************************
* Function Name: Eeprom_Init
* Decription   : Eeprom初始化
* Input        : 无
* Output       : 无
* Others       : 采用DMA模式传输
************************************************************************************************/
void Eeprom_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    I2C_InitTypeDef I2C_InitStructure;

    // 禁能I2C1功能并复位,I2C1时钟禁能
    I2C_Cmd(I2C1, DISABLE);
    I2C_DeInit(I2C1);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, DISABLE);

    // DMA1通道6,7禁能
    DMA_Cmd(DMA1_Channel6, DISABLE);
    DMA_Cmd(DMA1_Channel7, DISABLE);

    // I2C1 IO引脚启动时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    // I2C1 GPIO引脚初始化
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 使能DMA1时钟并复位通道6,7
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    DMA_DeInit(DMA1_Channel6);
    DMA_DeInit(DMA1_Channel7);

    // 配置并使能I2C DMA TX通道中断
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // 配置并使能I2C DMA RX通道中断
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_Init(&NVIC_InitStructure);

    // 使能DMA1通道中断
    DMA_ITConfig(DMA1_Channel6, DMA_IT_TC, ENABLE);
    DMA_ITConfig(DMA1_Channel7, DMA_IT_TC, ENABLE);

    // I2C功能配置
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = EEPROM_DEVADDR;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED;

    // 使能I2C1并应用配置
    I2C_Cmd(I2C1, ENABLE);
    I2C_Init(I2C1, &I2C_InitStructure);

    // 使能I2C DMA功能
    I2C_DMACmd(I2C1, ENABLE);
}

/************************************************************************************************
* Function Name: Eeprom_ReadWrite
* Decription   : Eeprom读写数据
* Input        : BufPtr-保存读出或写入数据的指针;
*                SubAddr-芯片内部从地址;
*                Count-读出或写入的数量
*                AccessMode-访问芯片模式
* Output       : 成功或失败
* Others       : 采用DMA模式传输
************************************************************************************************/
bool Eeprom_ReadWrite(uint8 *BufPtr, uint32 SubAddr, uint16 Count, uint8 AccessMode)
{
    bool result, flag;
    uint8 err, addr, timeOut;
    uint16 num;

    if (Count == 0 ) {
        return ERROR;
    }

    // 等待获取Eeprom的访问权限,如果超时则复位Eeprom芯片和I2c总线
    OSMutexPend(EepromAccessMutex, TIME_DELAY_MS(10000), &err);
    if (OS_ERR_TIMEOUT == err) {
        Eeprom_Timeout_UserCallback();
    }
    flag = TRUE;
    while (flag) {
        OSSchedLock();                  // 禁止任务调度,否则可能会影响到I2C的时序
        EepromDMADone = 0;
        addr = (uint8)(SubAddr & (EEPROM_PAGESIZE - 1));
        if (addr + Count <= EEPROM_PAGESIZE) {
            result = Eeprom_AccessProc(EEPROM_DEVADDR, SubAddr, BufPtr, Count, AccessMode);
            flag = FALSE;
        } else {
            num = EEPROM_PAGESIZE - addr;
            result = Eeprom_AccessProc(EEPROM_DEVADDR, SubAddr, BufPtr, num, AccessMode);
            SubAddr += num;
            Count -= num;
            BufPtr += num;
        }
        OSSchedUnlock();
        timeOut = 0;
        while (0 == EepromDMADone && timeOut ++ < 200) {
            OSTimeDly(TIME_DELAY_MS(10));
        }

        if (WRITE_ATTR == AccessMode) {                     // 要大于20MS,留给Eeprom写,此处不可以改写
            OSTimeDly(TIME_DELAY_MS(20));
        }
        // DMA中断中发生了错误
        if (2 == EepromDMADone) {
            Eeprom_Timeout_UserCallback();
            result = ERROR;
        }

        // 出现错误中断传输
        if (result == ERROR) {
            break;
        }
    }

    OSMutexPost(EepromAccessMutex);
    return result;
}

/***************************************End of file*********************************************/

