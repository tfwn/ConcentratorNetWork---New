/************************************************************************************************
*                                   SRWF-6009
*    (c) Copyright 2015, Software Department, Sunray Technology Co.Ltd
*                               All Rights Reserved
*
* FileName     : SerialPort.c
* Description  : 此函数用于处理5个串口和1个Usb端口的数据收发
* Version      :
* Function List:
*------------------------------Revision History--------------------------------------------------
* No.   Version     Date            Revised By      Item            Description
* 1     V1.0        07/16/2015      Zhangxp         SRWF-6009       Original Version
************************************************************************************************/

#define SERIALPORT_GLOBALS

/************************************************************************************************
*                             Include File Section
************************************************************************************************/
#include "Stm32f10x_conf.h"
#include "ucos_ii.h"
#include "Main.h"
#include "SerialPort.h"
#include "Gprs.h"
#include "Led.h"
#include "Usb_lib.h"

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
* Function Name: Serialport_RxFilter
* Decription   : 端口接收数据处理函数
* Input        : PortNo-串口号,Ch-接收的字节
* Output       : 无
* Others       : 无
************************************************************************************************/
void Serialport_RxFilter(PORT_NO PortNo)
{
    uint16 i, j;
    PORT_BUF_FORMAT *bufPtr;

    bufPtr = (PORT_BUF_FORMAT *)(SerialPort.Port[PortNo].RxMemPtr);
    switch (PortNo) {
        case Usart_Debug:
        case Usart_Rf:
        case Uart_Gprs:
        case Uart_Rs485:
            if (bufPtr->Length >= 2 &&
                SYNCWORD1 == bufPtr->Buffer[bufPtr->Length - 2] &&
                SYNCWORD2 == bufPtr->Buffer[bufPtr->Length - 1]) {
                bufPtr->Length = 0;
                bufPtr->Property.FilterDone = TRUE;
            }
            break;
        case Usb_Port:
            i = (bufPtr->Length > USB_PKG_SIZE) ? (bufPtr->Length - USB_PKG_SIZE - 1) : 0;
            for ( ; i < bufPtr->Length - 1; i++) {
                if (SYNCWORD1 == bufPtr->Buffer[i] &&
                    SYNCWORD2 == bufPtr->Buffer[i + 1]) {
                    i += 2;
                    bufPtr->Property.FilterDone = TRUE;
                    break;
                }
            }
            if (TRUE == bufPtr->Property.FilterDone) {
                for (j = 0; i < bufPtr->Length; i++, j++) {
                    bufPtr->Buffer[j] = bufPtr->Buffer[i];
                }
                bufPtr->Length = j;
            }
            break;
        default:
            break;
    }
}

/************************************************************************************************
* Function Name: SerialPort_DelayProc
* Decription   : 串行端口接收数据延时函数
* Input        : 无
* Output       : 无
* Others       : 无
************************************************************************************************/
void SerialPort_DelayProc(void)
{
    uint8 err;
    PORT_NO portNo;
    SERIALPORT_PORT *portPtr;

    for (portNo = Port_Start; portNo < Port_Total; portNo++) {
        portPtr = &SerialPort.Port[portNo];

        // 接收数据超时处理
        if (SERIALPORT_TIMER_STOP == portPtr->RxTimer) {
        } else if (portPtr->RxTimer > 0) {
            portPtr->RxTimer--;
        } else {
            if (OS_ERR_NONE == OSMboxPost(portPtr->MboxRx, portPtr->RxMemPtr)) {
                portPtr->RxTimer = SERIALPORT_TIMER_STOP;
                OSFlagPost(GlobalEventFlag, (OS_FLAGS)(1 << portNo), OS_FLAG_SET, &err);
                portPtr->RxMemPtr = (void *)0;
            } else if (portPtr->MboxPostTime++ > MBOX_POST_MAX_TIME) {
                OSMemPut(LargeMemoryPtr, portPtr->RxMemPtr);
                portPtr->RxTimer = SERIALPORT_TIMER_STOP;
                portPtr->RxMemPtr = (void *)0;
            } else {
                portPtr->RxTimer = 20;
            }
        }

        // 发送数据超时处理
        if (SERIALPORT_TIMER_STOP == portPtr->TxTimer) {
        } else if (portPtr->TxTimer > 0) {
            portPtr->TxTimer--;
        } else {
            if ((void *)0 != portPtr->TxMemPtr) {
                OSMemPut(LargeMemoryPtr, portPtr->TxMemPtr);
                portPtr->TxMemPtr = (void *)0;
            }
            portPtr->TxTimer = SERIALPORT_TIMER_STOP;
            if (Uart_Rs485 == portNo) {
                RS485_RX_ON();
            }
        }
    }
}

/************************************************************************************************
* Function Name: Uart_RxProc
* Decription   : Uart接收数据处理函数
* Input        : UartNo-串口号,Ch-接收的字节
* Output       : 无
* Others       : 无
************************************************************************************************/
void Uart_RxProc(PORT_NO UartNo, uint8 Ch)
{
    uint8 err;
    SERIALPORT_PORT *portPtr = &SerialPort.Port[UartNo];
    PORT_BUF_FORMAT *bufPtr;

    if (UartNo >= Uart_Total || OSRunning == OS_FALSE) {
        return;
    }

    // 如果没有接收缓冲区,则申请一个接收缓冲区
    if ((void *)0 == portPtr->RxMemPtr) {
        portPtr->RxMemPtr = OSMemGet(LargeMemoryPtr, &err);
        if (OS_ERR_NONE == err) {
            bufPtr = (PORT_BUF_FORMAT *)(portPtr->RxMemPtr);
            bufPtr->Length = 0;
            bufPtr->Property.PortNo = UartNo;
            bufPtr->Property.FilterDone = FALSE;
            portPtr->MboxPostTime = 0;
        }
    }

    // 有接收缓冲区则直接接收数据
    if ((void *)0 != portPtr->RxMemPtr) {
        bufPtr = (PORT_BUF_FORMAT *)(portPtr->RxMemPtr);
        if (bufPtr->Length < MEM_LARGE_BLOCK_LEN - 3) {
            bufPtr->Buffer[bufPtr->Length++] = Ch;
            if (FALSE == bufPtr->Property.FilterDone) {
                Serialport_RxFilter(UartNo);
            }
        }
        portPtr->RxTimer = portPtr->RxOutTime;
    }
}

/************************************************************************************************
* Function Name: Uart_TxProc
* Decription   : Uart发送数据处理函数
* Input        : UartNo-串口号,Ch-发送的字节指针
* Output       : True-有数据要发送,False-数据发送完毕
* Others       : 无
************************************************************************************************/
bool Uart_TxProc(PORT_NO UartNo, uint8 *Ch)
{
    SERIALPORT_PORT *portPtr = &SerialPort.Port[UartNo];
    PORT_BUF_FORMAT *bufPtr;

    if (UartNo >= Uart_Total || (void *)0 == portPtr->TxMemPtr) {
        return FALSE;
    }

    bufPtr = (PORT_BUF_FORMAT *)(portPtr->TxMemPtr);

    // 如果全部数据都已经发送完毕,则释放发送缓冲区到内存中
    if (portPtr->TxCounter >= bufPtr->Length) {
        OSMemPut(LargeMemoryPtr, portPtr->TxMemPtr);
        portPtr->TxMemPtr = (void *)0;
        portPtr->TxTimer = SERIALPORT_TIMER_STOP;
        return FALSE;
    } else {
        * Ch = bufPtr->Buffer[portPtr->TxCounter];
        portPtr->TxCounter++;
        portPtr->TxTimer = SERIALPORT_TX_MAX_TIME;
        return TRUE;
    }
}

/************************************************************************************************
* Function Name: Uart_IRQHandler
* Decription   : USART公共中断服务程序
* Input        : USARTx-Uart端口,UartNo-被定义的对应功能号码
* Output       : 无
* Others       : 无
************************************************************************************************/
void Uart_IRQHandler(USART_TypeDef *USARTx, PORT_NO UartNo)
{
    uint8 ch;

    OSIntEnter();
    Led_FlashTime(WORKSTATUS_LED, Delay50ms, Delay50ms, TRUE);
    if (USART_GetFlagStatus(USARTx, USART_FLAG_ORE) != RESET) {         // 注意!不能使用if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)来判断
        USART_ReceiveData(USARTx);
    }

    if (USART_GetITStatus(USARTx, USART_IT_RXNE) != RESET) {            // 串口接收中断,接收数据
        ch = USART_ReceiveData(USARTx);
        USART_ClearITPendingBit(USARTx, USART_IT_RXNE);
        Uart_RxProc(UartNo, ch);
    }

    if (USART_GetITStatus(USARTx, USART_IT_TXE) != RESET) {             // 数据寄存器空中断,串口发送数据
        USART_ClearITPendingBit(USARTx, USART_IT_TXE);
        if (TRUE == Uart_TxProc(UartNo, &ch)) {
            USART_SendData(USARTx, ch);
        } else if (Uart_Rs485 != UartNo) {
            USART_ITConfig(USARTx, USART_IT_TXE, DISABLE);              // 发送缓冲区空中断禁止
        }
    }

    if (USART_GetFlagStatus(USARTx, USART_FLAG_LBD) != RESET) {
        USART_ClearITPendingBit(USARTx, USART_IT_LBD);
    }

    if (USART_GetFlagStatus(USARTx, USART_FLAG_TC) != RESET) {
        USART_ClearITPendingBit(USARTx, USART_IT_TC);
        if (Uart_Rs485 == UartNo) {
            RS485_RX_ON();
            USART_ITConfig(USARTx, USART_IT_TXE, DISABLE);
        }
    }
    Led_FlashTime(WORKSTATUS_LED, Delay100ms, Delay3000ms, FALSE);
    OSIntExit();
}

/************************************************************************************************
* Function Name: USART1_IRQHandler
* Decription   : USART1中断服务程序
* Input        : 无
* Output       : 无
* Others       : 无
************************************************************************************************/
void USART1_IRQHandler(void)
{
    Uart_IRQHandler(USART1, Usart_Debug);
}

/************************************************************************************************
* Function Name: USART2_IRQHandler
* Decription   : USART2中断服务程序
* Input        : 无
* Output       : 无
* Others       : 无
************************************************************************************************/
void USART2_IRQHandler(void)
{
    Uart_IRQHandler(USART2, Usart_Ir);
}

/************************************************************************************************
* Function Name: USART3_IRQHandler
* Decription   : USART3中断服务程序
* Input        : 无
* Output       : 无
* Others       : 无
************************************************************************************************/
void USART3_IRQHandler(void)
{
    Uart_IRQHandler(USART3, Usart_Rf);
}

/************************************************************************************************
* Function Name: UART4_IRQHandler
* Decription   : UART4中断服务程序
* Input        : 无
* Output       : 无
* Others       : 无
************************************************************************************************/
void UART4_IRQHandler(void)
{
    Uart_IRQHandler(UART4, Uart_Rs485);
}

/************************************************************************************************
* Function Name: UART5_IRQHandler
* Decription   : UART5中断服务程序
* Input        : 无
* Output       : 无
* Others       : 无
************************************************************************************************/
void UART5_IRQHandler(void)
{
    Uart_IRQHandler(UART5, Uart_Gprs);
}

/************************************************************************************************
* Function Name: Uart_ParameterInit
* Decription   : 串口参数初始化函数
* Input        : UartNo-串口号
*                Baudrate-波特率
* Output       : 无
* Others       : RxOutTime用来判断串口接收结束,当最后一个字节接收完毕后延时RxOutTime时间,在这段时
*                间内没有收到新的字节即认为数据接收完毕
************************************************************************************************/
static void Uart_ParameterInit(PORT_NO UartNo, uint32 Baudrate)
{
    SERIALPORT_PORT *portPtr = &SerialPort.Port[UartNo];
    if (UartNo >= Uart_Total) {
        return;
    }

    portPtr->TxTimer = SERIALPORT_TIMER_STOP;
    portPtr->TxCounter = 0;
    portPtr->TxMemPtr = (void *)0;
    portPtr->MboxPostTime = 0;
    portPtr->RxTimer = SERIALPORT_TIMER_STOP;
    portPtr->RxOutTime = Baudrate > 9600 ? 10 : 20;
    portPtr->RxMemPtr = (void *)0;
    portPtr->MboxTx = (void *)0;
    portPtr->MboxRx = (void *)0;
}

/************************************************************************************************
* Function Name: Uart_Config
* Decription   : 串口功能配置函数
* Input        : UartNo-串口号
*                Parity-奇偶检验
*                DataBits-数据位
*                StopBits-停止位
*                Baudrate-波特率
* Output       : 无
* Others       : 无
************************************************************************************************/
static void Uart_Config(PORT_NO UartNo,
                         PARITY_TYPE Parity,
                         DATABITS_TYPE DataBits,
                         STOPBITS_TYPE StopBits,
                         uint32 Baudrate)
{
    USART_InitTypeDef USART_InitStruct;
    GPIO_InitTypeDef GPIO_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    switch (UartNo) {
        case Usart_Debug:
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);                      // Usart1采用APB2时钟，其余Usart采用APB1时钟
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA, ENABLE); // 使能usart1时钟

            GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;                                      // USART1_Tx PA9
            GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;                                // GPIO_Mode_AF_PP 复用推挽输出,修改为复用开漏输出
            GPIO_Init(GPIOA, &GPIO_InitStruct);
            GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;                                     // USART1_Rx PA10
            GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;                          // 普通输入模式(浮空)
            GPIO_Init(GPIOA, &GPIO_InitStruct);

            USART_InitStruct.USART_BaudRate = Baudrate;                                 // 波特率
            USART_InitStruct.USART_WordLength = DataBits;                               // 位长
            USART_InitStruct.USART_StopBits = StopBits;                                 // 停止位数
            USART_InitStruct.USART_Parity = Parity;                                     // 奇偶校验
            USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
            USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;// 流控设为无
            USART_Init(USART1, &USART_InitStruct);                                      // 设置串口参数

            NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;                              // Usart1中断设置
            NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
            NVIC_InitStruct.NVIC_IRQChannelSubPriority = 3;
            NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
            NVIC_Init(&NVIC_InitStruct);

            USART_Cmd(USART1, ENABLE);
            USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);                              // 先使能接收中断
            USART_ITConfig(USART1, USART_IT_TXE, DISABLE);                              // 先禁止发送中断,若先使能发送中断，则可能会先发0x00

            break;

        case Usart_Ir:
            RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA, ENABLE); // 使能Usart2时钟

            GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;                                      // USART2_Tx PA2
            GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;                                // 复用推挽输出
            GPIO_Init(GPIOA, &GPIO_InitStruct);
            GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;                                      // USART2_Rx PA3
            GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;                          // 普通输入模式(浮空)
            GPIO_Init(GPIOA, &GPIO_InitStruct);

            USART_InitStruct.USART_BaudRate = Baudrate;                                 // 波特率
            USART_InitStruct.USART_WordLength = DataBits;                               // 位长
            USART_InitStruct.USART_StopBits = StopBits;                                 // 停止位数
            USART_InitStruct.USART_Parity = Parity;                                     // 奇偶校验
            USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
            USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;// 流控设为无
            USART_Init(USART2, &USART_InitStruct);                                      // 设置串口参数

            NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;                              // Usart2中断设置
            NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
            NVIC_InitStruct.NVIC_IRQChannelSubPriority = 5;
            NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
            NVIC_Init(&NVIC_InitStruct);

            USART_Cmd(USART2, ENABLE);
            USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);                              // 先使能接收中断
            USART_ITConfig(USART2, USART_IT_TXE, DISABLE);                              // 先禁止发送中断,若先使能发送中断，则可能会先发0x00

            break;

        case Usart_Rf:
            RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE); // 使能Usart3时钟

            GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;                                     // USART3_Tx PB10
            GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStruct.GPIO_Mode =  GPIO_Mode_AF_PP;                               // GPIO_Mode_AF_PP 复用推挽输出 修改为复用开漏输出
            GPIO_Init(GPIOB, &GPIO_InitStruct);
            GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;                                     // USART3_Rx PB11
            GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;                          // 普通输入模式(浮空)
            GPIO_Init(GPIOB, &GPIO_InitStruct);

            USART_InitStruct.USART_BaudRate = Baudrate;                                 // 波特率
            USART_InitStruct.USART_WordLength = DataBits;                               // 位长
            USART_InitStruct.USART_StopBits = StopBits;                                 // 停止位数
            USART_InitStruct.USART_Parity = Parity;                                     // 奇偶校验
            USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
            USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;// 流控设为无
            USART_Init(USART3, &USART_InitStruct);                                      // 设置串口参数

            NVIC_InitStruct.NVIC_IRQChannel = USART3_IRQn;                              // Usart3中断设置
            NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
            NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
            NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
            NVIC_Init(&NVIC_InitStruct);

            USART_Cmd(USART3, ENABLE);
            USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);                              // 先使能接收中断
            USART_ITConfig(USART3, USART_IT_TXE, DISABLE);                              // 先禁止发送中断,若先使能发送中断，则可能会先发00

            break;

        case Uart_Rs485:
            RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOC, ENABLE); // 使能Uart4时钟

            GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;                                     // UART4_Tx PC10
            GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;                                // GPIO_Mode_AF_PP 复用推挽输出 修改为复用开漏输出
            GPIO_Init(GPIOC, &GPIO_InitStruct);
            GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;                                     // UART4_Rx PC11
            GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;                          // 普通输入模式(浮空)
            GPIO_Init(GPIOC, &GPIO_InitStruct);

            GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;
            GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
            GPIO_Init(GPIOC, &GPIO_InitStruct);                                         // 配置485_RE引脚
            RS485_RX_ON();

            USART_InitStruct.USART_BaudRate = Baudrate;                                 // 波特率
            USART_InitStruct.USART_WordLength = DataBits;                               // 位长
            USART_InitStruct.USART_StopBits = StopBits;                                 // 停止位数
            USART_InitStruct.USART_Parity = Parity;                                     // 奇偶校验
            USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
            USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;// 流控设为无
            USART_Init(UART4, &USART_InitStruct);                                       // 设置串口参数

            NVIC_InitStruct.NVIC_IRQChannel = UART4_IRQn;                               // Uart4中断设置
            NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
            NVIC_InitStruct.NVIC_IRQChannelSubPriority = 4;
            NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
            NVIC_Init(&NVIC_InitStruct);

            USART_Cmd(UART4, ENABLE);
            USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);                               // 先使能接收中断
            USART_ITConfig(UART4, USART_IT_TXE, DISABLE);                               // 先禁止发送中断,若先使能发送中断，则可能会先发00

            break;

        case Uart_Gprs:
            RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);                       // 使能Uart5时钟
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);

            GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;                                     // UART5_Tx PC12
            GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;                                // GPIO_Mode_AF_PP 复用推挽输出
            GPIO_Init(GPIOC, &GPIO_InitStruct);
            GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;                                      // UART5_Rx PD2
            GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;                          // 普通输入模式(浮空)
            GPIO_Init(GPIOD, &GPIO_InitStruct);

            USART_InitStruct.USART_BaudRate = Baudrate;                                 // 波特率
            USART_InitStruct.USART_WordLength = DataBits;                               // 位长
            USART_InitStruct.USART_StopBits = StopBits;                                 // 停止位数
            USART_InitStruct.USART_Parity = Parity;                                     // 奇偶校验
            USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
            USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;// 流控设为无
            USART_Init(UART5, &USART_InitStruct);                                       // 设置串口参数

            NVIC_InitStruct.NVIC_IRQChannel = UART5_IRQn;                               // Uart5中断设置
            NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
            NVIC_InitStruct.NVIC_IRQChannelSubPriority = 2;
            NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
            NVIC_Init(&NVIC_InitStruct);

            USART_Cmd(UART5, ENABLE);
            USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);                               // 先使能接收中断
            USART_ITConfig(UART5, USART_IT_TXE, DISABLE);                               // 先禁止发送中断,若先使能发送中断，则可能会先发00

            break;

        default:

            break;
    }

    // Uart参数配置
    Uart_ParameterInit(UartNo, Baudrate);
}

/************************************************************************************************
* Function Name: Usb_Config
* Decription   : Usb端口配置函数
* Input        : 无
* Output       : 无
* Others       : 无
************************************************************************************************/
void Usb_Config(void)
{
    SERIALPORT_PORT *portPtr = &SerialPort.Port[Usb_Port];

    USB_Init();
    portPtr->TxTimer = SERIALPORT_TIMER_STOP;
    portPtr->TxCounter = 0;
    portPtr->TxMemPtr = (void *)0;
    portPtr->MboxPostTime = 0;
    portPtr->RxTimer = SERIALPORT_TIMER_STOP;
    portPtr->RxOutTime = USB_RX_OUTTIME;
    portPtr->RxMemPtr = (void *)0;
    portPtr->MboxTx = (void *)0;
    portPtr->MboxRx = (void *)0;
}

/************************************************************************************************
* Function Name: Usb_RxProc
* Decription   : Usb端口接收数据处理函数
* Input        : 无
* Output       : 无
* Others       : 无
************************************************************************************************/
void Usb_RxProc(void)
{
    uint8 len;
    SERIALPORT_PORT *portPtr = &SerialPort.Port[Usb_Port];
    PORT_BUF_FORMAT *bufPtr;

    if (OSRunning == OS_FALSE) {
        return;
    }

    len = GetEPRxCount(ENDP1);
    Led_FlashTime(WORKSTATUS_LED, Delay50ms, Delay50ms, TRUE);

    // 如果没有接收缓冲区,则申请一个接收缓冲区
    if ((void *)0 == portPtr->RxMemPtr) {
        if ((void *)0 != (portPtr->RxMemPtr = OSMemGetOpt(LargeMemoryPtr, 10, TIME_DELAY_MS(50)))) {
            bufPtr = (PORT_BUF_FORMAT *)(portPtr->RxMemPtr);
            bufPtr->Length = 0;
            bufPtr->Property.PortNo = Usb_Port;
            bufPtr->Property.FilterDone = FALSE;
            portPtr->MboxPostTime = 0;
        }
    }

    // 有接收缓冲区则直接接收数据
    if ((void *)0 != portPtr->RxMemPtr) {
        bufPtr = (PORT_BUF_FORMAT *)(portPtr->RxMemPtr);
        if (bufPtr->Length < MEM_LARGE_BLOCK_LEN - 3 - len) {
            PMAToUserBufferCopy(bufPtr->Buffer + bufPtr->Length, ENDP1_RXADDR, len);
            bufPtr->Length += len;
            if (FALSE == bufPtr->Property.FilterDone) {
                Serialport_RxFilter(Usb_Port);
            }
        }
        SetEPRxValid(ENDP1);
        portPtr->RxTimer = portPtr->RxOutTime;
    }
    Led_FlashTime(WORKSTATUS_LED, Delay100ms, Delay3000ms, FALSE);
}

/************************************************************************************************
* Function Name: Usb_TxProc
* Decription   : Usb端口发送数据处理函数
* Input        : 无
* Output       : 无
* Others       : 无
************************************************************************************************/
void Usb_TxProc(void)
{
    SERIALPORT_PORT *portPtr = &SerialPort.Port[Usb_Port];
    PORT_BUF_FORMAT *bufPtr = (PORT_BUF_FORMAT *)(portPtr->TxMemPtr);

    if ((void *)0 == portPtr->TxMemPtr) {
        return;
    }

    Led_FlashTime(WORKSTATUS_LED, Delay50ms, Delay50ms, TRUE);
    if (EP_TX_NAK == GetEPTxStatus(ENDP1)) {
        portPtr->TxCounter += USB_PKG_SIZE;
        if (portPtr->TxCounter >= bufPtr->Length) {
            OSMemPut(LargeMemoryPtr, portPtr->TxMemPtr);
            portPtr->TxMemPtr = (void *)0;
            portPtr->TxTimer = SERIALPORT_TIMER_STOP;
            Led_FlashTime(WORKSTATUS_LED, Delay100ms, Delay3000ms, FALSE);
            return;
        } else {
            UserToPMABufferCopy(bufPtr->Buffer + portPtr->TxCounter, GetEPTxAddr(ENDP1), USB_PKG_SIZE);
            SetEPTxCount(ENDP1, USB_PKG_SIZE);
            SetEPTxValid(ENDP1);
            portPtr->TxTimer = SERIALPORT_TX_MAX_TIME;
        }
    }
    Led_FlashTime(WORKSTATUS_LED, Delay100ms, Delay3000ms, FALSE);
}

/************************************************************************************************
* Function Name: SerialPort_Init
* Decription   : 串行端口初始化函数
* Input        : 无
* Output       : 无
* Others       : 串行口包括5个Uart口和1个Usb口
************************************************************************************************/
void SerialPort_Init(void)
{
    Uart_Config(Usart_Debug, Parity_Even, DataBits_9, StopBits_1, 115200);          // Debug串口初始化
    Uart_Config(Usart_Ir, Parity_Even, DataBits_9, StopBits_1, 9600);               // IR串口初始化
    Uart_Config(Usart_Rf, Parity_Even, DataBits_9, StopBits_1, 9600);               // RF串口初始化
    Uart_Config(Uart_Rs485, Parity_Even, DataBits_9, StopBits_1, 9600);             // RS485串口初始化
    Uart_Config(Uart_Gprs, Parity_None, DataBits_8, StopBits_1, 115200);            // GPRS串口初始化
    Usb_Config();                                                                   // USB端口初始化
}

/************************************************************************************************
* Function Name: SerialPort_Task
* Decription   : 串口数据处理任务函数,只处理发送事件
* Input        : *p_arg-参数指针
* Output       : 无
* Others       : 无
************************************************************************************************/
void SerialPort_Task(void *p_arg)
{
    uint8 retry, err, *msg;
    PORT_NO portNo;
    OS_FLAGS eventFlag;
    SERIALPORT_PORT *portPtr;

    (void)p_arg;

    while (TRUE ) {
        eventFlag = OSFlagPend(GlobalEventFlag, (OS_FLAGS)SERIALPORT_TX_EVENT_FILTER, (OS_FLAG_WAIT_SET_ANY | OS_FLAG_CONSUME), TIME_DELAY_MS(5000), &err);

        // 得到了要发送的串口数据(标识位于高字节)
        eventFlag >>= SERIALPORT_TX_FLAG_OFFSET;
        for (portNo = Port_Start; portNo < Port_Total; portNo++) {
            portPtr = &SerialPort.Port[portNo];
            if (eventFlag & (1 << portNo)) {
                msg = OSMboxAccept(portPtr->MboxTx);
                if ((void *)0 != msg) {
                    // 如果长度超标则放弃
                    if (((PORT_BUF_FORMAT *)msg)->Length > MEM_LARGE_BLOCK_LEN - 3) {
                        OSMemPut(LargeMemoryPtr, msg);
                        continue;
                    }
                    // 如果发送缓冲区中有数据,则连续请求3次
                    retry = 3;
                    while ((void *)0 != portPtr->TxMemPtr && retry) {
                        retry -= 1;
                        OSTimeDlyHMSM(0, 0, 0, 200);
                    }
                    if (0 == retry) {
                        OSMemPut(LargeMemoryPtr, msg);
                    } else {
                        portPtr->TxMemPtr = msg;
                        portPtr->TxCounter = 0;
                        portPtr->TxTimer = SERIALPORT_TX_MAX_TIME;
                        switch (portNo) {
                            case Usart_Debug:
                                USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
                                break;
                            case Usart_Ir:
                                USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
                                break;
                            case Usart_Rf:
                                USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
                                break;
                            case Uart_Rs485:
                                RS485_TX_ON();
                                USART_ITConfig(UART4, USART_IT_TXE, ENABLE);
                                break;
                            case Uart_Gprs:
                                USART_ITConfig(UART5, USART_IT_TXE, ENABLE);
                                break;
                            case Usb_Port:
                                UserToPMABufferCopy(((PORT_BUF_FORMAT *)(portPtr->TxMemPtr))->Buffer, GetEPTxAddr(ENDP1), USB_PKG_SIZE);
                                SetEPTxCount(ENDP1, USB_PKG_SIZE);
                                SetEPTxValid(ENDP1);
                                break;
                            default:
                                OSMemPut(LargeMemoryPtr, msg);
                                break;
                        }
                    }
                }
            }
        }
    }
}

/***************************************End of file*********************************************/

