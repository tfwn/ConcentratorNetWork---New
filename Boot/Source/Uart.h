/************************************************************************************************
*                                   SRWF-6009-BOOT
*    (c) Copyright 2015, Software Department, Sunray Technology Co.Ltd
*                               All Rights Reserved
*
* FileName     : Uart.h
* Description  :
* Version      :
* Function List:
*------------------------------Revision History--------------------------------------------------
* No.   Version     Date            Revised By      Item            Description
* 1     V1.0        10/08/2015      Zhangxp         SRWF-6009-BOOT  Original Version
************************************************************************************************/
#ifndef _UART_H_
#define _UART_H_

/************************************************************************************************
*                             Include File Section
************************************************************************************************/
#include "stm32f10x.h"

/************************************************************************************************
*                          Pubilc Macro Define Section
************************************************************************************************/
#define UART1_TX_INT_ENABLE                     USART_ITConfig(USART1, USART_IT_TXE, ENABLE)
#define UART1_TX_INT_DISABLE                    USART_ITConfig(USART1, USART_IT_TXE, DISABLE)

#define USART1_TX_PIN                           GPIO_Pin_9           /* MCU->PA9        */
#define USART1_RX_PIN                           GPIO_Pin_10          /* MCU->PA10       */

#define UART_BUF_SIZE                           255
#define UART_RX_OVERTIME                        15                      // 判断串口接收完成的超时时间(ms)

/************************************************************************************************
*                                   Enum Define Section
************************************************************************************************/
// Uart通讯端口定义
typedef enum
{
    UART_NAME_USART1 = USART1_BASE,
    UART_NAME_UART5 = UART5_BASE
}UART_NAME_ENUM;

// Uart通讯波特率定义
typedef enum
{
    UART_BAUDRATE_1200 = 1200,
    UART_BAUDRATE_2400 = 2400,
    UART_BAUDRATE_4800 = 4800,
    UART_BAUDRATE_9600 = 9600,
    UART_BAUDRATE_14400 = 14400,

    UART_BAUDRATE_19200 = 19200,
    UART_BAUDRATE_38400 = 38400,
    UART_BAUDRATE_57600 = 57600,
    UART_BAUDRATE_115200 = 115200
}UART_BAUDRATE_ENUM;

// Uart数据位长度定义
typedef enum
{
    UART_DATA_BITS_7 = USART_WordLength_8b,
    UART_DATA_BITS_8 = USART_WordLength_9b
}UART_DATA_BITS_ENUM;

// Uart奇偶校验位定义
typedef enum
{
    UART_PARITY_NONE = USART_Parity_No,
    UART_PARITY_EVEN = USART_Parity_Even,
    UART_PARITY_ODD = USART_Parity_Odd
}UART_PARITY_ENUM;

// Uart停止位定义
typedef enum
{
    UART_STOP_BITS_0_5 = USART_StopBits_0_5,
    UART_STOP_BITS_1 = USART_StopBits_1,
    UART_STOP_BITS_1_5 = USART_StopBits_1_5,
    UART_STOP_BITS_2 = USART_StopBits_2
}UART_STOP_BITS_ENUM;

/************************************************************************************************
*                                   Struct Define Section
************************************************************************************************/
// UART串口属性库定义
typedef struct
{
    uint8 TxCount;                                          // 发送计数变量
    uint8 RxTimer;                                          // 最后一包接收后判断接收完毕的时间控制
    uint8 TxLen;
    uint8 RxLen;
    uint8 TxBuf[UART_BUF_SIZE];
    uint8 RxBuf[UART_BUF_SIZE];
}UART_PARAM_STRUCT;

/************************************************************************************************
*                            Variable Declare Section
************************************************************************************************/
extern UART_PARAM_STRUCT Uart1;


/************************************************************************************************
*                            Function Declare Section
************************************************************************************************/
extern void Uart_Init(void);
extern void Uart_Handler(void);
extern void Uart_StartTx(void);
extern void Uart_IntHandler(void);

#endif

/**************************************End of file**********************************************/

