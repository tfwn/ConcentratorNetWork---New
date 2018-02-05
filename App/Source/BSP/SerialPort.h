/************************************************************************************************
*                                   SRWF-6009
*    (c) Copyright 2015, Software Department, Sunray Technology Co.Ltd
*                               All Rights Reserved
*
* FileName     : SerialPort.h
* Description  :
* Version      :
* Function List:
*------------------------------Revision History--------------------------------------------------
* No.   Version     Date            Revised By      Item            Description
* 1     V1.0        07/16/2015      Zhangxp         SRWF-6009       Original Version
************************************************************************************************/

#ifndef  SERIALPORT_H
#define  SERIALPORT_H

#ifdef   SERIALPORT_GLOBALS
#define  SERIALPORT_EXT
#else
#define  SERIALPORT_EXT  extern
#endif

/************************************************************************************************
*                               Pubilc Macro Define Section
************************************************************************************************/
#define SERIALPORT_TIMER_STOP                 0xFF                          // 计时器停止工作
#define SERIALPORT_TX_FLAG_OFFSET             8U                            // 串口发送数据标识偏移量
#define SERIALPORT_TX_EVENT_FILTER            (FLAG_USART_DEBUG_TX |      \
                                               FLAG_USART_IR_TX |         \
                                               FLAG_USART_RF_TX |         \
                                               FLAG_UART_RS485_TX |       \
                                               FLAG_UART_GPRS_TX |        \
                                               FLAG_USB_TX)

#define WORKSTATUS_LED                        Led1
#define USB_RX_OUTTIME                        50                            // Usb接收延时等待时间
#define USB_PKG_SIZE                          64                            // Usb收发时每包的字节数
#define MBOX_POST_MAX_TIME                    5                             // 数据邮箱发送失败后最多重试次数
#define SERIALPORT_TX_MAX_TIME                200                           // 串行数据必须在这个时间段内发送出去

#define RS485_TX_ON()                         GPIO_SetBits(GPIOC, GPIO_Pin_7)
#define RS485_RX_ON()                         GPIO_ResetBits(GPIOC, GPIO_Pin_7)


/************************************************************************************************
*                                   Enum Define Section
************************************************************************************************/
// 定义使用的USART口和UART口,其中1 2 3为USART,4 5为UART,此处要和main.h函数中的对应定义一致
typedef enum {
    Port_Start = 0,                                                         // 端口开始序号
    Usart_Debug = Port_Start,                                               // Debug串口
    Usart_Ir,                                                               // Ir通信串口
    Usart_Rf,                                                               // Rf模块口
    Uart_Rs485,                                                             // Rs485接口
    Uart_Gprs,                                                              // Gprs模块接口,此数据为Gprs直接收到的数据
    Uart_Total,                                                             // Uart端口总数
    Usb_Port = Uart_Total,                                                  // Usb端口
    Port_Total,                                                             // 端口总数
    Gprs_Port = Port_Total,                                                 // Gprs端口,此数据为Gprs数据解包后的数据
    End_Port
} PORT_NO;

// 定义奇偶校验参数:无校验 用户自定义 偶校验 奇校验
typedef enum {
    Parity_Even = (uint16)0x0400,
    Parity_Odd = (uint16)0x0600,
    Parity_None = (uint16)0x0000
} PARITY_TYPE;

// 定义数据长度类型:8位 9位(9位不常用)
typedef enum {
    DataBits_8 = (uint16)0x0000,
    DataBits_9 = (uint16)0x1000
} DATABITS_TYPE;

// 停止位长度:1位 0.5位 2位 1.5位
typedef enum {
    StopBits_1 = (uint16)0x0000,
    StopBits_05 = (uint16)0x1000,
    StopBits_2 = (uint16)0x2000,
    StopBits_15 = (uint16)0x3000
} STOPBITS_TYPE;

/************************************************************************************************
*                                  Struct Define Section
************************************************************************************************/
// 端口数据包的属性
typedef struct {
    uint8 PortNo: 7;                                                        // Bit0-Bit6 指定是从哪个串口收发的
    uint8 FilterDone: 1;                                                    // Bit7 是否经过过滤
} PORT_DATA_PROPERTY;

// 端口收发数据特定的结构定义
typedef struct {
    uint16 Length;                                                          // 数据长度,该长度不包括Property和自己本身
    PORT_DATA_PROPERTY Property;                                            // 数据包的属性
    uint8 Buffer[1];                                                        // 数据数组
} PORT_BUF_FORMAT;

// 串行端口参数的结构体定义
typedef struct {
    uint8 TxTimer;                                                          // 发送超时时间
    uint16 TxCounter;                                                       // 发送的个数
    uint8 *TxMemPtr;                                                        // 发送缓冲区指针
    uint8 MboxPostTime;                                                     // 数据邮箱发送时失败次数
    uint8 RxTimer;                                                          // 接收最后一包后的延时计时器
    uint8 RxOutTime;                                                        // 接收超时时间
    uint8 *RxMemPtr;                                                        // 接收缓冲区指针
    OS_EVENT *MboxTx;                                                       // 发送邮箱
    OS_EVENT *MboxRx;                                                       // 接收邮箱
} SERIALPORT_PORT;

// 串行端口参数结构体变量定义
typedef struct {
    SERIALPORT_PORT Port[Port_Total];                                       // 串行端口定义
    OS_STK Task_Stk[TASK_SERIALPORT_STK_SIZE];                              // 串行端口定义任务堆栈
} SERIALPORT_PARAM;

/************************************************************************************************
*                            Variable Declare Section
************************************************************************************************/
// 串行端口参数定义
SERIALPORT_EXT SERIALPORT_PARAM SerialPort;


/************************************************************************************************
*                            Function Declare Section
************************************************************************************************/
SERIALPORT_EXT void SerialPort_Init(void);
SERIALPORT_EXT void SerialPort_DelayProc(void);
SERIALPORT_EXT void Usb_RxProc(void);
SERIALPORT_EXT void Usb_TxProc(void);
SERIALPORT_EXT void SerialPort_Task(void *p_arg);


#endif
/***************************************End of file*********************************************/

