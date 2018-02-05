/************************************************************************************************
*                                   SRWF-6009
*    (c) Copyright 2015, Software Department, Sunray Technology Co.Ltd
*                               All Rights Reserved
*
* FileName     : Main.h
* Description  :
* Version      :
* Function List:
*------------------------------Revision History--------------------------------------------------
* No.   Version     Date            Revised By      Item            Description
* 1     V1.0        07/16/2015      Zhangxp         SRWF-6009       Original Version
************************************************************************************************/

#ifndef  MAIN_H
#define  MAIN_H


#ifdef   MAIN_GLOBALS
#define  MAIN_EXT
#else
#define  MAIN_EXT  extern
#endif

/************************************************************************************************
*                               Pubilc Macro Define Section
************************************************************************************************/
// 全局信号量集定义(定义顺序不能打乱,否则执行会出错)
#define FLAG_USART_DEBUG_RX                         BIT(Usart_Debug)
#define FLAG_USART_IR_RX                            BIT(Usart_Ir)
#define FLAG_USART_RF_RX                            BIT(Usart_Rf)
#define FLAG_UART_RS485_RX                          BIT(Uart_Rs485)
#define FLAG_UART_GPRS_RX                           BIT(Uart_Gprs)
#define FLAG_USB_RX                                 BIT(Usb_Port)
#define FLAG_GPRS_RX                                BIT(Gprs_Port)
#define FLAG_RESVER1                                BIT(7)

#define FLAG_USART_DEBUG_TX                         BIT(Usart_Debug + SERIALPORT_TX_FLAG_OFFSET)
#define FLAG_USART_IR_TX                            BIT(Usart_Ir + SERIALPORT_TX_FLAG_OFFSET)
#define FLAG_USART_RF_TX                            BIT(Usart_Rf + SERIALPORT_TX_FLAG_OFFSET)
#define FLAG_UART_RS485_TX                          BIT(Uart_Rs485 + SERIALPORT_TX_FLAG_OFFSET)
#define FLAG_UART_GPRS_TX                           BIT(Uart_Gprs + SERIALPORT_TX_FLAG_OFFSET)
#define FLAG_USB_TX                                 BIT(Usb_Port + SERIALPORT_TX_FLAG_OFFSET)
#define FLAG_GPRS_TX                                BIT(Gprs_Port + SERIALPORT_TX_FLAG_OFFSET)
#define FLAG_RESVER2                                BIT(15)

#define FLAG_DELAY_SAVE_TIMER                       BIT(16)
#define FLAG_DATA_REPLENISH_TIMER                   BIT(17)
#define FLAG_DATA_UPLOAD_TIMER                      BIT(18)
#define FLAG_RTC_TIMING_TIMER                       BIT(19)
#define FLAG_HEATBEAT_TIMER                         BIT(20)
#define FLAG_LOGOFF_EVENT                           BIT(21)
#define FLAG_MODIFY_SCAN_CHANNEL                    BIT(22)

// 内存块长度定义
#define MEM_LARGE_BLOCK_LEN                         1024
#define MEM_SMALL_BLOCK_LEN                         200

/************************************************************************************************
*                                   Enum Define Section
************************************************************************************************/

/************************************************************************************************
*                                  Struct Define Section
************************************************************************************************/

/************************************************************************************************
*                                 Variable Declare Section
************************************************************************************************/
MAIN_EXT OS_FLAG_GRP *GlobalEventFlag;

// 内存块指针定义
MAIN_EXT OS_MEM *SmallMemoryPtr;
MAIN_EXT OS_MEM *LargeMemoryPtr;

// 内存块分配定义
MAIN_EXT uint8 MemoryLargeBlock[TOTAL_LARGE_BLOCK][MEM_LARGE_BLOCK_LEN];
MAIN_EXT uint8 MemorySmallBlock[TOTAL_SMALL_BLOCK][MEM_SMALL_BLOCK_LEN];

// 主任务内存分配定义
MAIN_EXT OS_STK MainTask_Stk[TASK_MAIN_STK_SIZE];

/************************************************************************************************
*                            Function Declare Section
************************************************************************************************/


#endif
/***************************************End of file*********************************************/

