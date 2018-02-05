/************************************************************************************************
*                                   SRWF-6009-BOOT
*    (c) Copyright 2015, Software Department, Sunray Technology Co.Ltd
*                               All Rights Reserved
*
* FileName     : Uart_Update.h
* Description  :
* Version      :
* Function List:
*------------------------------Revision History--------------------------------------------------
* No    Version     Date            Revised By      Item            Description
* 1     V1.0        10/08/2015      Zhangxp         SRWF-6009-BOOT  Original Version
************************************************************************************************/
#ifndef _UART_UPDATE_H_
#define _UART_UPDATE_H_


/************************************************************************************************
*                                   Macro Define Section
************************************************************************************************/
#define UART_ENTER_UPDATE_CMD           0x01                // 进入串口升级命令
#define UART_ENTER_UPDATE_ACK           0x02                // 进入串口升级命令应答
#define UART_UPDATEING_CMD              0x03                // 发送升级数据包命令
#define UART_UPDATEING_ACK              0x04                // 发送升级数据包命令应答

/************************************************************************************************
*                                   Struct Define Section
************************************************************************************************/
// Boot相关参数
typedef struct
{
    uint16 SoftVersion;                                     // 升级软件版本
    uint16 CodeFilePkgNum;                                  // 升级文件总包数
    uint16 AllPkgCRC1;                                      // 校验结果1
    uint16 AllPkgCRC2;                                      // 校验结果2
    uint16 AllPkgCRC3;                                      // 校验结果3
    uint16 AllPkgCRC4;                                      // 校验结果4
    uint16 CalAllPkgCRC;                                    // 计算出来的校验结果
    uint16 PkgCounter;                                      // 串口升级时包计数器
    uint16 CurPkgNum;                                       // 当前接收的包号
}BOOT_PARAM_STRUCT;


/************************************************************************************************
*                            Variable Declare Section
************************************************************************************************/


/************************************************************************************************
*                            Function Declare Section
************************************************************************************************/
extern void Uart_UpdateProcess(void);

#endif

/***************************************End of file*********************************************/


