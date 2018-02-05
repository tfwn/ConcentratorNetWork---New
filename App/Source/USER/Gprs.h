/************************************************************************************************
*                                   SRWF-6009
*    (c) Copyright 2015, Software Department, Sunray Technology Co.Ltd
*                               All Rights Reserved
*
* FileName     : Bsp.h
* Description  :
* Version      :
* Function List:
*------------------------------Revision History--------------------------------------------------
* No.   Version     Date            Revised By      Item            Description
* 1     V1.0        07/14/2015      Zhangxp         SRWF-6009       Original Version
************************************************************************************************/

#ifndef  GPRS_H
#define  GPRS_H


#ifdef   GPRS_GLOBALS
#define  GPRS_EXT
#else
#define  GPRS_EXT  extern
#endif

/************************************************************************************************
*                               Pubilc Macro Define Section
************************************************************************************************/
#define GPRS_POWER_OFF()                GPIO_ResetBits(GPIOC, GPIO_Pin_9)       // Gprs模块电源控制引脚PC9:0-OFF
#define GPRS_POWER_ON()                 GPIO_SetBits(GPIOC, GPIO_Pin_9)         // Gprs模块电源控制引脚PC9:1-ON

#define GPRS_PWRKEY_DOWN()              GPIO_SetBits(GPIOB, GPIO_Pin_8)         // Gprs模块电源按键按下PB8:1-DOWN
#define GPRS_PWRKEY_UP()                GPIO_ResetBits(GPIOB, GPIO_Pin_8)       // Gprs模块电源按键抬起PB8:0-UP

#define GPRS_EMERG_OFF()                GPIO_ResetBits(GPIOB, GPIO_Pin_9)       // Gprs模块急停开关始终关PB9

#define GPRS_STATUS()                   GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5)// Gprs模块状态PB5:1-ON 0-OFF

#define GPRS_NORMALWORKTIME             (30 * 60 * 1000)                        // 正常工作的时间值
#define GPRS_CONTINUERESTARTTIME        10                                      // 允许Gprs模块连续重启的次数
#define GPRS_CONTINUECLOSEPDPTIME       10                                      // 允许Gprs模块连续关闭PDP的次数
#define GPRS_POWERDOWNTIME              10                                      // 多次连接都失败后,模块关机此时间(分钟)后重连

#define GPRS_FLAG_FILTER                (FLAG_UART_GPRS_RX | FLAG_GPRS_TX | FLAG_HEATBEAT_TIMER | FLAG_LOGOFF_EVENT)

#define GPRS_IP_CLOSE                   "CLOSE"
#define GPRS_LOCAL_PORT                 2020
#define GPRS_LED                        Led2

#define GPRS_HEATBEAT_CMD               0x01                                    // 心跳包指令
#define GPRS_LOGOFF_CMD                 0x02                                    // 注销指令
#define GPRS_DATAPKG_CMD                0x09                                    // 数据包指令

#define GPRS_DATA_MAX_DATA              (MEM_LARGE_BLOCK_LEN - 50)              // Gprs每包最大能够发送的字节数,50包括GPRS打包数据和协议帧中数据域之外的数据长度
#define GPRS_BACKUP_IP_CONNECT          3                                       // 连接备用IP地址时的次数

/************************************************************************************************
*                                   Enum Define Section
************************************************************************************************/
// Gprs命令变量
typedef enum {
    GprsPowerOff = 0,                                       // Gprs模块关电源
    GprsPowerOn,                                            // Gprs模块开电源
    GprsRestart,                                            // Gprs模块重新启动
    GprsConfig,                                             // Gprs模块基础配置
    GprsGsmStatus,                                          // Gprs模块网络状态
    GprsDataFormat,                                         // Gprs数据格式设置
    GprsOpenPdp,                                            // Gprs模块打开PDP场景
    GprsOpenTcp,                                            // Gprs模块打开TCP连接
    GprsClosePdp,                                           // Gprs模块关闭PDP场景
    GprsCloseTcp,                                           // Gprs模块关闭TCP连接
    GprsReceiveData,                                        // Gprs模块接收数据
    GprsTransmitData,                                       // Gprs模块发送数据
    GprsCheckConnect,                                       // Gprs模块发送心跳包
    GprsLogoffConnect,                                      // Gprs模块注销连接
    GprsConnectIdleStatus,                                  // Gprs已连接且处于空闲状态
} GPRS_CMD;

// Gprs模块类型
typedef enum {
    Quectel_M35 = 0,                                        // 移远2G GSM模块
    Quectel_Ec20,                                           // 移远4G全网通模块
    Unknow_Type
} GPRS_MODULE_TYPE;

/************************************************************************************************
*                                  Struct Define Section
************************************************************************************************/
// Gprs参数结构体变量定义
typedef struct {
    GPRS_CMD Cmd;                                           // Gprs命令
    GPRS_MODULE_TYPE ModuleType;                            // Gprs模块类型
    uint8 RestartCount;                                     // Gprs热启动的次数计数器
    uint8 ClosePDPCount;                                    // Gprs关闭PDP的次数计数器
    uint8 ConnectCount;                                     // Gprs连接计数器
    uint8 Online: 1;                                        // Gprs在线标识
    uint8 HeartbeatRetryTime: 7;                            // Gprs心跳包失败重试次数
    uint16 HeartbeatInterval;                               // Gprs心跳包间隔
    uint32 WorkTime;                                        // Gprs模块启动后的工作时间
    uint8 LocalIp[4];                                       // Gprs本地IP
    OS_EVENT *MboxTx;                                       // Gprs发送邮箱
    OS_EVENT *MboxRx;                                       // Gprs接收邮箱
    OS_STK Task_Stk[TASK_GPRS_STK_SIZE];                    // Gprs任务堆栈
} GPRS_PARAM;

/************************************************************************************************
*                                 Variable Declare Section
************************************************************************************************/
// Gprs命令参数定义
GPRS_EXT GPRS_PARAM Gprs;

/************************************************************************************************
*                            Function Declare Section
************************************************************************************************/
GPRS_EXT uint8 Gprs_GetCSQ(void);
GPRS_EXT uint8 Gprs_GetIMSI(uint8 *BufPtr);
GPRS_EXT uint8 Gprs_GetGMM(uint8 *BufPtr);
GPRS_EXT void Gprs_Init(void);
GPRS_EXT void Gprs_Task(void *p_arg);


#endif
/***************************************End of file*********************************************/

