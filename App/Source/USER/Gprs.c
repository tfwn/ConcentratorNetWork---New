/************************************************************************************************
*                                   SRWF-6009
*    (c) Copyright 2015, Software Department, Sunray Technology Co.Ltd
*                               All Rights Reserved
*
* FileName     : Gprs.c
* Description  :
* Version      :
* Function List:
*------------------------------Revision History--------------------------------------------------
* No.   Version     Date            Revised By      Item            Description
* 1     V2.1        08/15/2016      Zhangxp         SRWF-6009       Second Version
************************************************************************************************/

#define GPRS_GLOBALS
/************************************************************************************************
*                             Include File Section
************************************************************************************************/
#include "Stm32f10x_conf.h"
#include "ucos_ii.h"
#include "Main.h"
#include "Gprs.h"
#include "SerialPort.h"
#include "Led.h"
#include "Rtc.h"
#include "DataHandle.h"
#include "Database.h"
#include <string.h>

/************************************************************************************************
*                        Global Variable Declare Section
************************************************************************************************/
const char *GprsModuleList[] = {"M35", "EC20"};

/************************************************************************************************
*                           Prototype Declare Section
************************************************************************************************/

/************************************************************************************************
*                           Function Declare Section
************************************************************************************************/

/************************************************************************************************
* Function Name: Gprs_OutputDebugMsg
* Decription   : Gprs输出调试信息
* Input        : NeedTime-是否需要增加时间,StrPtr-调试信息指针
* Output       : 无
* Others       : 无
************************************************************************************************/
void Gprs_OutputDebugMsg(bool NeedTime, uint8 *StrPtr)
{
    uint16 i, len;
    RTC_TIME rtcTime;
    uint8 *bufPtr;

    if ((void *)0 == (bufPtr = OSMemGetOpt(LargeMemoryPtr, 20, TIME_DELAY_MS(50)))) {
        return;
    }
    len = 0;
    if (TRUE == NeedTime) {
        Rtc_Get(&rtcTime, Format_Bcd);
        len += BcdToAscii(&rtcTime.Hour, bufPtr + len, 1, 2);
        *(bufPtr + len++) = ':';
        len += BcdToAscii(&rtcTime.Minute, bufPtr + len, 1, 2);
        *(bufPtr + len++) = ':';
        len += BcdToAscii(&rtcTime.Second, bufPtr + len, 1, 2);
        *(bufPtr + len++) = ' ';
    }
    for (i = 0; i < strlen((char *)StrPtr); i++) {
        *(bufPtr + len++) = *(StrPtr + i);
    }
    DataHandle_OutputMonitorMsg(Gprs_Connect_Msg, bufPtr, len);
    OSMemPut(LargeMemoryPtr, bufPtr);
    return;
}

/************************************************************************************************
* Function Name: Gprs_ConnectProc
* Decription   : Gprs模块连接处理过程
* Input        : CmdPtr-数据指针
*                CmdLen-数据长度,0-表示CmdPtr是字符串,根据strlen计算,数据以0x0D0x0A结尾;
*                                其他-具体的长度,表示CmdPtr是十六进制数据,数据以CTRL+A结尾
*                AckStr1-应答1字符串指针,#为通配符
*                AckStr2-应答2字符串指针,#为通配符
*                WaitAckTime-等待应答的时间
*                PeriodTime-重试时等待的时间
*                TryTimes-应答失败或无应答时重试的次数
*                PostHandle-是否需要后续处理,TRUE则将返回数据指针,后续处理完成后必须要释放这个
*                           缓冲区,FALSE将释放数据缓冲区
* Output       : 失败返回0,成功则返回一个指向内存空间的指针(在PostHandle为TRUE时)或(void *)1
* Others       : 注意,如果成功,则调用该函数的函数必须在处理完返回的内存数据后释放该内存
************************************************************************************************/
void *Gprs_ConnectProc(const char *CmdPtr,
                          uint16 CmdLen,
                          const char *AckStr1,
                          const char *AckStr2,
                          uint32 WaitAckTime,
                          uint32 PeriodTime,
                          uint8 TryTimes,
                          bool PostHandle)
{
    uint8 i, count, loop, len, err, *txMsg, *rxMsg;
    uint16 j, k;
    const char *ackStr;
    PORT_BUF_FORMAT *bufPtr;
    SERIALPORT_PORT *portPtr = &SerialPort.Port[Uart_Gprs];

    OSTimeDlyHMSM(0, 0, 0, 50);
    for (loop = 0; loop < TryTimes; ) {
        if ((void *)0 == (txMsg = OSMemGet(LargeMemoryPtr, &err))) {
            OSTimeDlyHMSM(0, 0, 3, 0);
            continue;
        }

        // 申请到了内存,则创建发送数据并传递给串口
        bufPtr = (PORT_BUF_FORMAT *)txMsg;
        if (CmdLen == 0) {
            strcpy((char *)bufPtr->Buffer, CmdPtr);
            bufPtr->Length = strlen(CmdPtr);
            bufPtr->Buffer[bufPtr->Length++] = 0x0D;
            bufPtr->Buffer[bufPtr->Length++] = 0x0A;
        } else {
            memcpy(bufPtr->Buffer, CmdPtr, CmdLen);
            bufPtr->Length = CmdLen;
            bufPtr->Buffer[bufPtr->Length++] = 0x1A;
        }

        // 注意要先传邮箱,后清空接收缓冲区,最后发Flag
        if (OSMboxPost(portPtr->MboxTx, txMsg) != OS_ERR_NONE) {
            OSMemPut(LargeMemoryPtr, txMsg);
            OSTimeDlyHMSM(0, 0, 1, 0);
            continue;
        }
        rxMsg = OSMboxAccept(portPtr->MboxRx);                                  // 清空接收缓冲区的数据,有可能是上一次通信遗留下来的没有处理的数据
        if ((void *)0 != rxMsg) {
            OSMemPut(LargeMemoryPtr, rxMsg);
        }
        OSFlagPost(GlobalEventFlag, (OS_FLAGS)FLAG_UART_GPRS_TX, OS_FLAG_SET, &err);

        // 等待串口传过来Gprs模块的应答数据
        rxMsg = OSMboxPend(portPtr->MboxRx, WaitAckTime, &err);

        // 如果没有收到数据则重试
        if ((void *)0 == rxMsg) {
            loop += 1;
            continue;
        }

        // 如果收到数据则验证数据的正确性,正确则返回该数据指针
        for (count = 0; count < 2; count++) {
            bufPtr = (PORT_BUF_FORMAT *)rxMsg;
            for (i = 0; i < 2; i++) {
                ackStr = i == 0 ? AckStr1 : AckStr2;
                if ((void *)0 == ackStr) {
                    continue;
                }
                len = strlen(ackStr);
                for (j = 0; j < bufPtr->Length - len; j++) {
                    for (k = 0; k < len; k++) {
                        if (*(ackStr + k) != bufPtr->Buffer[j + k] &&
                            *(ackStr + k) != '#') {
                            break;
                        }
                    }
                    if (k >= len) {
                        if (TRUE == PostHandle) {
                            return rxMsg;
                        } else {
                            OSMemPut(LargeMemoryPtr, rxMsg);
                            return ((void *)1);
                        }
                    }
                }
            }
            OSMemPut(LargeMemoryPtr, rxMsg);
            if (count >= 1) {
                break;
            }

            // 延时的目的是为了等待Gprs的响应时间或者Gprs的处理时间
            OSTimeDly(PeriodTime);

            // 检测延时这段时间内是否有数据发出
            rxMsg = OSMboxAccept(portPtr->MboxRx);
            if ((void *)0 == rxMsg) {
                break;
            }
        }

        loop += 1;
    }

    return ((void *)0);
}

/************************************************************************************************
* Function Name: Gprs_PowerOff
* Decription   : Gprs模块关机
* Input        : 无
* Output       : 无
* Others       : 硬关机,Gprs模块电源断电
************************************************************************************************/
static void Gprs_PowerOff(void)
{
    GPRS_POWER_OFF();
    GPRS_PWRKEY_UP();
    GPRS_EMERG_OFF();
}

/************************************************************************************************
* Function Name: Gprs_PowerON
* Decription   : Gprs模块开机
* Input        : 无
* Output       : SUCCESS-重启动成功,ERROR-重启动失败
* Others       : 硬开机,Gprs模块电源供电,并确认模块已经开机
************************************************************************************************/
static bool Gprs_PowerOn(void)
{
    uint8 i;

    // Gprs模块上电并初始化各引脚电平
    GPRS_POWER_ON();
    GPRS_PWRKEY_UP();
    GPRS_EMERG_OFF();

    for (i = 0; i < 5; i++) {
        if ((void *)1 == Gprs_ConnectProc("AT", 0, "OK", "AT", TIME_DELAY_MS(300), TIME_DELAY_MS(300), 50, FALSE)) {
            return SUCCESS;
        }
        GPRS_PWRKEY_DOWN();
        OSTimeDlyHMSM(0, 0, 5, 0);
        GPRS_PWRKEY_UP();
    }
    return ERROR;
}

/************************************************************************************************
* Function Name: Gprs_Restart
* Decription   : Gprs模块热启动
* Input        : 无
* Output       : SUCCESS-重启动成功,ERROR-重启动失败
* Others       : Gprs模块电源一直开着,用PwrKey引脚来实现热启动
************************************************************************************************/
static bool Gprs_Restart(void)
{
    uint8 i;

    // 确认是否已经关机
    for (i = 0; i < 5; i++) {
        if ((void *)0 == Gprs_ConnectProc("AT", 0, "OK", "AT", TIME_DELAY_MS(300), TIME_DELAY_MS(100), 3, FALSE)) {
            break;
        }
        Gprs_ConnectProc("AT+QPOWD=0", 0, "OK", (void *)0, TIME_DELAY_MS(300), TIME_DELAY_MS(50), 1, FALSE);
    }
    // 通过管脚模拟按键开机
    GPRS_POWER_ON();
    GPRS_PWRKEY_UP();
    GPRS_EMERG_OFF();
    OSTimeDlyHMSM(0, 0, 1, 0);
    GPRS_PWRKEY_DOWN();
    OSTimeDlyHMSM(0, 0, 5, 0);
    GPRS_PWRKEY_UP();
    if ((void *)1 == Gprs_ConnectProc("AT", 0, "OK", "AT", TIME_DELAY_MS(300), TIME_DELAY_MS(300), 100, FALSE)) {
        return SUCCESS;
    }
    return ERROR;
}

/************************************************************************************************
* Function Name: Gprs_IpFormatConvert
* Decription   : Gprs的Ip地址格式转换
* Input        : StrPtr-指向字符串的指针, BufPtr-指向数组的指针, Dir-0:字符串转数组,1:数组转字符串
* Output       : 无
* Others       : 无
************************************************************************************************/
void Gprs_IpFormatConvert(uint8 *StrPtr, uint8 *BufPtr, uint8 Dir)
{
    uint8 i, j, val, div, flag;

    if (Dir) {
        for (i = 0; i < 4; i++) {
            val = *BufPtr++;
            div = 100;
            flag = 0;
            for (j = 0; j < 2; j++) {
                *StrPtr = val / div;
                if (1 == flag || *StrPtr > 0) {
                    flag = 1;
                    *StrPtr++ += '0';
                }
                val %= div;
                div /= 10;
            }
            *StrPtr++ = val + '0';
            *StrPtr++ = '.';
        }
        *(StrPtr - 1) = 0;
    } else {
        val = 0;
        for (i = 0, j = 0; i < strlen((char *)StrPtr) && j < 4; i++) {
            if ('.' == *(StrPtr + i)) {
                *(BufPtr + j) = val;
                j += 1;
                val = 0;
            } else if (*(StrPtr + i) >= '0' && *(StrPtr + i) <= '9') {
                val *= 10;
                val += *(StrPtr + i) - '0';
            }
        }
        *(BufPtr + j) = val;
    }
}

/************************************************************************************************
* Function Name: Gprs_TxPkgHandle
* Decription   : Gprs发送数据包之前的打包函数
* Input        : Cmd-命令类型:0x01为心跳包,0x09为数据包,BufPtr-指向数据的指针
* Output       : TRUE-正确,FALSE-错误
* Others       : 无
************************************************************************************************/
bool Gprs_TxPkgHandle(uint8 Cmd, uint8 *BufPtr)
{
    uint16 i;
    PORT_BUF_FORMAT *ptr;

    // 要在原数据的基础上增加16个字节
    ptr = (PORT_BUF_FORMAT *)BufPtr;
    if (ptr->Length + 16 > MEM_LARGE_BLOCK_LEN - 3) {
        return FALSE;
    }

    // 尾部增加0x7B结尾字符
    ptr->Buffer[ptr->Length++] = 0x7B;

    // 原数据后移15个字节
    for (i = 1; i <= ptr->Length; i++) {
        ptr->Buffer[ptr->Length + 15 - i] = ptr->Buffer[ptr->Length - i];
    }
    ptr->Length += 15;

    // 增加头部信息
    ptr->Buffer[0] = 0x7B;
    ptr->Buffer[1] = Cmd;
    ptr->Buffer[2] = (uint8)(ptr->Length >> 8);
    ptr->Buffer[3] = (uint8)(ptr->Length);

    // 增加身份识别信息
    BcdToAscii(&Concentrator.LongAddr[0], &ptr->Buffer[4], 1, 2);
    ptr->Buffer[4] = ptr->Buffer[5];
    BcdToAscii(&Concentrator.LongAddr[1], &ptr->Buffer[5], 1, 2);
    BcdToAscii(&Concentrator.LongAddr[2], &ptr->Buffer[7], 1, 2);
    BcdToAscii(&Concentrator.LongAddr[3], &ptr->Buffer[9], 1, 2);
    BcdToAscii(&Concentrator.LongAddr[4], &ptr->Buffer[11], 1, 2);
    BcdToAscii(&Concentrator.LongAddr[5], &ptr->Buffer[13], 1, 2);

    return TRUE;
}

/************************************************************************************************
* Function Name: Gprs_GetCSQ
* Decription   : Gprs模块获得信号质量
* Input        : 无
* Output       : 信号质量 0-31或99
* Others       : 无
************************************************************************************************/
uint8 Gprs_GetCSQ(void)
{
    uint8 csq, *msg, *p;

    msg = Gprs_ConnectProc("AT+CSQ", 0, "+CSQ: ", (void *)0, TIME_DELAY_MS(1000), TIME_DELAY_MS(500), 2, TRUE);
    if ((void *)0 != msg) {
        p = (uint8 *)strstr((char *)(((PORT_BUF_FORMAT *)msg)->Buffer), "+CSQ: ") + strlen("+CSQ: ");
        csq = *p++ - '0';
        if (*p >= '0' && *p <= '9') {
            csq *= 10;
            csq += *p - '0';
        }
    } else {
        csq = 99;
    }
    OSMemPut(LargeMemoryPtr, msg);

    return csq;
}

/************************************************************************************************
* Function Name: Gprs_GetIMSI
* Decription   : Gprs模块获得SIM卡的IMSI
* Input        : BufPtr-保存数据的指针
* Output       : 保存数据的长度
* Others       : 无
************************************************************************************************/
uint8 Gprs_GetIMSI(uint8 *BufPtr)
{
    uint8 i, len;
    PORT_BUF_FORMAT *portBufPtr;

    len = 0;
    portBufPtr = Gprs_ConnectProc("AT+CIMI", 0, "OK", (void *)0, TIME_DELAY_MS(1000), TIME_DELAY_MS(500), 3, TRUE);
    if ((void *)0 != portBufPtr) {
        for (i = 1; i < portBufPtr->Length; i++) {
            if (portBufPtr->Buffer[i - 1] == 0x0D && portBufPtr->Buffer[i] == 0x0A) {
                i += 1;
                break;
            }
        }
        for (; i < portBufPtr->Length; i++) {
            if (portBufPtr->Buffer[i] >= '0' && portBufPtr->Buffer[i] <= '9') {
                *(BufPtr + 1 + len++) = portBufPtr->Buffer[i];
            } else {
                break;
            }
        }
    }
    *BufPtr = len;
    OSMemPut(LargeMemoryPtr, portBufPtr);
    return len + 1;
}

/************************************************************************************************
* Function Name: Gprs_GetGMM
* Decription   : Gprs模块获得模块型号
* Input        : BufPtr-保存数据的指针
* Output       : 保存数据的长度
* Others       : 无
************************************************************************************************/
uint8 Gprs_GetGMM(uint8 *BufPtr)
{
    uint8 i, len;
    PORT_BUF_FORMAT *portBufPtr;

    len = 0;
    portBufPtr = Gprs_ConnectProc("AT+GMM", 0, "OK", (void *)0, TIME_DELAY_MS(1000), TIME_DELAY_MS(300), 5, TRUE);
    if ((void *)0 != portBufPtr) {
        for (i = 1; i < portBufPtr->Length; i++) {
            if (portBufPtr->Buffer[i - 1] == 0x0D && portBufPtr->Buffer[i] == 0x0A) {
                i += 1;
                break;
            }
        }
        for (; i < portBufPtr->Length - 1; i++) {
            if (portBufPtr->Buffer[i] == 0x0D && portBufPtr->Buffer[i + 1] == 0x0A) {
                break;
            } else {
                *(BufPtr + 1 + len++) = portBufPtr->Buffer[i];
            }
        }
    }
    *BufPtr = len;
    OSMemPut(LargeMemoryPtr, portBufPtr);
    return len + 1;
}

/************************************************************************************************
* Function Name: Gprs_Init
* Decription   : Gprs模块初始化
* Input        : 无
* Output       : 无
* Others       : 无
************************************************************************************************/
void Gprs_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

    // Gprs控制端口初始化
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;                                  // Gprs模块的运行状态PB5:H-开机,L-关机
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;                     // Gprs模块Pwr_Key(PB8)和Emerg_Off(PB9)
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;                                  // Gprs电源控制引脚PC9
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStruct);

    // Gprs控制引脚电平初始化,参数初始化,注意这时候OS还没有运行
    Gprs.Cmd = GprsPowerOn;
    Gprs.RestartCount = 0;
    Gprs.ClosePDPCount = 0;
    Gprs.ConnectCount = 0;
    Gprs.HeartbeatRetryTime = 0;
    Gprs.WorkTime = 0;
    Gprs.Online = FALSE;
    Gprs.HeartbeatInterval = Concentrator.GprsParam.HeatBeatPeriod * 10;
}

/************************************************************************************************
* Function Name: Gprs_Task
* Decription   : Gprs连接管理任务函数
* Input        : *p_arg-参数指针
* Output       : 无
* Others       : 无
************************************************************************************************/
void Gprs_Task(void *p_arg)
{
    uint8 err, *ret, *msg, *cmd, *dscIpPtr;
    uint16 i, loop, dscPort;
    PORT_BUF_FORMAT *bufPtr;
    OS_FLAGS eventFlag;

    (void)p_arg;

    while (TRUE) {
        switch (Gprs.Cmd) {
            case GprsPowerOff:
                Gprs_OutputDebugMsg(TRUE, "Gprs模块关机...\n");
                Led_FlashTime(GPRS_LED, Delay100ms, AlwaysOff, FALSE);
                Gprs_PowerOff();
                OSTimeDlyHMSM(0, 0, 1, 0);
                Gprs.Cmd = GprsPowerOn;
                break;

            case GprsPowerOn:
                Gprs_OutputDebugMsg(TRUE, "Gprs模块开机...\n");
                Led_FlashTime(GPRS_LED, Delay50ms, Delay1000ms, FALSE);
                Gprs_PowerOn();
                Gprs.Cmd = GprsConfig;
                break;

            case GprsRestart:
                // 本段代码的目的是为了防止Gprs模块频繁的重启而损坏模块
                Gprs.Online = FALSE;
                if (Gprs.WorkTime > GPRS_NORMALWORKTIME) {
                    Gprs.RestartCount = 0;
                }
                if (Gprs.RestartCount < GPRS_CONTINUERESTARTTIME &&
                    Gprs.ClosePDPCount < GPRS_CONTINUECLOSEPDPTIME) {
                    Gprs_OutputDebugMsg(TRUE, "Gprs模块重新启动,请稍候...\n");
                    Led_FlashTime(GPRS_LED, Delay50ms, Delay1000ms, TRUE);
                    Gprs_Restart();
                    Gprs.Cmd = GprsConfig;
                    Gprs.RestartCount++;
                } else {
                    Gprs_OutputDebugMsg(TRUE, "Gprs模块连续重新启动次数过多,将关机10分钟后再试,请稍候...\n");
                    Led_FlashTime(GPRS_LED, Delay100ms, AlwaysOff, TRUE);
                    Gprs_PowerOff();
                    OSTimeDlyHMSM(0, GPRS_POWERDOWNTIME, 0, 0);
                    Gprs.Cmd = GprsPowerOn;
                    Gprs.RestartCount = 0;
                    Gprs.ClosePDPCount = 0;
                    Gprs.ConnectCount = 0;
                }
                Gprs.WorkTime = 0;
                break;

            case GprsConfig:
                Gprs.Online = FALSE;
                // 改变指示灯闪烁状态-未注册到网络
                Led_FlashTime(GPRS_LED, Delay50ms, Delay800ms, FALSE);
                // 发送AT指令使模块的通信波特率与串口匹配
                Gprs_OutputDebugMsg(FALSE, "Gprs模块配置流程...");
                ret = Gprs_ConnectProc("AT", 0, "OK", (void *)0, TIME_DELAY_MS(500), TIME_DELAY_MS(500), 10, FALSE);
                if ((void *)0 == ret) {
                    Gprs_OutputDebugMsg(FALSE, "\n   未检测到Gprs模块\n");
                    Gprs.Cmd = GprsRestart;
                    break;
                }
                // 发送ATE0指令关回显
                ret = Gprs_ConnectProc("ATE0", 0, "OK", (void *)0, TIME_DELAY_MS(2000), TIME_DELAY_MS(500), 5, FALSE);
                if ((void *)0 == ret) {
                    Gprs_OutputDebugMsg(FALSE, "\n   关闭回显功能失败\n");
                    Gprs.Cmd = GprsRestart;
                    break;
                }
                // 配置通信波特率,但是不要写死,此步骤可以省略
                ret = Gprs_ConnectProc("AT+IPR=115200", 0, "OK", (void *)0, TIME_DELAY_MS(2000), TIME_DELAY_MS(500), 5, FALSE);
                if ((void *)0 == ret) {
                    Gprs_OutputDebugMsg(FALSE, "\n   设置通信波特率失败\n");
                    Gprs.Cmd = GprsRestart;
                    break;
                }
                // 获取GPRS模块型号
                Gprs.ModuleType = Unknow_Type;
                msg = Gprs_ConnectProc("AT+GMM", 0, "OK", (void *)0, TIME_DELAY_MS(1000), TIME_DELAY_MS(300), 5, TRUE);
                if ((void *)0 != msg) {
                    bufPtr = (PORT_BUF_FORMAT *)msg;
                    for (Gprs.ModuleType = Quectel_M35; Gprs.ModuleType < Unknow_Type; Gprs.ModuleType++) {
                        for (i = 0; i < bufPtr->Length - strlen(GprsModuleList[Gprs.ModuleType]); i++) {
                            if (0 == strncmp((const char *)(&bufPtr->Buffer[i]), GprsModuleList[Gprs.ModuleType], strlen(GprsModuleList[Gprs.ModuleType]))) {
                                i = 1000;
                                break;
                            }
                        }
                        if (i >= 1000) {
                            break;
                        }
                    }
                    OSMemPut(LargeMemoryPtr, msg);
                }
                if (Gprs.ModuleType >= Unknow_Type) {
                    Gprs_OutputDebugMsg(FALSE, "\n    未知的GPRS模块类型\n");
                    Gprs.Cmd = GprsRestart;
                    break;
                }
                // 查询SIM卡是否就绪
                ret = Gprs_ConnectProc("AT+CPIN?", 0, "+CPIN: READY", (void *)0, TIME_DELAY_MS(2000), TIME_DELAY_MS(2000), 5, FALSE);
                if ((void *)0 == ret) {
                    Gprs_OutputDebugMsg(FALSE, "\n   SIM卡检测异常\n");
                    Gprs.Cmd = GprsRestart;
                    break;
                }
                Gprs_OutputDebugMsg(FALSE, "成功\n侦测到GPRS模块类型为");
                Gprs_OutputDebugMsg(FALSE, (uint8 *)(GprsModuleList[Gprs.ModuleType]));
                Gprs_OutputDebugMsg(FALSE, "\n");
                Gprs.Cmd = GprsGsmStatus;
                OSTimeDlyHMSM(0, 0, 0, 500);
                break;

            case GprsGsmStatus:
                // 改变指示灯闪烁状态-未注册到网络
                Led_FlashTime(GPRS_LED, Delay50ms, Delay800ms, FALSE);
                // Gsm网络注册状态判断
                Gprs_OutputDebugMsg(FALSE, "检查网络注册状态...");
                ret = Gprs_ConnectProc("AT+CREG?", 0, "+CREG: #,1", "+CREG: #,5", TIME_DELAY_MS(5000), TIME_DELAY_MS(2000), 10, FALSE);
                /**
                if ((void *)0 == ret) {
                    Gprs_OutputDebugMsg(FALSE, "\n   网络未注册\n");
                    Gprs.Cmd = GprsRestart;
                    break;
                }
                **/
                // Gprs网络注册状态
                ret = Gprs_ConnectProc("AT+CGREG?", 0, "+CGREG: #,1", "+CGREG: #,5", TIME_DELAY_MS(5000), TIME_DELAY_MS(2000), 20, FALSE);
                if ((void *)0 == ret) {
                    Gprs_OutputDebugMsg(FALSE, "\n   Gprs网络未注册\n");
                    Gprs.Cmd = GprsRestart;
                    break;
                }
                Gprs_OutputDebugMsg(FALSE, "成功\n");
                Gprs.Cmd = Quectel_M35 == Gprs.ModuleType ? GprsDataFormat : GprsOpenPdp;
                // 改变指示灯闪烁状态(配置状态,已注册到网络)
                Led_FlashTime(GPRS_LED, Delay50ms, Delay500ms, FALSE);
                break;

            case GprsDataFormat:
                // 改变指示灯闪烁状态-未注册到网络
                Led_FlashTime(GPRS_LED, Delay50ms, Delay500ms, FALSE);
                Gprs_OutputDebugMsg(FALSE, "设置Gprs数据通讯格式...");
                // 配置前置场景
                ret = Gprs_ConnectProc("AT+QIFGCNT=0", 0, "OK", (void *)0, TIME_DELAY_MS(2000), TIME_DELAY_MS(500), 2, FALSE);
                if ((void *)0 == ret) {
                    Gprs_OutputDebugMsg(FALSE, "\n   配置前置场景失败\n");
                    Gprs.Cmd = GprsRestart;
                    break;
                }
                // 配置成Gprs连接模式
                ret = Gprs_ConnectProc("AT+QICSGP=1", 0, "OK", (void *)0, TIME_DELAY_MS(2000), TIME_DELAY_MS(500), 2, FALSE);
                if ((void *)0 == ret) {
                    Gprs_OutputDebugMsg(FALSE, "\n   设置为Gprs连接方式失败\n");
                    Gprs.Cmd = GprsRestart;
                    break;
                }
                // 接收数据时显示IP头
                ret = Gprs_ConnectProc("AT+QIHEAD=0", 0, "OK", (void *)0, TIME_DELAY_MS(2000), TIME_DELAY_MS(500), 2, FALSE);
                if ((void *)0 == ret) {
                    Gprs_OutputDebugMsg(FALSE, "\n   设置接收数据时不显示IP头失败\n");
                    Gprs.Cmd = GprsRestart;
                    break;
                }
                // 配置成接收数据时不显示IP地址和端口号
                ret = Gprs_ConnectProc("AT+QISHOWRA=0", 0, "OK", (void *)0, TIME_DELAY_MS(2000), TIME_DELAY_MS(500), 2, FALSE);
                if ((void *)0 == ret) {
                    Gprs_OutputDebugMsg(FALSE, "\n   设置接收数据时不显示IP地址失败\n");
                    Gprs.Cmd = GprsRestart;
                    break;
                }
                // 配置接收数据时IP头不显示IP协议
                ret = Gprs_ConnectProc("AT+QISHOWPT=0", 0, "OK", (void *)0, TIME_DELAY_MS(2000), TIME_DELAY_MS(500), 2, FALSE);
                if ((void *)0 == ret) {
                    Gprs_OutputDebugMsg(FALSE, "\n   设置不显示传输协议失败\n");
                    Gprs.Cmd = GprsRestart;
                    break;
                }
                // 配置TCP应用模式为非透传
                ret = Gprs_ConnectProc("AT+QIMODE=0", 0, "OK", (void *)0, TIME_DELAY_MS(2000), TIME_DELAY_MS(500), 5, FALSE);
                if ((void *)0 == ret) {
                    Gprs_OutputDebugMsg(FALSE, "\n   设置TCP为非透传模式失败\n");
                    Gprs.Cmd = GprsRestart;
                    break;
                }
                Gprs_OutputDebugMsg(FALSE, "成功\n");
                Gprs.Cmd = GprsOpenPdp;
                break;

            case GprsOpenPdp:
                // 改变指示灯闪烁状态-未注册到网络
                Led_FlashTime(GPRS_LED, Delay50ms, Delay500ms, FALSE);
                Gprs_OutputDebugMsg(FALSE, "建立TCP连接进程...");
                // 申请内存空间,如果申请失败,则每隔2秒钟再申请一次
                if ((void *)0 == (cmd = OSMemGet(SmallMemoryPtr, &err))) {
                    Gprs_OutputDebugMsg(FALSE, "\n   申请内存空间失败\n");
                    OSTimeDlyHMSM(0, 0, 2, 0);
                    break;
                }
                if (Quectel_M35 == Gprs.ModuleType) {
                    // 配置本地端口号
                    strcpy((char *)cmd, "AT+QILPORT=\"TCP\",");
                    Uint16ToString(GPRS_LOCAL_PORT, &cmd[strlen((char *)cmd)]);
                    ret = Gprs_ConnectProc((char *)cmd, 0, "OK", (void *)0, TIME_DELAY_MS(2000), TIME_DELAY_MS(500), 2, FALSE);
                    if ((void *)0 == ret) {
                        Gprs_OutputDebugMsg(FALSE, "\n   配置本地端口失败\n");
                        OSMemPut(SmallMemoryPtr, cmd);
                        Gprs.Cmd = GprsRestart;
                        break;
                    }
                    // 设置接入点Apn,Username和Password
                    if (strlen(Concentrator.GprsParam.Apn) > 0) {
                        strcpy((char *)cmd, "AT+QIREGAPP=\"");
                        strcat((char *)cmd, Concentrator.GprsParam.Apn);
                        strcat((char *)cmd, "\",\"");
                        strcat((char *)cmd, Concentrator.GprsParam.Username);
                        strcat((char *)cmd, "\",\"");
                        strcat((char *)cmd, Concentrator.GprsParam.Password);
                        strcat((char *)cmd, "\"");
                    } else {
                        strcpy((char *)cmd, "AT+QIREGAPP");
                    }
                    ret = Gprs_ConnectProc((char *)cmd, 0, "OK", (void *)0, TIME_DELAY_MS(3000), TIME_DELAY_MS(500), 3, FALSE);
                    if ((void *)0 == ret) {
                        Gprs_OutputDebugMsg(FALSE, "\n   配置接入点信息时失败\n");
                        OSMemPut(SmallMemoryPtr, cmd);
                        Gprs.Cmd = GprsRestart;
                        break;
                    }
                    // 激活移动场景,发起Gprs无线连接
                    ret = Gprs_ConnectProc("AT+QIACT", 0, "OK", (void *)0, TIME_DELAY_MS(6000), TIME_DELAY_MS(1000), 2, FALSE);
                    if ((void *)0 == ret) {
                        Gprs_OutputDebugMsg(FALSE, "\n   启动Gprs无线连接失败\n");
                        OSMemPut(SmallMemoryPtr, cmd);
                        Gprs.Cmd = GprsRestart;
                        break;
                    }
                    // 获取本地IP地址
                    msg = Gprs_ConnectProc("AT+QILOCIP", 0, "###", (void *)0, TIME_DELAY_MS(3000), TIME_DELAY_MS(1000), 3, TRUE);
                    if ((void *)0 == msg) {
                        Gprs_OutputDebugMsg(FALSE, "\n   获取本地IP地址失败\n");
                        OSMemPut(SmallMemoryPtr, cmd);
                        Gprs.Cmd = GprsClosePdp;
                        break;
                    }
                    bufPtr = (PORT_BUF_FORMAT *)msg;
                    bufPtr->Buffer[bufPtr->Length] = 0;
                    Gprs_IpFormatConvert(bufPtr->Buffer, Gprs.LocalIp, 0);
                    OSMemPut(LargeMemoryPtr, msg);
                } else {
                    // 设置接入点Apn,Username和Password
                    if (strlen(Concentrator.GprsParam.Apn) > 0) {
                        strcpy((char *)cmd, "AT+QICSGP=1,1,\"");
                        strcat((char *)cmd, Concentrator.GprsParam.Apn);
                        strcat((char *)cmd, "\",\"");
                        strcat((char *)cmd, Concentrator.GprsParam.Username);
                        strcat((char *)cmd, "\",\"");
                        strcat((char *)cmd, Concentrator.GprsParam.Password);
                        strcat((char *)cmd, "\",1");
                    } else {
                        strcpy((char *)cmd, "AT+QICSGP=1,1,\"\",\"\",\"\",1");
                    }
                    ret = Gprs_ConnectProc((char *)cmd, 0, "OK", (void *)0, TIME_DELAY_MS(3000), TIME_DELAY_MS(500), 3, FALSE);
                    if ((void *)0 == ret) {
                        Gprs_OutputDebugMsg(FALSE, "\n   配置接入点信息时失败\n");
                        OSMemPut(SmallMemoryPtr, cmd);
                        Gprs.Cmd = GprsRestart;
                        break;
                    }
                    // 激活移动场景,发起Gprs无线连接
                    for (loop = 0; loop < 5; loop++) {
                        msg = Gprs_ConnectProc("AT+QIACT?", 0, "+QIACT: ", (void *)0, TIME_DELAY_MS(3000), TIME_DELAY_MS(500), 3, TRUE);
                        if ((void *)0 != msg) {
                            bufPtr = (PORT_BUF_FORMAT *)msg;
                            for (i = bufPtr->Length - 1; i > 0; i--) {
                                if (bufPtr->Buffer[i] == 0x22 && bufPtr->Buffer[i - 1] == 0x2C) {
                                    Gprs_IpFormatConvert(&bufPtr->Buffer[i], Gprs.LocalIp, 0);
                                    break;
                                }
                                if (bufPtr->Buffer[i] == 0x22) {
                                    bufPtr->Buffer[i] = 0;
                                }
                            }
                            OSMemPut(LargeMemoryPtr, msg);
                            if (i > 0) {
                                break;
                            }
                        }
                        Gprs_ConnectProc("AT+QIDEACT=1", 0, "OK", (void *)0, TIME_DELAY_MS(2000), TIME_DELAY_MS(200), 3, FALSE);
                        OSTimeDlyHMSM(0, 0, 1, 0);
                        Gprs_ConnectProc("AT+QIACT=1", 0, "OK", (void *)0, TIME_DELAY_MS(50000), TIME_DELAY_MS(200), 1, FALSE);
                    }
                    if (loop >= 5) {
                        Gprs_OutputDebugMsg(FALSE, "\n   启动Gprs无线连接失败\n");
                        OSMemPut(SmallMemoryPtr, cmd);
                        Gprs.Cmd = GprsRestart;
                        break;
                    }
                }
                if (strlen(Concentrator.GprsParam.Apn) > 0) {
                    strcpy((char *)cmd, "成功\n   接入点信息 APN:\"");
                    strcat((char *)cmd, Concentrator.GprsParam.Apn);
                    strcat((char *)cmd, "\", 用户名:\"");
                    strcat((char *)cmd, Concentrator.GprsParam.Username);
                    strcat((char *)cmd, "\", 密码:\"");
                    strcat((char *)cmd, Concentrator.GprsParam.Password);
                    strcat((char *)cmd, "\"\n");
                    Gprs_OutputDebugMsg(FALSE, cmd);
                } else {
                    Gprs_OutputDebugMsg(FALSE, "成功\n   未设置接入点信息!\n");
                }
                OSMemPut(SmallMemoryPtr, cmd);
                Gprs.Cmd= GprsOpenTcp;
                break;

            case GprsOpenTcp:
                // 改变指示灯闪烁状态-未注册到网络
                Led_FlashTime(GPRS_LED, Delay50ms, Delay200ms, FALSE);
                Gprs_OutputDebugMsg(FALSE, "建立服务器连接...");

                // 申请内存空间,如果申请失败,则每隔2秒钟再申请一次
                if ((void *)0 == (cmd = OSMemGet(SmallMemoryPtr, &err))) {
                    Gprs_OutputDebugMsg(FALSE, "\n   申请内存空间时失败\n");
                    OSTimeDlyHMSM(0, 0, 2, 0);
                    break;
                }
                // 建立Tcp连接
                if (Gprs.ConnectCount < GPRS_BACKUP_IP_CONNECT) {
                    Gprs.ConnectCount += 1;
                    dscIpPtr = Concentrator.GprsParam.PriorDscIp;
                    dscPort = Concentrator.GprsParam.PriorDscPort;
                } else {
                    Gprs.ConnectCount = 0;
                    dscIpPtr = Concentrator.GprsParam.BackupDscIp;
                    dscPort = Concentrator.GprsParam.BackupDscPort;
                }
                if (Quectel_M35 == Gprs.ModuleType) {
                    strcpy((char *)cmd, "AT+QIOPEN=\"TCP\",\"");
                    Gprs_IpFormatConvert(&cmd[strlen((char *)cmd)], dscIpPtr, 1);
                    strcat((char *)cmd, "\",\"");
                    Uint16ToString(dscPort, &cmd[strlen((char *)cmd)]);
                    strcat((char *)cmd, "\"");
                    ret = Gprs_ConnectProc((char *)cmd, 0, "CONNECT OK", "ALREADY CONNECT", TIME_DELAY_MS(5000), TIME_DELAY_MS(3500), 5, FALSE);
                } else {
                    strcpy((char *)cmd, "AT+QIOPEN=1,0,\"TCP\",\"");
                    Gprs_IpFormatConvert(&cmd[strlen((char *)cmd)], dscIpPtr, 1);
                    strcat((char *)cmd, "\",");
                    Uint16ToString(dscPort, &cmd[strlen((char *)cmd)]);
                    strcat((char *)cmd, ",0,1");
                    ret = Gprs_ConnectProc((char *)cmd, 0, "+QIOPEN: 0,0", "+QIOPEN: 0,563", TIME_DELAY_MS(10000), TIME_DELAY_MS(3500), 5, FALSE);
                }
                strcpy((char *)cmd, "   服务器信息 IP:");
                Gprs_IpFormatConvert(&cmd[strlen((char *)cmd)], dscIpPtr, 1);
                strcat((char *)cmd, ", Port:");
                Uint16ToString(dscPort, &cmd[strlen((char *)cmd)]);
                strcat((char *)cmd, "\n");
                if ((void *)0 == ret) {
                    Gprs_OutputDebugMsg(FALSE, "失败\n");
                    Gprs_OutputDebugMsg(FALSE, cmd);
                    OSMemPut(SmallMemoryPtr, cmd);
                    Gprs.Cmd = GprsClosePdp;
                    break;
                }
                Gprs.ClosePDPCount = 0;
                Gprs.ConnectCount = 0;
                Gprs_OutputDebugMsg(FALSE, "成功\n");
                Gprs_OutputDebugMsg(FALSE, cmd);
                OSMemPut(SmallMemoryPtr, cmd);
                Gprs_OutputDebugMsg(FALSE, "-------------------------------------------------\n");
                // 发送心跳包检查是否已经连接上
                Gprs.HeartbeatRetryTime = 0;
                Gprs.Cmd = GprsCheckConnect;
                break;

            case GprsClosePdp:
                Gprs.Online = FALSE;
                // 改变指示灯闪烁状态-未联网
                Led_FlashTime(GPRS_LED, Delay50ms, Delay1000ms, TRUE);
                // 关闭PDP场景
                Gprs_OutputDebugMsg(FALSE, "关闭PDP场景...");
                if (Quectel_M35 == Gprs.ModuleType) {
                    ret = Gprs_ConnectProc("AT+QIDEACT", 0, "DEACT OK", (void *)0, TIME_DELAY_MS(60000), TIME_DELAY_MS(500), 1, FALSE);
                } else {
                    ret = Gprs_ConnectProc("AT+QIDEACT=1", 0, "OK", (void *)0, TIME_DELAY_MS(40000), TIME_DELAY_MS(500), 1, FALSE);
                }
                if ((void *)0 == ret) {
                    Gprs_OutputDebugMsg(FALSE, "失败\n");
                    Gprs.Cmd = GprsRestart;
                    break;
                }
                // 如果连续发生5次则重启模块
                Gprs_OutputDebugMsg(FALSE, "成功\n");
                if (Gprs.ClosePDPCount < GPRS_CONTINUECLOSEPDPTIME) {
                    Gprs.ClosePDPCount++;
                    Gprs.Cmd = GprsConfig;
                } else {
                    Gprs.Cmd = GprsRestart;
                }
                break;

            case GprsCloseTcp:
                Gprs.Online = FALSE;
                // 改变指示灯闪烁状态-未联网
                Led_FlashTime(GPRS_LED, Delay50ms, Delay1000ms, TRUE);
                // 关闭TCP连接
                Gprs_OutputDebugMsg(FALSE, "关闭TCP连接...");
                if (Quectel_M35 == Gprs.ModuleType) {
                    ret = Gprs_ConnectProc("AT+QISTATE", 0, "IP CLOSE", "PDP DEACT", TIME_DELAY_MS(1000), TIME_DELAY_MS(500), 1, FALSE);
                    if ((void *)0 == ret) {
                        ret = Gprs_ConnectProc("AT+QICLOSE", 0, "CLOSE OK", (void *)0, TIME_DELAY_MS(30000), TIME_DELAY_MS(500), 1, FALSE);
                        if ((void *)0 == ret) {
                            Gprs_OutputDebugMsg(FALSE, "失败\n");
                            Gprs.Cmd = GprsRestart;
                            break;
                        }
                    }
                } else {
                    ret = Gprs_ConnectProc("AT+QISTATE=1,0", 0, "+QISTATE:", (void *)0, TIME_DELAY_MS(1500), TIME_DELAY_MS(500), 1, FALSE);
                    if ((void *)0 != ret) {
                        ret = Gprs_ConnectProc("AT+QICLOSE=0", 0, "OK", (void *)0, TIME_DELAY_MS(1500), TIME_DELAY_MS(500), 1, FALSE);
                        if ((void *)0 == ret) {
                            Gprs_OutputDebugMsg(FALSE, "失败\n");
                            Gprs.Cmd = GprsRestart;
                            break;
                        }
                    }
                }
                Gprs_OutputDebugMsg(FALSE, "成功\n");
                Gprs.Cmd = GprsOpenTcp;
                break;

            case GprsCheckConnect:
                // 申请内存空间,如果申请失败,则每隔1秒钟再申请一次
                if ((void *)0 == (cmd = OSMemGet(SmallMemoryPtr, &err))) {
                    OSTimeDlyHMSM(0, 0, 1, 0);
                    break;
                }
                // 发送心跳包数据
                if (Quectel_M35 == Gprs.ModuleType) {
                    Gprs_ConnectProc("AT+QISEND=22", 0, ">", (void *)0, TIME_DELAY_MS(3000), TIME_DELAY_MS(500), 1, FALSE);
                } else {
                    Gprs_ConnectProc("AT+QISEND=0,22", 0, ">", (void *)0, TIME_DELAY_MS(3000), TIME_DELAY_MS(500), 1, FALSE);
                }
                Gprs_OutputDebugMsg(TRUE, ">>>心跳包");
                eventFlag &= ~FLAG_HEATBEAT_TIMER;
                bufPtr = (PORT_BUF_FORMAT *)cmd;
                bufPtr->Length = 0;
                bufPtr->Property.PortNo = Uart_Gprs;
                bufPtr->Buffer[bufPtr->Length++] = Gprs.LocalIp[0];
                bufPtr->Buffer[bufPtr->Length++] = Gprs.LocalIp[1];
                bufPtr->Buffer[bufPtr->Length++] = Gprs.LocalIp[2];
                bufPtr->Buffer[bufPtr->Length++] = Gprs.LocalIp[3];
                bufPtr->Buffer[bufPtr->Length++] = (uint8)(GPRS_LOCAL_PORT >> 8);
                bufPtr->Buffer[bufPtr->Length++] = (uint8)GPRS_LOCAL_PORT;
                Gprs_TxPkgHandle(GPRS_HEATBEAT_CMD, cmd);
                ret = Gprs_ConnectProc((char *)(bufPtr->Buffer), bufPtr->Length, "SEND OK", (void *)0, TIME_DELAY_MS(5000), TIME_DELAY_MS(500), 1, FALSE);
                if ((void *)0 == ret) {
                    OSMemPut(SmallMemoryPtr, cmd);
                    Gprs.Cmd = GprsCloseTcp;
                    Gprs_OutputDebugMsg(FALSE, "-失败<<<\n");
                    break;
                }
                // 等待服务器的心跳包应答数据
                for (loop = 0; loop < 5; loop++) {
                    msg = OSMboxPend(SerialPort.Port[Uart_Gprs].MboxRx, TIME_DELAY_MS(5000), &err);
                    if ((void *)0 == msg) {
                        if (++Gprs.HeartbeatRetryTime > 3) {
                            Gprs.Cmd = GprsCloseTcp;
                        }
                        Gprs_OutputDebugMsg(FALSE, "-失败<<<\n");
                        break;
                    }
                    bufPtr = (PORT_BUF_FORMAT *)msg;
                    for (i = 0; i < LONG_ADDR_SIZE; i++) {
                        BcdToAscii(&Concentrator.LongAddr[i], &cmd[i * 2], 1, 2);
                    }
                    if (bufPtr->Length > LONG_ADDR_SIZE * 2) {
                        for (i = 0; i < bufPtr->Length - LONG_ADDR_SIZE * 2 + 1; i++) {
                            if (0 == memcmp(&cmd[1], &bufPtr->Buffer[i], LONG_ADDR_SIZE * 2 - 1)) {
                                Gprs.HeartbeatRetryTime = 0;
                                Gprs.Cmd = GprsConnectIdleStatus;
                                Gprs.HeartbeatInterval = Concentrator.GprsParam.HeatBeatPeriod * 10;
                                if (FALSE == Gprs.Online) {
                                    DataUploadTimer = 50;
                                    Gprs.Online = TRUE;
                                }
                                loop = 200;
                                Gprs_OutputDebugMsg(FALSE, "-成功<<<\n");
                                break;
                            }
                        }
                    }
                    OSMemPut(LargeMemoryPtr, msg);
                }
                OSMemPut(SmallMemoryPtr, cmd);
                if (loop == 5) {
                    Gprs.Cmd = GprsCloseTcp;
                    Gprs_OutputDebugMsg(FALSE, "-失败<<<\n");
                }
                break;

            case GprsLogoffConnect:
                // 申请内存空间,如果申请失败,则每隔1秒钟再申请一次
                if ((void *)0 == (cmd = OSMemGet(SmallMemoryPtr, &err))) {
                    OSTimeDlyHMSM(0, 0, 1, 0);
                    break;
                }
                // 改变指示灯闪烁状态-注销
                Led_FlashTime(GPRS_LED, Delay1000ms, AlwaysOff, FALSE);
                // 发送注销指令
                if (Quectel_M35 == Gprs.ModuleType) {
                    Gprs_ConnectProc("AT+QISEND=16", 0, ">", (void *)0, TIME_DELAY_MS(3000), TIME_DELAY_MS(500), 1, FALSE);
                } else {
                    Gprs_ConnectProc("AT+QISEND=0,16", 0, ">", (void *)0, TIME_DELAY_MS(3000), TIME_DELAY_MS(500), 1, FALSE);
                }
                Gprs_OutputDebugMsg(TRUE, ">>>设备重启,连接注销<<<\n");
                bufPtr = (PORT_BUF_FORMAT *)cmd;
                bufPtr->Length = 0;
                bufPtr->Property.PortNo = Uart_Gprs;
                Gprs_TxPkgHandle(GPRS_LOGOFF_CMD, cmd);
                Gprs_ConnectProc((char *)(bufPtr->Buffer), bufPtr->Length, "SEND OK", (void *)0, TIME_DELAY_MS(5000), TIME_DELAY_MS(500), 1, FALSE);
                OSMemPut(SmallMemoryPtr, cmd);
				Gprs.Cmd = GprsPowerOff;
                break;

            case GprsConnectIdleStatus:
                // 改变指示灯闪烁状态-已经联网
                Led_FlashTime(GPRS_LED, AlwaysOn, Delay1000ms, FALSE);
                eventFlag = OSFlagPend(GlobalEventFlag, (OS_FLAGS)GPRS_FLAG_FILTER, (OS_FLAG_WAIT_SET_ANY | OS_FLAG_CONSUME), TIME_DELAY_MS(60000), &err);
                if (eventFlag & FLAG_UART_GPRS_RX) {
                    Gprs.Cmd = GprsReceiveData;
                } else if (eventFlag & FLAG_GPRS_TX) {
                    Gprs.Cmd = GprsTransmitData;
                } else if (eventFlag & FLAG_HEATBEAT_TIMER) {
                    Gprs.Cmd = GprsCheckConnect;
                } else if (eventFlag & FLAG_LOGOFF_EVENT) {
                    Gprs.Cmd = GprsLogoffConnect;
                }
                break;

            case GprsReceiveData:
                // 改变指示灯闪烁状态-数据通讯
                Led_FlashTime(GPRS_LED, AlwaysOn, Delay50ms, TRUE);
                msg = OSMboxAccept(SerialPort.Port[Uart_Gprs].MboxRx);
                if ((void *)0 != msg) {
                    // 对接收数据的处理
                    bufPtr = (PORT_BUF_FORMAT *)msg;
                    if (FALSE == bufPtr->Property.FilterDone && bufPtr->Length > strlen(GPRS_IP_CLOSE)) {
                        for (loop = 0; loop < bufPtr->Length - strlen(GPRS_IP_CLOSE); loop++) {
                            if (0 == memcmp((char *)(&bufPtr->Buffer[loop]), GPRS_IP_CLOSE, strlen(GPRS_IP_CLOSE))) {
                                Gprs.Cmd = GprsCloseTcp;
                                break;
                            }
                        }
                    }
                    if (GprsCloseTcp == Gprs.Cmd) {
                        OSMemPut(LargeMemoryPtr, msg);
                        break;
                    }
                    // 向数据处理任务发送数据
                    Gprs.Online = TRUE;
                    Gprs.HeartbeatInterval = Concentrator.GprsParam.HeatBeatPeriod * 10;
                    loop = 3;
                    while (loop && OS_ERR_NONE != OSMboxPost(Gprs.MboxRx, msg)) {
                        loop -= 1;
                        OSTimeDlyHMSM(0, 0, 0, 100);
                    }
                    if (0 == loop) {
                        OSMemPut(LargeMemoryPtr, msg);
                    } else {
                        OSFlagPost(GlobalEventFlag, FLAG_GPRS_RX, OS_FLAG_SET, &err);
                    }
                }
                eventFlag &= ~FLAG_UART_GPRS_RX;
                Gprs.Cmd = (eventFlag & FLAG_GPRS_TX) ? GprsTransmitData : GprsConnectIdleStatus;
                break;

            case GprsTransmitData:
                // 改变指示灯闪烁状态-数据通讯
                Led_FlashTime(GPRS_LED, AlwaysOn, Delay50ms, TRUE);
                // 申请内存空间,如果申请失败,则每隔500毫秒钟再申请一次
                if ((void *)0 == (cmd = OSMemGet(SmallMemoryPtr, &err))) {
                    OSTimeDlyHMSM(0, 0, 0, 500);
                    break;
                }
                msg = OSMboxAccept(Gprs.MboxTx);
                if ((void *)0 != msg) {
                    // 数据打包成功则发送
                    if (TRUE== Gprs_TxPkgHandle(GPRS_DATAPKG_CMD, msg)) {
                        bufPtr = (PORT_BUF_FORMAT *)msg;
                        if (Quectel_M35 == Gprs.ModuleType) {
                            strcpy((char *)cmd, "AT+QISEND=");
                        } else {
                            strcpy((char *)cmd, "AT+QISEND=0,");
                        }
                        Uint16ToString(bufPtr->Length, &cmd[strlen((char *)cmd)]);
                        Gprs_ConnectProc((char *)cmd, 0, ">", (void *)0, TIME_DELAY_MS(3000), TIME_DELAY_MS(500), 1, FALSE);
                        ret = Gprs_ConnectProc((char *)(bufPtr->Buffer), bufPtr->Length, "SEND OK", (void *)0, TIME_DELAY_MS(5000), TIME_DELAY_MS(20), 1, FALSE);
                        Gprs.HeartbeatInterval = 20;
                        OSMemPut(LargeMemoryPtr, msg);
                        if ((void *)0 == ret) {
                            OSMemPut(SmallMemoryPtr, cmd);
                            eventFlag &= ~FLAG_GPRS_TX;
                            Gprs.Cmd = (eventFlag & FLAG_UART_GPRS_RX) ? GprsReceiveData : GprsCloseTcp;
                            break;
                        }
                    }
                }
                OSMemPut(SmallMemoryPtr, cmd);
                eventFlag &= ~FLAG_GPRS_TX;
                Gprs.Cmd = (eventFlag & FLAG_UART_GPRS_RX) ? GprsReceiveData : GprsConnectIdleStatus;
                break;

            default:
                break;

        }
        OSTimeDlyHMSM(0, 0, 0, 5);
    };
}

/***************************************End of file*********************************************/

