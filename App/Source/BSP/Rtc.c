/************************************************************************************************
*                                   SRWF-6009
*    (c) Copyright 2015, Software Department, Sunray Technology Co.Ltd
*                               All Rights Reserved
*
* FileName     : Rtc.c
* Description  :
* Version      :
* Function List:
*------------------------------Revision History--------------------------------------------------
* No.   Version     Date            Revised By      Item            Description
* 1     V1.0        07/31/2015      Zhangxp         SRWF-6009       Original Version
************************************************************************************************/

#define RTC_GLOBALS

/************************************************************************************************
*                             Include File Section
************************************************************************************************/
#include "Stm32f10x_conf.h"
#include "Rtc.h"

/************************************************************************************************
*                        Global Variable Declare Section
************************************************************************************************/
// 错误校时的计数器
static uint8 Rtc_ErrTimes = 0;

// 平年的月份日期表
const uint8 Rtc_MonthTab[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

/************************************************************************************************
*                           Prototype Declare Section
************************************************************************************************/

/************************************************************************************************
*                           Function Declare Section
************************************************************************************************/

/************************************************************************************************
* Function Name: Rtc_IsLeapYear
* Decription   : 判断一个年是否为闰年
* Input        : Year-年
* Output       : 无
* Others       : 无
************************************************************************************************/
static bool Rtc_IsLeapYear(uint16 Year)
{
    if (Year % 400 == 0 || Year % 4 == 0 && Year % 100 != 0) {
        return TRUE;
    } else {
        return FALSE;
    }
}

/************************************************************************************************
* Function Name: Rtc_LastDayofMonth
* Decription   : 计算出一个月的最后一天是几号
* Input        : Year-年,Month-月
* Output       : 日期
* Others       : 都是BCD格式
************************************************************************************************/
uint8 Rtc_LastDayofMonth(uint16 Year, uint8 Month)
{
    uint8 a, b;

    a = (uint8)Year;
    b = (uint8)(Year >> 8);
    Year = (a >> 4 & 0x0F) * 1000 + (a & 0x0F) * 100 + (b >> 4 & 0x0F) * 10 + (b & 0x0F);
    Month = BcdToBin(Month);
    if (2 == Month) {
        return (Rtc_IsLeapYear(Year) ? 0x29 : 0x28);
    }
    if (Month - 1 < sizeof(Rtc_MonthTab)) {
        return (BinToBcd(Rtc_MonthTab[Month - 1]));
    }
    return 0x30;
}


/************************************************************************************************
* Function Name: Rtc_Set
* Decription   : 设置当前的时间,校时的时间和当前时间的差值不能超过一天,如果超过一天,需连续校时
*                RTC_MAX_ERRTIME次数后才能成功
* Input        : Time-时间的结构体指针, Format-写入的格式
* Output       : 成功或者失败
* Others       : 设置时间的时候不需要设置星期
************************************************************************************************/
bool Rtc_Set(RTC_TIME Time, RTC_FORMAT Format)
{
    uint8 day;
    uint16 i;
    uint32 temp, curTemp, dValue;

    // 首先将格式转换成HEX格式
    if (Format_Bcd == Format) {
        Time.Year = BcdToBin(Time.Year / 256) + BcdToBin(Time.Year) * 100;
        Time.Month = BcdToBin(Time.Month);
        Time.Day = BcdToBin(Time.Day);
        Time.Hour = BcdToBin(Time.Hour);
        Time.Minute = BcdToBin(Time.Minute);
        Time.Second = BcdToBin(Time.Second);
    }

    // 检查校时时间是否正确
    day = Rtc_MonthTab[Time.Month - 1];
    if (2 == Time.Month && TRUE == Rtc_IsLeapYear(Time.Year)) {
        day += 1;
    }
    if (Time.Year < RTC_MINYEAR || Time.Year > RTC_MAXYEAR ||
        Time.Month == 0 || Time.Month > 12 ||
        Time.Day == 0 || Time.Day > day ||
        Time.Hour >= 24 || Time.Minute >= 60 || Time.Second >= 60) {
        return ERROR;
    }

    // 计算年份的天数 闰年的天数 平年的天数
    temp = 0;
    for (i = RTC_MINYEAR; i < Time.Year; i++) {
        temp += TRUE == Rtc_IsLeapYear(i) ? 366 : 365;
    }

    // 计算月份的天数
    for (i = 0; i < Time.Month - 1; i++) {
        temp += Rtc_MonthTab[i];
        if (i == 1 && TRUE == Rtc_IsLeapYear(Time.Year)) {
            temp += 1;
        }
    }

    // 计算总天数并计算出总秒数
    temp += Time.Day - 1;
    temp = temp * 24 * 60 * 60;

    // 读出当前时间的总秒数
    curTemp = RTC_GetCounter();

    // 计算出差值并判断是否应该校时
    dValue = (curTemp >= temp) ? (curTemp - temp) : (temp - curTemp);
    dValue /= 24 * 60 * 60;
    if (dValue && Rtc_ErrTimes < RTC_MAX_ERRTIME) {
        Rtc_ErrTimes += 1;
        return ERROR;
    }
    Rtc_ErrTimes = 0;

    // 计算时 分 秒
    temp += (uint32)Time.Hour * 60 * 60;
    temp += (uint32)Time.Minute * 60;
    temp += Time.Second;

    // 设置时钟,使能电源时钟,使能备份时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

    // 使能RTC和后备寄存器访问权限
    PWR_BackupAccessCmd(ENABLE);

    // 设置RTC计数器的值
    RTC_SetCounter(temp);

    // 等待对RTC寄存器的写操作完成
    RTC_WaitForLastTask();

    Rtc_Init();

    return SUCCESS;
}

/************************************************************************************************
* Function Name: Rtc_Get
* Decription   : 得到当前的时间
* Input        : Time-时间的结构体指针, Format-读出的格式
* Output       : 无
* Others       : 无
************************************************************************************************/
void Rtc_Get(RTC_TIME *Time, RTC_FORMAT Format)
{
    uint32 temp, secCount;

    // 获取秒计数器的值,得到天数
    secCount = RTC_GetCounter();
    temp = secCount / (24 * 60 * 60);

    // 计算得到星期(加4是因为1970年1月1日是星期四)
    Time->Week = (temp + RTC_STARTWEEK) % 7;

    // 计算得到年
    Time->Year = RTC_MINYEAR;
    while (temp >= 365) {
        if (Rtc_IsLeapYear(Time->Year)) {
            if (temp >= 366) {
                temp -= 366;
            } else {
                break;
            }
        } else {
            temp -= 365;
        }
        Time->Year += 1;
    }

    // 计算得到月
    Time->Month = 0;
    while (temp >= 28) {
        if (Time->Month == 1 && TRUE == Rtc_IsLeapYear(Time->Year)) {
            if (temp >= 29) {
                temp -= 29;
            } else {
                break;
            }
        } else {
            if (temp >= Rtc_MonthTab[Time->Month]) {
                temp -= Rtc_MonthTab[Time->Month];
            } else {
                break;
            }
        }
        Time->Month += 1;
    }
    Time->Month += 1;

    // 计算得到日
    Time->Day = temp + 1;

    // 计算得到时 分 秒
    temp = secCount % (24 * 60 * 60);
    Time->Hour = temp / (60 * 60);
    temp %= (60 * 60);
    Time->Minute = temp / 60;
    Time->Second = temp % 60;

    // 格式转换
    if (Format_Bcd == Format) {
        Time->Year = BinToBcd(Time->Year / 100) | BinToBcd(Time->Year % 100) << 8;
        Time->Month = BinToBcd(Time->Month);
        Time->Day = BinToBcd(Time->Day);
        Time->Hour = BinToBcd(Time->Hour);
        Time->Minute = BinToBcd(Time->Minute);
        Time->Second = BinToBcd(Time->Second);
    }
}

/************************************************************************************************
* Function Name: Rtc_Init
* Decription   : Rtc功能初始化
* Input        : 无
* Output       : 无
* Others       : 无
************************************************************************************************/
void Rtc_Init(void)
{
    // 需要增加这三条命令,否则会当在RTC_WaitForSynchro
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
    PWR_BackupAccessCmd(ENABLE);
    RCC_RTCCLKCmd(ENABLE);

    // 如果系统不在待机模式下恢复并且不是由NRST引脚引起的复位则:清除待机位,使能RTC,并确保已经同步
    if (PWR_GetFlagStatus(PWR_FLAG_SB) != RESET && RCC_GetFlagStatus(RCC_FLAG_PINRST) == RESET) {
        // 不需要配置RTC,因为在从待机状态下恢复的时候,RTC配置都没有变
        PWR_ClearFlag(PWR_FLAG_SB);
        RCC_RTCCLKCmd(ENABLE);
        RTC_WaitForSynchro();
    } else {
        // RTC时钟源配置
        if (PWR_GetFlagStatus(PWR_FLAG_SB) != RESET) {
            PWR_ClearFlag(PWR_FLAG_SB);
        }
        if (RCC_GetFlagStatus(RCC_FLAG_PORRST) != RESET) {          // 上电复位时,若集中器ID号非全F,则给SRWF-1028模块发送写ID命令

        }
        RCC_ClearFlag();                                            // 清除标志

        if (BKP_ReadBackupRegister(BKP_DR1) != 0x5555) {            // 读取后备寄存器1的数据
            BKP_DeInit();                                           // 复位备份寄存区域的数据
        }

#if 1
        // Rtc时钟源配置,外部低速时钟源,并等待其稳定
        RCC_LSEConfig(RCC_LSE_ON);
        while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET) {
        }
        RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
#else
        // 如果使用的是内部低速时钟源
        RCC_LSICmd(ENABLE);
        while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET) {
        }
        RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
#endif
        RCC_RTCCLKCmd(ENABLE);                                      // 使能RTC时钟

        RTC_WaitForSynchro();                                       // 等待RTC APB寄存器同步完成
        RTC_WaitForLastTask();                                      // 等待最近一次对RTC寄存器的写操作完成

        RTC_ITConfig(RTC_IT_ALR, DISABLE);                          // 允许RTC报警中断 zhangxp-暂时禁止,没发现什么鸟用
        RTC_WaitForLastTask();                                      // 等待最后一条写指令完成

        RTC_SetPrescaler(32767);                                    // 设置RTC时间基准为1秒
        RTC_WaitForLastTask();                                      // 等待最后一条写命令

        BKP_WriteBackupRegister(BKP_DR1, 0x5555);                   // 写入RTC后备寄存器1 0x5555 表示时钟已经配置过了

        RCC_ClearFlag();
    }
}

/***************************************End of file*********************************************/

