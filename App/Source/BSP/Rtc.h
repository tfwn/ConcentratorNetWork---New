/************************************************************************************************
*                                   SRWF-6009
*    (c) Copyright 2015, Software Department, Sunray Technology Co.Ltd
*                               All Rights Reserved
*
* FileName     : Rtc.h
* Description  :
* Version      :
* Function List:
*------------------------------Revision History--------------------------------------------------
* No.   Version     Date            Revised By      Item            Description
* 1     V1.0        07/31/2015      Zhangxp         SRWF-6009       Original Version
************************************************************************************************/

#ifndef  RTC_H
#define  RTC_H

#ifdef   RTC_GLOBALS
#define  RTC_EXT
#else
#define  RTC_EXT  extern
#endif

/************************************************************************************************
*                               Pubilc Macro Define Section
************************************************************************************************/
// Rtc的合法时间段定义
#define RTC_MINYEAR                     1970                // 实时时钟开始年数
#define RTC_MAXYEAR                     2099                // 实时时钟终止年数
#define RTC_STARTWEEK                   4                   // 1970年1月1日是星期四

#define RTC_MAX_ERRTIME                 2                   // 错误校时的最大次数

/************************************************************************************************
*                                  Struct Define Section
************************************************************************************************/
// Rtc变量的结构体定义
typedef  struct {
    uint16 Year;
    uint8 Month;
    uint8 Day;
    uint8 Hour;
    uint8 Minute;
    uint8 Second;
    uint8 Week;                                             // 0代表星期日
} RTC_TIME;

/************************************************************************************************
*                                   Enum Define Section
************************************************************************************************/
// 读写RTC时的格式
typedef enum {
    Format_Bcd,                                             // Bcd格式读写
    Format_Bin,                                             // Bin格式读写
} RTC_FORMAT;
/************************************************************************************************
*                            Variable Declare Section
************************************************************************************************/

/************************************************************************************************
*                            Function Declare Section
************************************************************************************************/
RTC_EXT uint8 Rtc_LastDayofMonth(uint16 Year, uint8 Month);
RTC_EXT bool Rtc_Set(RTC_TIME Time, RTC_FORMAT Format);
RTC_EXT void Rtc_Get(RTC_TIME *Time, RTC_FORMAT Format);
RTC_EXT void Rtc_Init(void);

#endif
/***************************************End of file*********************************************/

