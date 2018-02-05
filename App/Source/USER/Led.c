/************************************************************************************************
*                                   SRWF-6009
*    (c) Copyright 2015, Software Department, Sunray Technology Co.Ltd
*                               All Rights Reserved
*
* FileName     : Led.c
* Description  :
* Version      :
* Function List:
*------------------------------Revision History--------------------------------------------------
* No.   Version     Date            Revised By      Item            Description
* 1     V1.0        08/13/2015      Zhangxp         SRWF-6009       Original Version
************************************************************************************************/

#define LED_GLOBALS

/************************************************************************************************
*                             Include File Section
************************************************************************************************/
#include "Stm32f10x_conf.h"
#include "ucos_ii.h"
#include "Led.h"
#include "DataHandle.h"
#include "Database.h"

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
* Function Name: Led_OnOffCtrl
* Decription   : Led灯的开关控制函数
* Input        : Led-Led的索引,OnOff-开或者关
* Output       : 无
* Others       : 无
************************************************************************************************/
static void Led_OnOffCtrl(LED_TYPE Led, bool OnOff)
{
    if (Led1 == Led) {
        if (OnOff) {
            GPIO_SetBits(GPIOD, GPIO_Pin_0);
        } else {
            GPIO_ResetBits(GPIOD, GPIO_Pin_0);
        }
    } else if (Led2 == Led) {
        if (OnOff) {
            GPIO_SetBits(GPIOD, GPIO_Pin_1);
        } else {
            GPIO_ResetBits(GPIOD, GPIO_Pin_1);
        }
    }
}

/************************************************************************************************
* Function Name: Led_Init
* Decription   : Led初始化GPIO端口
* Input        : 无
* Output       : 无
* Others       : 无
************************************************************************************************/
void Led_Init(void)
{
    bool onOff = ON;
    uint8 size1, size2, size3;
    uint32 delayTime;
    GPIO_InitTypeDef GPIO_InitStructure;

    //OSC_IN、OSC_OUT重映射为IO口
    GPIO_PinRemapConfig(GPIO_Remap_PD01, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // 检查Flash存储单元的大小必须为偶数,如果不是偶数则双闪LED提示
    size1 = sizeof(CUSTOM_ROUTE_INFO);
    size2 = sizeof(SUBNODE_INFO);
    size3 = sizeof(CONCENTRATOR_INFO);
    if ((size1 & 0x01) || (size2 & 0x01) || (size3 & 0x01)) {
        onOff = ~onOff;
        while (TRUE) {
            onOff = !onOff;
            delayTime = 100000;
            while (delayTime--) {
                Led_OnOffCtrl(Led1, onOff);
                Led_OnOffCtrl(Led2, !onOff);
            }
        }
    }
}

/************************************************************************************************
* Function Name: Led_Task
* Decription   : Led任务
* Input        : *p_arg-参数指针
* Output       : 无
* Others       : 无
************************************************************************************************/
void Led_Task(void *p_arg)
{
    uint8 i;
    LED_CONTROL *ledPtr;
    const uint16 FlashTimeArray[] = {0, 1, 2, 4, 6, 10, 16, 20, 24, 30, 36, 40, 50, 60, 100, 0x1F};

    (void)p_arg;

    // 开始时连续闪烁5次,用于检验LED是否正常工作
    for (i = 0; i < 5; i++) {
        Led_OnOffCtrl(Led1, ON);
        Led_OnOffCtrl(Led2, ON);
        OSTimeDlyHMSM(0, 0, 0, 100);
        Led_OnOffCtrl(Led1, OFF);
        Led_OnOffCtrl(Led2, OFF);
        OSTimeDlyHMSM(0, 0, 0, 80);
    }

    // 初始化LED控制参数
    for (i = Led1; i < TotalLed; i++) {
        Led.Control[i].RunTime = 0;
        Led.Control[i].Status = 0;
        Led.Control[i].DelayTimer = FLASHDELAYTIME / 50;
    }

    // LED闪烁控制
    while (TRUE) {
        OSTimeDlyHMSM(0, 0, 0, 50);
        for (i = Led1; i < TotalLed; i++) {
            ledPtr = &Led.Control[i];
            if (--ledPtr->DelayTimer == 0) {
                ledPtr->DelayTimer = FLASHDELAYTIME / 50;
                ledPtr->CurLedStatus = ledPtr->FlashCtrl;
            }
            if (ledPtr->RunTime > 0) {
                ledPtr->RunTime--;
            } else {
                if (AlwaysOff == FlashOffTime(ledPtr->CurLedStatus)) {
                    ledPtr->Status = OFF;
                } else if (AlwaysOn == FlashOnTime(ledPtr->CurLedStatus)) {
                    ledPtr->Status = ON;
                } else {
                    if (ON == ledPtr->Status) {
                        ledPtr->RunTime = FlashTimeArray[FlashOffTime(ledPtr->CurLedStatus)];
                        ledPtr->Status = OFF;
                    } else {
                        ledPtr->RunTime = FlashTimeArray[FlashOnTime(ledPtr->CurLedStatus)];
                        ledPtr->Status = ON;
                    }
                }
                Led_OnOffCtrl((LED_TYPE)i, ledPtr->Status);
            }
        }
    }
}

/************************************************************************************************
* Function Name: Led_FlashTime
* Decription   : Led闪烁控制函数
* Input        : LedType-Led类型,OnTime-Led亮的时间,OffTime-Led灭的时间,Effect-是否立即实施
* Output       : 无
* Others       : 无
************************************************************************************************/
void Led_FlashTime(LED_TYPE LedType, FLASH_TIME OnTime, FLASH_TIME OffTime, bool Effect)
{
    uint8 ledFlash = (OnTime << 4 & 0xF0 | OffTime);
    LED_CONTROL *ledPtr = &Led.Control[LedType];

    if (TRUE == Effect) {
        ledPtr->CurLedStatus = ledFlash;
        ledPtr->DelayTimer = 1;
        ledPtr->RunTime = 0;
    }
    ledPtr->FlashCtrl = ledFlash;
}

/***************************************End of file*********************************************/

