/************************************************************************************************
*                                   SRWF-6009-BOOT
*    (c) Copyright 2015, Software Department, Sunray Technology Co.Ltd
*                               All Rights Reserved
*
* FileName     : Interrupt.h
* Description  :
* Version      :
* Function List:
*------------------------------Revision History--------------------------------------------------
* No.   Version     Date            Revised By      Item            Description
* 1     V1.0        10/08/2015      Zhangxp         SRWF-6009-BOOT  Original Version
************************************************************************************************/
#ifndef _INTERRUPT_H_
#define _INTERRUPT_H_

/************************************************************************************************
*                          Pubilc Macro Define Section
************************************************************************************************/
//中断水平定义
#define INT_LEVEL_1                                     0x01
#define INT_LEVEL_2                                     0x02
#define INT_LEVEL_3                                     0x03
#define INT_LEVEL_4                                     0x04
#define INT_LEVEL_5                                     0x05
#define INT_LEVEL_6                                     0x06
#define INT_LEVEL_7                                     0x07
#define INT_LEVEL_OFF                                   0x00    //关中断

#define INT_ON                                          1
#define INT_OFF                                         0
#define INT_SET                                         1
#define INT_CLR                                         0

// 中断脚IO选择
#define EINT_LOC_SET(intnum,loc)                        do                                  \
                                                        {                                   \
                                                            if (loc == EINT_LOC_SET_PORT_3) \
                                                            {                               \
                                                                intsr &= ~BV(intnum);       \
                                                            }                               \
                                                            else                            \
                                                            {                               \
                                                                intsr |= BV(intnum);        \
                                                            }                               \
                                                        } while(0)

//intmun可以取0-7
//loc可以取
#define EINT_LOC_SET_PORT_3                             0
#define EINT_LOC_SET_PORT_11                            1

//中断极性选择
#define EINT_POLARITY_SET(intnum, pol)                  int##intnum##pl = pol
//intmun可以取0-7
//pol可以取
#define EINT_POLARITY_ONE                               0
#define EINT_POLARITY_BOTH                              1

//中断沿选择，在选择单沿时有效,双边沿时必须写０
#define EINT_EDGE_SET(intnum, edge)                     pol_int##intnum##ic = edge
//intmun可以取0-7
//edge可以取
#define EINT_EDGE_FALLING                               0
#define EINT_EDGE_RISING                                1

//外部中断优先级选择
#define EINT_LEVEL(intnum, level)                       do                                  \
                                                        {                                   \
                                                            int##intnum##ic &= 0xf8;        \
                                                            int##intnum##ic |= level;       \
                                                        } while(0)


//外部中断开关
#define EINT_ENABLE(intnum, on)                         int##intnum##en = on


//intmun可以取0-7
//on可以取 INT_ON / INT_OFF

//外部中断标志清除
#define EINT_FLAG_CLEAR(intnum)                         ir_int##intnum##ic = 0


/************************************************************************************************
*                            Variable Declare Section
************************************************************************************************/

/************************************************************************************************
*                            Function Declare Section
************************************************************************************************/
extern void ConfigureInterrupts(void);

#endif

/**************************************End of file**********************************************/


