/**
  ******************************************************************************
  * @file    stm32_eval_i2c_ee.c
  * @author  MCD Application Team
  * @version V4.5.0
  * @date    07-March-2011
  * @brief   This file provides a set of functions needed to manage the I2C M24CXX 
  *          EEPROM memory mounted on STM32xx-EVAL board (refer to stm32_eval.h
  *          to know about the boards supporting this memory). 
  *          
  *          ===================================================================      
  *          Note: This driver is intended for STM32F10x families devices only.
  *          ===================================================================
  *            
  *          It implements a high level communication layer for read and write 
  *          from/to this memory. The needed STM32 hardware resources (I2C and 
  *          GPIO) are defined in stm32xx_eval.h file, and the initialization is 
  *          performed in sEE_LowLevel_Init() function declared in stm32xx_eval.c 
  *          file.
  *          You can easily tailor this driver to any other development board, 
  *          by just adapting the defines for hardware resources and 
  *          sEE_LowLevel_Init() function. 
  *        
  *          @note In this driver, basic read and write functions (sEE_ReadBuffer() 
  *                and sEE_WritePage()) use the DMA to perform the data transfer 
  *                to/from EEPROM memory (except when number of requested data is
  *                equal to 1). Thus, after calling these two functions, user 
  *                application may perform other tasks while DMA is transferring
  *                data. The application should then monitor the variable holding 
  *                the number of data in order to determine when the transfer is
  *                completed (variable decremented to 0). Stopping transfer tasks
  *                are performed into DMA interrupt handlers (which are integrated
  *                into this driver).
  *            
  *     +-----------------------------------------------------------------+
  *     |                        Pin assignment                           |                 
  *     +---------------------------------------+-----------+-------------+
  *     |  STM32 I2C Pins                       |   sEE     |   Pin       |
  *     +---------------------------------------+-----------+-------------+
  *     | .                                     |   E0(GND) |    1  (0V)  |
  *     | .                                     |   E1(GND) |    2  (0V)  |
  *     | .                                     |   E2(GND) |    3  (0V)  |
  *     | .                                     |   E0(VSS) |    4  (0V)  |
  *     | sEE_I2C_SDA_PIN/ SDA                  |   SDA     |    5        |
  *     | sEE_I2C_SCL_PIN/ SCL                  |   SCL     |    6        |
  *     | .                                     |   /WC(VDD)|    7 (3.3V) |
  *     | .                                     |   VDD     |    8 (3.3V) |
  *     +---------------------------------------+-----------+-------------+  
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************  
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32_eval_i2c_ee.h"
#include "pcf8563.h"
#include "bsp.h"
#include "i2c.h"                             //20110507 增加
#include "includes.h"

/** @addtogroup Utilities
  * @{
  */
  
/** @addtogroup STM32_EVAL
  * @{
  */ 

/** @addtogroup Common
  * @{
  */
  
/** @addtogroup STM32_EVAL_I2C_EE
  * @brief      This file includes the I2C EEPROM driver of STM32-EVAL boards.
  * @{
  */ 

/** @defgroup STM32_EVAL_I2C_EE_Private_Types
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup STM32_EVAL_I2C_EE_Private_Defines
  * @{
  */  
/**
  * @}
  */ 


/** @defgroup STM32_EVAL_I2C_EE_Private_Macros
  * @{
  */
/**
  * @}
  */ 
  

/** @defgroup STM32_EVAL_I2C_EE_Private_Variables
  * @{
  */
__IO uint16_t  EEAddress = 0;   
__IO uint32_t  EETimeout = EE_LONG_TIMEOUT;   
__IO uint16_t* EEDataReadPointer;   
__IO uint8_t*  EEDataWritePointer;  
__IO uint8_t   EEDataNum;
/**
  * @}
  */ 


/** @defgroup STM32_EVAL_I2C_EE_Private_Function_Prototypes
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup STM32_EVAL_I2C_EE_Private_Functions
  * @{
  */ 

/**
  * @brief  DeInitializes peripherals used by the I2C EEPROM driver.
  * @param  None
  * @retval None
  */

void EE_LowLevel_DeInit(void)                         //20110505 增加
{
  GPIO_InitTypeDef  GPIO_InitStructure; 
  NVIC_InitTypeDef NVIC_InitStructure;    
   
  /* EE_I2C Peripheral Disable */
  I2C_Cmd(I2C1, DISABLE);
 
  /* EE_I2C DeInit */
  I2C_DeInit(I2C1);

  /*!< EE_I2C Periph clock disable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, DISABLE);
    
  /*!< GPIO configuration */  
  /*!< Configure EE_I2C pins: SCL */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /*!< Configure EE_I2C pins: SDA */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  /* Configure and enable I2C DMA TX Channel interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel6_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Configure and enable I2C DMA RX Channel interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_Init(&NVIC_InitStructure);   
  
  /* Disable and Deinitialize the DMA channels */
  DMA_Cmd(DMA1_Channel6, DISABLE);
  DMA_Cmd(DMA1_Channel7, DISABLE);
  DMA_DeInit(DMA1_Channel6);
  DMA_DeInit(DMA1_Channel7);
}

/**
  * @brief  Initializes peripherals used by the I2C EEPROM driver.
  * @param  None
  * @retval None
  */
void EE_LowLevel_Init(void)                              //20110505 增加
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  NVIC_InitTypeDef  NVIC_InitStructure;  
  DMA_InitTypeDef   EEDMA_InitStructure;                 
    
  /*!< EE_I2C_SCL_GPIO_CLK and sEE_I2C_SDA_GPIO_CLK Periph clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

  /*!< EE_I2C Periph clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    
  /*!< GPIO configuration */  
  /*!< Configure EE_I2C pins: SCL */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Configure and enable I2C DMA TX Channel interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel6_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   //20110507 修改
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;          //20110507 修改
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Configure and enable I2C DMA RX Channel interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //20110507 修改
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;         //20110507 修改
  NVIC_Init(&NVIC_InitStructure);  
  
  /*!< I2C DMA TX and RX channels configuration */
  /* Enable the DMA clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  /* I2C TX DMA Channel configuration */
  DMA_DeInit(DMA1_Channel6);
  EEDMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)I2C1_DR_Address;
  EEDMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)0;   /* This parameter will be configured durig communication */
  EEDMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;    /* This parameter will be configured durig communication */
  EEDMA_InitStructure.DMA_BufferSize = 0xFFFF;            /* This parameter will be configured durig communication */
  EEDMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  EEDMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  EEDMA_InitStructure.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte;
  EEDMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  EEDMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  EEDMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  EEDMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel6, &EEDMA_InitStructure);  
  
  /* I2C RX DMA Channel configuration */
  DMA_DeInit(DMA1_Channel7);
  DMA_Init(DMA1_Channel7, &EEDMA_InitStructure);  
  
  /* Enable the DMA Channels Interrupts */
  DMA_ITConfig(DMA1_Channel6, DMA_IT_TC, ENABLE);
  DMA_ITConfig(DMA1_Channel7, DMA_IT_TC, ENABLE);    
}

/**
  * @brief  DeInitializes peripherals used by the I2C EEPROM driver.
  * @param  None
  * @retval None
  */

/**
  * @brief  Initializes DMA channel used by the I2C EEPROM driver.
  * @param  None
  * @retval None
  */
void EE_LowLevel_DMAConfig(uint32_t pBuffer, uint32_t BufferSize, uint32_t Direction)
{
DMA_InitTypeDef   EEDMA_InitStructure; 

  /* Initialize the DMA with the new parameters */
  if (Direction == 0)
  {
    /* Configure the DMA Tx Channel with the buffer address and the buffer size */
   /*   20110507 修改
    EEDMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)pBuffer;
    EEDMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;    
    EEDMA_InitStructure.DMA_BufferSize = (uint32_t)BufferSize;  
    DMA_Init(DMA1_Channel6, &EEDMA_InitStructure);  */

    EEDMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)I2C1_DR_Address;
    EEDMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)pBuffer;   /* This parameter will be configured durig communication */
    EEDMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;    /* This parameter will be configured durig communication */
    EEDMA_InitStructure.DMA_BufferSize = (uint32_t)BufferSize;            /* This parameter will be configured durig communication */
    EEDMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    EEDMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    EEDMA_InitStructure.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte;
    EEDMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    EEDMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    EEDMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    EEDMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel6, &EEDMA_InitStructure);     
  }
  else
  { 
    /* Configure the DMA Rx Channel with the buffer address and the buffer size */
    /*  20110507 修改
    EEDMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)pBuffer;
    EEDMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    EEDMA_InitStructure.DMA_BufferSize = (uint32_t)BufferSize;      
    DMA_Init(DMA1_Channel7, &EEDMA_InitStructure); */

    EEDMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)I2C1_DR_Address;
    EEDMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)pBuffer;   /* This parameter will be configured durig communication */
    EEDMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;          /* This parameter will be configured durig communication */
    EEDMA_InitStructure.DMA_BufferSize = (uint32_t)BufferSize;            /* This parameter will be configured durig communication */
    EEDMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    EEDMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    EEDMA_InitStructure.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte;
    EEDMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    EEDMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    EEDMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    EEDMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel7, &EEDMA_InitStructure);     
  }
}

void EE_DeInit(void)
{
  EE_LowLevel_DeInit(); 
}

/**
  * @brief  Initializes peripherals used by the I2C EEPROM driver.
  * @param  None
  * @retval None
  */
void EE_Init(void)
{  
  EE_DeInit();                   //20110508 增加
  
  EE_LowLevel_Init();  
  I2C_configuration();
}

void I2C_configuration(void)    //20110507 增加
{
  I2C_InitTypeDef  I2C_InitStructure;
  
  /*!< I2C configuration */
  /* sEE_I2C configuration */
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = I2C_SLAVE_ADDRESS7;   //从模式才用到自身地址,主模式不需要
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED;
  
  /* I2C1 Peripheral Enable */
  I2C_Cmd(I2C1, ENABLE);
  /* Apply I2C1 configuration after enabling it */
  I2C_Init(I2C1, &I2C_InitStructure);

  /* Enable the I2C1 peripheral DMA requests */
  I2C_DMACmd(I2C1, ENABLE);
}

/**
  * @brief  Reads a block of data from the EEPROM.
  * @param  pBuffer : pointer to the buffer that receives the data read from 
  *         the EEPROM.
  * @param  ReadAddr : EEPROM's internal address to start reading from.
  * @param  NumByteToRead : pointer to the variable holding number of bytes to 
  *         be read from the EEPROM.
  * 
  *        @note The variable pointed by NumByteToRead is reset to 0 when all the 
  *              data are read from the EEPROM. Application should monitor this 
  *              variable in order know when the transfer is complete.
  * 
  * @note When number of data to be read is higher than 1, this function just 
  *       configures the communication and enable the DMA channel to transfer data.
  *       Meanwhile, the user application may perform other tasks.
  *       When number of data to be read is 1, then the DMA is not used. The byte
  *       is read in polling mode.
  * 
  * @retval sEE_OK (0) if operation is correctly performed, else return value 
  *         different from sEE_OK (0) or the timeout user callback.
  */
uint32_t EE_ReadBuffer(uint8_t devAddr, uint8_t* pBuffer, uint16_t ReadAddr, uint16_t* NumByteToRead)  //20110507 函数增加参数 devAddr
{  
  /* Set the pointer to the Number of data to be read. This pointer will be used 
      by the DMA Transfer Completer interrupt Handler in order to reset the 
      variable to 0. User should check on this variable in order to know if the 
      DMA transfer has been complete or not. */
  EEDataReadPointer = NumByteToRead;
  
  EEAddress = devAddr;   
  
  /*!< While the bus is busy */
  EETimeout = EE_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))
  {
    if((EETimeout--) == 0) 
    {
      return EE_TIMEOUT_UserCallback();
    }
  }
  
  /*!< Send START condition */
  I2C_GenerateSTART(I2C1, ENABLE);
  
  /*!< Test on EV5 and clear it (cleared by reading SR1 then writing to DR) */
  EETimeout = EE_FLAG_TIMEOUT;
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
  {
    if((EETimeout--) == 0) 
    {
      return EE_TIMEOUT_UserCallback();
    }
  } 
  
  I2C_Send7bitAddress(I2C1, EEAddress, I2C_Direction_Transmitter);     //20110505 增加 原文件中少了这段时序，不能读取
  EETimeout = EE_FLAG_TIMEOUT;
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
  {
    if((EETimeout--) == 0) 
    {
      return EE_TIMEOUT_UserCallback();
    }
  }  

  if (EEAddress == ADDR_8563)                   //pcf8563 8位地址
  {
  
  /*!< Send the EEPROM's internal address to read from: Only one byte address */
    I2C_SendData(I2C1, ReadAddr);  
  }
  else if(EEAddress == ADDR_24C128)             //24c128 16位地址
  {

    /*!< Send the EEPROM's internal address to read from: MSB of the address first */
    I2C_SendData(I2C1, (uint8_t)((ReadAddr & 0xFF00) >> 8));    

    /*!< Test on EV8 and clear it */
    EETimeout = EE_FLAG_TIMEOUT;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
      if((EETimeout--) == 0) 
      {
        return EE_TIMEOUT_UserCallback();
      }
    }

    /*!< Send the EEPROM's internal address to read from: LSB of the address */
    I2C_SendData(I2C1, (uint8_t)(ReadAddr & 0x00FF));   
  }

  /*!< Test on EV8 and clear it */
  EETimeout = EE_FLAG_TIMEOUT;
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
  {
    if((EETimeout--) == 0) 
    {
      return EE_TIMEOUT_UserCallback();
    }
  }
  
  /*!< Send STRAT condition a second time */  
  I2C_GenerateSTART(I2C1, ENABLE);
  
  /*!< Test on EV5 and clear it (cleared by reading SR1 then writing to DR) */
  EETimeout = EE_FLAG_TIMEOUT;
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
  {
    if((EETimeout--) == 0) 
    {
      return EE_TIMEOUT_UserCallback();
    }
  } 
  
  /*!< Send EEPROM address for read */
  I2C_Send7bitAddress(I2C1, EEAddress, I2C_Direction_Receiver);      
  
  /* If number of data to be read is 1, then DMA couldn't be used */
  /* One Byte Master Reception procedure (POLLING) ---------------------------*/
  if ((uint16_t)(*NumByteToRead) < 2)
  {
    /* Wait on ADDR flag to be set (ADDR is still not cleared at this level */
    EETimeout = EE_FLAG_TIMEOUT;                                            //20110505 注释掉                                
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_ADDR) == RESET)
    {
      if((EETimeout--) == 0) 
      {  
        return EE_TIMEOUT_UserCallback();
      }
    }     
    
    /*!< Disable Acknowledgement */
    I2C_AcknowledgeConfig(I2C1, DISABLE);   

    /* Call User callback for critical section start (should typically disable interrupts) */
    EE_EnterCriticalSection_UserCallback();   
    
    /* Clear ADDR register by reading SR1 then SR2 register (SR1 has already been read) */
    (void)I2C1->SR2;    
    
    /*!< Send STOP Condition */
    I2C_GenerateSTOP(I2C1, ENABLE);  
   
    /* Call User callback for critical section end (should typically re-enable interrupts) */
    EE_ExitCriticalSection_UserCallback();  
    
    /* Wait for the byte to be received */
    EETimeout = EE_FLAG_TIMEOUT;
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE) == RESET)
    {
      if((EETimeout--) == 0) 
      {
        return EE_TIMEOUT_UserCallback();
      }
    }
    
    /*!< Read the byte received from the EEPROM */
    *pBuffer = I2C_ReceiveData(I2C1);
    
    /*!< Decrement the read bytes counter */
//  (uint16_t)(*NumByteToRead)--;         20110507 注释掉
    
    /* Wait to make sure that STOP control bit has been cleared */
    EETimeout = EE_FLAG_TIMEOUT;    
    while(I2C1->CR1 & I2C_CR1_STOP)
    {
      if((EETimeout--) == 0) 
      {
        return EE_TIMEOUT_UserCallback();
      }
    } 
    
    /*!< Re-Enable Acknowledgement to be ready for another reception */
    I2C_AcknowledgeConfig(I2C1, ENABLE);  
    
    DMAReadDoneFlag = TRUE; 
  }
  else/* More than one Byte Master Reception procedure (DMA) -----------------*/
  {
    /*!< Test on EV6 and clear it */
    EETimeout = EE_FLAG_TIMEOUT;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
    {
      if((EETimeout--) == 0) 
      {  
        return EE_TIMEOUT_UserCallback();
      }
    }  
    
    /* Configure the DMA Rx Channel with the buffer address and the buffer size */
    EE_LowLevel_DMAConfig((uint32_t)pBuffer, (uint16_t)(*NumByteToRead), 1);
    
    /* Inform the DMA that the next End Of Transfer Signal will be the last one */
    I2C_DMALastTransferCmd(I2C1, ENABLE); 
    
    /* Enable the DMA Rx Channel */
    DMA_Cmd(DMA1_Channel7, ENABLE);  
  }
  
  /* If all operations OK, return sEE_OK (0) */
  return EE_OK;
}

/**
  * @brief  Writes more than one byte to the EEPROM with a single WRITE cycle.
  *
  * @note   The number of bytes (combined to write start address) must not 
  *         cross the EEPROM page boundary. This function can only write into
  *         the boundaries of an EEPROM page.
  *         This function doesn't check on boundaries condition (in this driver 
  *         the function sEE_WriteBuffer() which calls sEE_WritePage() is 
  *         responsible of checking on Page boundaries).
  * 
  * @param  pBuffer : pointer to the buffer containing the data to be written to 
  *         the EEPROM.
  * @param  WriteAddr : EEPROM's internal address to write to.
  * @param  NumByteToWrite : pointer to the variable holding number of bytes to 
  *         be written into the EEPROM. 
  * 
  *        @note The variable pointed by NumByteToWrite is reset to 0 when all the 
  *              data are written to the EEPROM. Application should monitor this 
  *              variable in order know when the transfer is complete.
  * 
  * @note This function just configure the communication and enable the DMA 
  *       channel to transfer data. Meanwhile, the user application may perform 
  *       other tasks in parallel.
  * 
  * @retval sEE_OK (0) if operation is correctly performed, else return value 
  *         different from sEE_OK (0) or the timeout user callback.
  */
uint32_t EE_WritePage(uint8_t devAddr,uint8_t* pBuffer, uint16_t WriteAddr, uint8_t* NumByteToWrite)
{ 
  /* Set the pointer to the Number of data to be written. This pointer will be used 
      by the DMA Transfer Completer interrupt Handler in order to reset the 
      variable to 0. User should check on this variable in order to know if the 
      DMA transfer has been complete or not. */
  EEDataWritePointer = NumByteToWrite;  
  EEAddress = devAddr;
  
  /*!< While the bus is busy */
  EETimeout = EE_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))
  {
    if((EETimeout--) == 0) 
    {
      return EE_TIMEOUT_UserCallback();
    }
  }
  
  /*!< Send START condition */
  I2C_GenerateSTART(I2C1, ENABLE);
  
  /*!< Test on EV5 and clear it */
  EETimeout = EE_FLAG_TIMEOUT;
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
  {
    if((EETimeout--) == 0) 
    {
      return EE_TIMEOUT_UserCallback();
    }
  }
  
  /*!< Send EEPROM address for write */
  EETimeout = EE_FLAG_TIMEOUT;
  I2C_Send7bitAddress(I2C1, EEAddress, I2C_Direction_Transmitter);

  /*!< Test on EV6 and clear it */
  EETimeout = EE_FLAG_TIMEOUT;
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
  {
    if((EETimeout--) == 0) 
    {
      return EE_TIMEOUT_UserCallback();
    }
  }
  
  if (EEAddress == ADDR_8563)                   //pcf8563 8位地址
  {  
    /*!< Send the EEPROM's internal address to write to : only one byte Address */
    I2C_SendData(I2C1, WriteAddr);
  }
  else if (EEAddress == ADDR_24C128)
  {  
    /*!< Send the EEPROM's internal address to write to : MSB of the address first */
    I2C_SendData(I2C1, (uint8_t)((WriteAddr & 0xFF00) >> 8));

    /*!< Test on EV8 and clear it */
    EETimeout = EE_FLAG_TIMEOUT;  
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
      if((EETimeout--) == 0) 
      {
        return EE_TIMEOUT_UserCallback();
      }
    }  
  
    /*!< Send the EEPROM's internal address to write to : LSB of the address */
    I2C_SendData(I2C1, (uint8_t)(WriteAddr & 0x00FF));
  } 
  
  /*!< Test on EV8 and clear it */
  EETimeout = EE_FLAG_TIMEOUT; 
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
  {
    if((EETimeout--) == 0) 
    {
      return EE_TIMEOUT_UserCallback();
    }
  }  
  
  /* Configure the DMA Tx Channel with the buffer address and the buffer size */
  EE_LowLevel_DMAConfig((uint32_t)pBuffer, (uint8_t)(*NumByteToWrite), 0);
    
  /* Enable the DMA Tx Channel */
  DMA_Cmd(DMA1_Channel6, ENABLE);
  
  /* If all operations OK, return sEE_OK (0) */
  return EE_OK;
}

/**
  * @brief  Writes buffer of data to the I2C EEPROM.
  * @param  pBuffer : pointer to the buffer  containing the data to be written 
  *         to the EEPROM.
  * @param  WriteAddr : EEPROM's internal address to write to.
  * @param  NumByteToWrite : number of bytes to write to the EEPROM.
  * @retval None
  */
void EE_WriteBuffer(uint8_t* pBuffer, uint16_t WriteAddr, uint16_t NumByteToWrite)
{
/*    20110507 注释掉  
  uint8_t NumOfPage = 0, NumOfSingle = 0, count = 0;
  uint16_t Addr = 0;

  Addr = WriteAddr % EE_PAGESIZE;
  count = EE_PAGESIZE - Addr;
  NumOfPage =  NumByteToWrite / EE_PAGESIZE;
  NumOfSingle = NumByteToWrite % EE_PAGESIZE;
 
  //!< If WriteAddr is sEE_PAGESIZE aligned  
  if(Addr == 0) 
  {
    //!< If NumByteToWrite < sEE_PAGESIZE 
    if(NumOfPage == 0) 
    {
      // Store the number of data to be written 
      EEDataNum = NumOfSingle;
      // Start writing data 
      EE_WritePage(pBuffer, WriteAddr, (uint8_t*)(&EEDataNum));
      // Wait transfer through DMA to be complete 
      EETimeout = EE_LONG_TIMEOUT;
      while (EEDataNum > 0)
      {
        if((EETimeout--) == 0) {EE_TIMEOUT_UserCallback(); return;};
      }
      EE_WaitEepromStandbyState();
    }
    //!< If NumByteToWrite > sEE_PAGESIZE 
    else  
    {
      while(NumOfPage--)
      {
        // Store the number of data to be written 
        EEDataNum = EE_PAGESIZE;        
        EE_WritePage(pBuffer, WriteAddr, (uint8_t*)(&EEDataNum)); 
        // Wait transfer through DMA to be complete 
        EETimeout = EE_LONG_TIMEOUT;
        while (EEDataNum > 0)
        {
          if((EETimeout--) == 0) {EE_TIMEOUT_UserCallback(); return;};
        }      
        EE_WaitEepromStandbyState();
        WriteAddr +=  EE_PAGESIZE;
        pBuffer += EE_PAGESIZE;
      }

      if(NumOfSingle!=0)
      {
        // Store the number of data to be written 
        EEDataNum = NumOfSingle;          
        EE_WritePage(pBuffer, WriteAddr, (uint8_t*)(&EEDataNum));
        // Wait transfer through DMA to be complete 
        EETimeout = EE_LONG_TIMEOUT;
        while (EEDataNum > 0)
        {
          if((EETimeout--) == 0) {EE_TIMEOUT_UserCallback(); return;};
        }    
        EE_WaitEepromStandbyState();
      }
    }
  }
  //!< If WriteAddr is not sEE_PAGESIZE aligned  
  else 
  {
    //!< If NumByteToWrite < sEE_PAGESIZE 
    if(NumOfPage== 0) 
    {
      //!< If the number of data to be written is more than the remaining space in the current page:       
      if (NumByteToWrite > count)
      {
        // Store the number of data to be written 
        EEDataNum = count;        
        //!< Write the data conained in same page 
        EE_WritePage(pBuffer, WriteAddr, (uint8_t*)(&EEDataNum));
        // Wait transfer through DMA to be complete 
        EETimeout = EE_LONG_TIMEOUT;
        while (EEDataNum > 0)
        {
          if((EETimeout--) == 0) {EE_TIMEOUT_UserCallback(); return;};
        }          
        EE_WaitEepromStandbyState();      
        
        // Store the number of data to be written 
        EEDataNum = (NumByteToWrite - count);          
        //!< Write the remaining data in the following page 
        EE_WritePage((uint8_t*)(pBuffer + count), (WriteAddr + count), (uint8_t*)(&EEDataNum));
        // Wait transfer through DMA to be complete 
        EETimeout = EE_LONG_TIMEOUT;
        while (EEDataNum > 0)
        {
          if((EETimeout--) == 0) {EE_TIMEOUT_UserCallback(); return;};
        }     
        EE_WaitEepromStandbyState();        
      }      
      else      
      {
        // Store the number of data to be written 
        EEDataNum = NumOfSingle;         
        EE_WritePage(pBuffer, WriteAddr, (uint8_t*)(&EEDataNum));
        // Wait transfer through DMA to be complete 
        EETimeout = EE_LONG_TIMEOUT;
        while (EEDataNum > 0)
        {
          if((EETimeout--) == 0) {EE_TIMEOUT_UserCallback(); return;};
        }          
        EE_WaitEepromStandbyState();        
      }     
    }
    //!< If NumByteToWrite > sEE_PAGESIZE 
    else
    {
      NumByteToWrite -= count;
      NumOfPage =  NumByteToWrite / EE_PAGESIZE;
      NumOfSingle = NumByteToWrite % EE_PAGESIZE;
      
      if(count != 0)
      {  
        // Store the number of data to be written 
        EEDataNum = count;         
        EE_WritePage(pBuffer, WriteAddr, (uint8_t*)(&EEDataNum));
        // Wait transfer through DMA to be complete 
        EETimeout = EE_LONG_TIMEOUT;
        while (EEDataNum > 0)
        {
          if((EETimeout--) == 0) {EE_TIMEOUT_UserCallback(); return;};
        }     
        EE_WaitEepromStandbyState();
        WriteAddr += count;
        pBuffer += count;
      } 
      
      while(NumOfPage--)
      {
        // Store the number of data to be written 
        EEDataNum = EE_PAGESIZE;          
        EE_WritePage(pBuffer, WriteAddr, (uint8_t*)(&EEDataNum));
        // Wait transfer through DMA to be complete 
        EETimeout = EE_LONG_TIMEOUT;
        while (EEDataNum > 0)
        {
          if((EETimeout--) == 0) {EE_TIMEOUT_UserCallback(); return;};
        }        
        EE_WaitEepromStandbyState();
        WriteAddr +=  EE_PAGESIZE;
        pBuffer += EE_PAGESIZE;  
      }
      if(NumOfSingle != 0)
      {
        // Store the number of data to be written 
        EEDataNum = NumOfSingle;           
        EE_WritePage(pBuffer, WriteAddr, (uint8_t*)(&EEDataNum)); 
        //Wait transfer through DMA to be complete 
        EETimeout = EE_LONG_TIMEOUT;
        while (EEDataNum > 0)
        {
          if((EETimeout--) == 0) {EE_TIMEOUT_UserCallback(); return;};
        }         
        EE_WaitEepromStandbyState();
      }
    }
  }*/  
}

/**
  * @brief  Wait for EEPROM Standby state.
  * 
  * @note  This function allows to wait and check that EEPROM has finished the 
  *        last Write operation. It is mostly used after Write operation: after 
  *        receiving the buffer to be written, the EEPROM may need additional 
  *        time to actually perform the write operation. During this time, it 
  *        doesn't answer to I2C packets addressed to it. Once the write operation 
  *        is complete the EEPROM responds to its address.
  *        
  * @note  It is not necessary to call this function after sEE_WriteBuffer() 
  *        function (sEE_WriteBuffer() already calls this function after each
  *        write page operation).    
  * 
  * @param  None
  * @retval sEE_OK (0) if operation is correctly performed, else return value 
  *         different from sEE_OK (0) or the timeout user callback.
  */
uint32_t EE_WaitEepromStandbyState(void)      
{
  __IO uint16_t tmpSR1 = 0;
  __IO uint32_t EETrials = 0;

  /*!< While the bus is busy */
  EETimeout = EE_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))
  {
    if((EETimeout--) == 0) return EE_TIMEOUT_UserCallback();
  }

  /* Keep looping till the slave acknowledge his address or maximum number 
     of trials is reached (this number is defined by sEE_MAX_TRIALS_NUMBER define
     in stm32_eval_i2c_ee.h file) */
  while (1)
  {
    /*!< Send START condition */
    I2C_GenerateSTART(I2C1, ENABLE);

    /*!< Test on EV5 and clear it */
    EETimeout = EE_FLAG_TIMEOUT;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
    {
      if((EETimeout--) == 0) return EE_TIMEOUT_UserCallback();
    }    

    /*!< Send EEPROM address for write */
    I2C_Send7bitAddress(I2C1, EEAddress, I2C_Direction_Transmitter);
    
    /* Wait for ADDR flag to be set (Slave acknowledged his address) */
    EETimeout = EE_LONG_TIMEOUT;
    do
    {     
      /* Get the current value of the SR1 register */
      tmpSR1 = I2C1->SR1;
      
      /* Update the timeout value and exit if it reach 0 */
      if((EETimeout--) == 0) return EE_TIMEOUT_UserCallback();
    }
    /* Keep looping till the Address is acknowledged or the AF flag is 
       set (address not acknowledged at time) */
    while((tmpSR1 & (I2C_SR1_ADDR | I2C_SR1_AF)) == 0);
     
    /* Check if the ADDR flag has been set */
    if (tmpSR1 & I2C_SR1_ADDR)
    {
      /* Clear ADDR Flag by reading SR1 then SR2 registers (SR1 have already 
         been read) */
      (void)I2C1->SR2;
      
      /*!< STOP condition */    
      I2C_GenerateSTOP(I2C1, ENABLE);
        
      /* Exit the function */
      return EE_OK;
    }
    else
    {
      /*!< Clear AF flag */
      I2C_ClearFlag(I2C1, I2C_FLAG_AF);                  
    }
    
    /* Check if the maximum allowed numbe of trials has bee reached */
    if (EETrials++ == EE_MAX_TRIALS_NUMBER)
    {
      /* If the maximum number of trials has been reached, exit the function */
      return EE_TIMEOUT_UserCallback();
    }
  }
}

/**
  * @brief  This function handles the DMA Tx Channel interrupt Handler.
  * @param  None
  * @retval None
  */
void DMA1_Channel6_IRQHandler(void)
{
  /* Check if the DMA transfer is complete */ 
  if(DMA_GetFlagStatus(DMA1_IT_TC6) != RESET)
  {  
    /* Disable the DMA Tx Channel and Clear all its Flags */  
    DMA_Cmd(DMA1_Channel6, DISABLE);
    DMA_ClearFlag(DMA1_IT_GL6);

    /*!< Wait till all data have been physically transferred on the bus */
    EETimeout = EE_LONG_TIMEOUT;
    while(!I2C_GetFlagStatus(I2C1, I2C_FLAG_BTF))
    {
      if((EETimeout--) == 0) EE_TIMEOUT_UserCallback();
    }
    
    /*!< Send STOP condition */
    I2C_GenerateSTOP(I2C1, ENABLE);
    
    /* Perform a read on SR1 and SR2 register to clear eventualaly pending flags */
    (void)I2C1->SR1;
    (void)I2C1->SR2;
    
    /* Reset the variable holding the number of data to be written */
    *EEDataWritePointer = 0;  
    
    DMAWriteDoneFlag = TRUE;                                           //20110507 增加
  }
}

/**
  * @brief  This function handles the DMA Rx Channel interrupt Handler.
  * @param  None
  * @retval None
  */
void DMA1_Channel7_IRQHandler(void)
{
  /* Check if the DMA transfer is complete */  
  if(DMA_GetFlagStatus(DMA1_IT_TC7) != RESET)  
  {      
    /*!< Send STOP Condition */
    I2C_GenerateSTOP(I2C1, ENABLE);    
    
    /* Disable the DMA Rx Channel and Clear all its Flags */  
    DMA_Cmd(DMA1_Channel7, DISABLE);
    DMA_ClearFlag(DMA1_IT_GL7);
    
    /* Reset the variable holding the number of data to be read */
    *EEDataReadPointer = 0;
    
    DMAReadDoneFlag = TRUE;                                           //20110507 增加
  }
}

#ifdef USE_DEFAULT_TIMEOUT_CALLBACK
/**
  * @brief  Basic management of the timeout situation.
  * @param  None.
  * @retval None.
  */
uint32_t EE_TIMEOUT_UserCallback(void)
{
  /* Block communication and all processes */
unsigned char i = 0;  

/*20110514 注释掉
  while (1)
  {   
  }*/

  GPIO_InitTypeDef  GPIO_InitStructure;   
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  for (i = 0;i < 10;i++)                                                        //复位从器件方法：SCL 对应的端口为OD输出，再强制向SCL发送9个脉冲
  {  
    GPIO_SetBits(GPIOB, GPIO_Pin_6);
    delay_ms(10);
    GPIO_ResetBits(GPIOB, GPIO_Pin_6);  
  } 
  
  I2C_SoftwareResetCmd(I2C1, ENABLE);                                           //复位STM32的 i2c
  
  EE_Init();                                                                    //重新初始化i2c
 
  return 0;
}
#endif /* USE_DEFAULT_TIMEOUT_CALLBACK */

#ifdef USE_DEFAULT_CRITICAL_CALLBACK
/**
  * @brief  Start critical section: these callbacks should be typically used
  *         to disable interrupts when entering a critical section of I2C communication
  *         You may use default callbacks provided into this driver by uncommenting the 
  *         define USE_DEFAULT_CRITICAL_CALLBACK.
  *         Or you can comment that line and implement these callbacks into your 
  *         application.
  * @param  None.
  * @retval None.
  */
void EE_EnterCriticalSection_UserCallback(void)
{
  __disable_irq();  
}

/**
  * @brief  Start and End of critical section: these callbacks should be typically used
  *         to re-enable interrupts when exiting a critical section of I2C communication
  *         You may use default callbacks provided into this driver by uncommenting the 
  *         define USE_DEFAULT_CRITICAL_CALLBACK.
  *         Or you can comment that line and implement these callbacks into your 
  *         application.
  * @param  None.
  * @retval None.
  */
void EE_ExitCriticalSection_UserCallback(void)
{
  __enable_irq();
}
#endif /* USE_DEFAULT_CRITICAL_CALLBACK */

/****************************************************************
 * Function Name: WaitE2ReadOver
 * Decription   : 等待DMA传输和EEPROM写完
 * Calls        :
 * Called By    :
 * Input        :
 *              1.devAddr:          I2C设备地址
 *              2.ReadAddr:         要读出数据的I2C设备片内地址
 *              3.pBuffer:          将读出的数据存入以此为首地址的缓冲区
 *              4.NumByteToRead:    待读数据字节长度
 * Output       :
 * Return Value :
 * Others       :
 *------------------------- Revision History -------------------
 * No. Version Date      Revised by   Item      Description
 * 1   V1.0   2011-11-19 wangwenbin  SRWF-9547 Original Version
 *****************************************************************/
void WaitE2ReadOver(void)
{
uint16 i = 0;  
  
  while ((!DMAReadDoneFlag) && (i < 0xFFFF))                                    //DMA传输完成 20110514 增加 i < 2000 (测试DMA传输完64字节数据需要的值 > 800)
  {
    i++;
  }  

  DMAReadDoneFlag = FALSE;  
}

/****************************************************************
 * Function Name: WaitE2WriteOver
 * Decription   : 从I2C设备读取数据，用于内部寻址大于8位而不超过16位的I2C设备
 * Calls        :
 * Called By    :
 * Input        :
 *              1.devAddr:          I2C设备地址
 *              2.ReadAddr:         要读出数据的I2C设备片内地址
 *              3.pBuffer:          将读出的数据存入以此为首地址的缓冲区
 *              4.NumByteToRead:    待读数据字节长度
 * Output       :
 * Return Value :
 * Others       :
 *------------------------- Revision History -------------------
 * No. Version Date      Revised by   Item      Description
 * 1   V1.0   2011-11-19 wangwenbin  SRWF-9547 Original Version
 *****************************************************************/
void WaitE2WriteOver(void)
{
uint16 i = 0;  
  
  while ((!DMAWriteDoneFlag) && (i < 0xFFFF))                                   //DMA传输完成 20110514 增加 i < 2000 
  {
    i++;
  }
  
  DMAWriteDoneFlag = FALSE;   
//  delay_ms(10);  
  OSTimeDlyHMSM(0, 0, 0, 10); 
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
