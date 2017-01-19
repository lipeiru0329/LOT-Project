/**
  ******************************************************************************
  * @file     SPI1.h 
  * @author   xukai
  * @version  V0.1
  * @date     2012-3-29
  * @brief    ��ʼ��SPI1��SPI1��д��������       
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI1_H
#define __SPI1_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

//��������
void SPI1_Config(void);
uint8_t SPI1_SendByte(uint8_t byte);
uint8_t SPI1_ReceiveByte();
#endif
/***************************************************************END OF FILE****/
