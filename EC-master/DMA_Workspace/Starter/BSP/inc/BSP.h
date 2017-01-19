/*
 * BSP.h
 *
 *  Created on: May 31, 2016
 *      Author: ChenSi
 */


#ifndef __BSP_H__
#define __BSP_H__
#include "stm32f10x.h"
#include <stdio.h>
/*----------printf调试选项----------*/
#define _DEBUG 1
#if _DEBUG
#define PRINTF(fmt, ...) printf(fmt, ## __VA_ARGS__)
#else
#define PRINTF(fmt, ...)
#endif
/*----------串口1相关定义----------*/
#define COM1                USART1
#define COM1_RCC            RCC_APB2Periph_USART1
#define COM1_GPIO_RCC       RCC_APB2Periph_GPIOA
#define COM1_GPIO_PORT      GPIOA
#define COM1_TX_PIN         GPIO_Pin_9
#define COM1_RX_PIN         GPIO_Pin_10
/*----------串口2相关定义----------*/
#define COM2_USART          USART2
#define COM2_RCC            RCC_APB1Periph_USART2
#define COM2_GPIO_RCC       RCC_APB2Periph_GPIOA
#define COM2_GPIO_PORT      GPIOA
#define COM2_TX_PIN         GPIO_Pin_2
#define COM2_RX_PIN         GPIO_Pin_3
/*----------串口3相关定义----------*/
#define COM3                USART3
#define COM3_RCC            RCC_APB1Periph_USART3
#define COM3_GPIO_RCC       RCC_APB2Periph_GPIOC
#define COM3_GPIO_PORT      GPIOC
#define COM3_TX_PIN         GPIO_Pin_10
#define COM3_RX_PIN         GPIO_Pin_11
/*----------LED1相关定义----------*/
#define LED1_Pin			GPIO_Pin_6
#define LED1_GPIOX			GPIOF
#define LED1_GPIOX_CLK		RCC_APB2Periph_GPIOF
#define LED1_ON()			{LED1_GPIOX->BRR   = LED1_Pin;}
#define LED1_OFF()			{LED1_GPIOX->BSRR  = LED1_Pin;}
#define LED1_Toggle()   	{LED1_GPIOX->ODR = (LED1_GPIOX->ODR)^LED1_Pin;}



void BSP_Init(void);
void delay_1us(void);
void delay_nus(u32 nus);
void delay_1ms(void);
void delay_nms(u16);
void USART2_putc(uint8_t);
void USART2_Config(u32 baudRate);
void USART3_putc(uint8_t);
void USART3_Config(u32 baudRate);
void NVIC_Config(void);
void MUX_Init(void);
void MUX_Select(uint8_t);
void SPI_Configuration(void);
void GPIO_Configuration(void);
void RCC_Config(void);
void Calendar_RTC_Init(void);
void Systick_Init(void);
void USART3_DMA_Init(void);
void SPI1_Config(void);
uint8_t SPI1_SendByte(uint8_t byte);
uint8_t SPI1_ReceiveByte();
void SPI2_Config();

extern uint8_t USART_Buf[128];

#endif
