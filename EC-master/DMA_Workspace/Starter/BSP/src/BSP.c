/*
 * BSP.c
 *
 *  Created on: May 31, 2016
 *      Author: ChenSi
 */
#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "bsp.h"
#include "xprintf.h"
#include "stm32f10x_bkp.h"
#include "stm32f10x_rtc.h"
#include "system_stm32f10x.h"

uint8_t USART_Buf[128];

void delay_1us(void)  //延时1us
{
	u8 i = 0;
	i++;
	i++;
	i++;
	i++;
	i++;
	i++;
	i++;
	i++;
	i++;
	i++;
	i++;
	i++;
	i++;
	i++;
	i++;
	i++;
	i++;
	i++;
	i++;
	i++;
	i++;
	i++;
	i++;
	i++;
	i++;
	i++;
	i++;
	i++;
	i++;
	i++;
	i++;
	i++;
	i++;
	i++;
	i++;
	i++;
	i++;
	i++;
	i++;
	i++;
	i++;
	i++;
}
void delay_nus(u32 x) {
	u16 i = 0;
	for (i = 0; i < x; i++) {
		delay_1us();
	}
}
void delay_1ms(void) {
	u16 i = 0;
	for (i = 0; i < 6500; i++) {
		;
	}
}
void delay_nms(u16 nms) {
	u16 i = 0;
	for (i = 0; i < 6500 * nms; i++) {
		;
	}
}

void RCC_Config(void) {
	static volatile ErrorStatus HSEStartUpStatus = SUCCESS;

	RCC_DeInit();	                  //默认配置SYSCLK, HCLK, PCLK2, PCLK1, 复位后就是该配置
	RCC_HSEConfig(RCC_HSE_ON);                 //使能外部高速晶振
	HSEStartUpStatus = RCC_WaitForHSEStartUp();                 //等待外部高速稳定

	if (HSEStartUpStatus == SUCCESS) {
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);    //使能flash预读取缓冲区
		FLASH_SetLatency(FLASH_Latency_2); //令Flash处于等待状态，2是针对高频时钟的
		RCC_HCLKConfig(RCC_SYSCLK_Div1);   //HCLK = SYSCLK 设置高速总线时钟=系统时钟
		RCC_PCLK2Config(RCC_HCLK_Div1);    //PCLK2 = HCLK 设置低速总线2时钟=高速总线时钟
		RCC_PCLK1Config(RCC_HCLK_Div2);    //PCLK1 = HCLK/2 设置低速总线1的时钟=高速时钟的二分频
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9); //PLLCLK = 8MHz * 9 = 72 MHz 利用锁相环讲外部8Mhz晶振9倍频到72Mhz
		RCC_PLLCmd(ENABLE);		           //使能PLL锁相环
		while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) {
		} //等待锁相环输出稳定
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); //将锁相环输出设置为系统时钟
		while (RCC_GetSYSCLKSource() != 0x08) {
		}     //等待校验成功
		SystemCoreClockUpdate();
	}
	//使能GPIO口所使用的时钟
	RCC_APB2PeriphClockCmd(
			RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC
					| RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE
					| RCC_APB2Periph_GPIOF | RCC_APB2Periph_GPIOG, ENABLE);
}

void Systick_Init() {
	if (SysTick_Config(SystemCoreClock / 1000) != 0) {
		while (1)
			;
	};
}

void BSP_Init(void) {
	RCC_APB2PeriphClockCmd(
			RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC
					| RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE
					| RCC_APB2Periph_GPIOF | RCC_APB2Periph_GPIOG, ENABLE);
	USART2_Config(115200);
	USART3_Config(9600);
	MUX_Init();
	MUX_Select(3);
	NVIC_Config();
	xdev_out(USART3_putc);
	GPIO_Configuration();
	USART3_DMA_Init();
	SPI2_Config();
}

void MUX_Init() {
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void MUX_Select(uint8_t port) {
	if (port == 0)
		GPIO_ResetBits(GPIOC, GPIO_Pin_4 | GPIO_Pin_5);
	else if (port == 1) {
		GPIO_ResetBits(GPIOC, GPIO_Pin_4);
		GPIO_SetBits(GPIOC, GPIO_Pin_5);
	} else if (port == 2) {
		GPIO_ResetBits(GPIOC, GPIO_Pin_5);
		GPIO_SetBits(GPIOC, GPIO_Pin_4);
	} else
		GPIO_SetBits(GPIOC, GPIO_Pin_4 | GPIO_Pin_5);
}

static void USART2_Config(u32 baudRate) {
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB2PeriphClockCmd(COM2_RCC, ENABLE);       //使能 USART2 时钟
	RCC_APB2PeriphClockCmd(COM2_GPIO_RCC, ENABLE);  //使能串口2引脚时钟

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //配置 USART1 的Tx 引脚类型为推挽式的
	GPIO_InitStructure.GPIO_Pin = COM2_TX_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(COM1_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //配置 USART1 的Rx 为输入悬空
	GPIO_InitStructure.GPIO_Pin = COM2_RX_PIN;
	GPIO_Init(COM1_GPIO_PORT, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = baudRate; //设置波特率为baudRate
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //设置数据位为8位
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //设置停止位为1位
	USART_InitStructure.USART_Parity = USART_Parity_No;   //无奇偶校验
	USART_InitStructure.USART_HardwareFlowControl =
	USART_HardwareFlowControl_None; //没有硬件流控
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //发送与接收

	USART_ITConfig(COM2_USART, USART_IT_RXNE, ENABLE); //接收中断使能
	USART_Init(COM2_USART, &USART_InitStructure);	    //串口2相关寄存器的配置
	USART_Cmd(COM2_USART, ENABLE);			    //使能串口2
}

static void USART3_Config(u32 baudRate) {
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_APB2PeriphClockCmd(COM3_RCC, ENABLE);       //使能 USART3 时钟
	RCC_APB2PeriphClockCmd(COM3_GPIO_RCC, ENABLE);  //使能串口3引脚时钟

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //配置 USART3 的Tx 引脚类型为推挽式的
	GPIO_InitStructure.GPIO_Pin = COM3_TX_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(COM3_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //配置 USART3 的Rx 为输入悬空
	GPIO_InitStructure.GPIO_Pin = COM3_RX_PIN;
	GPIO_Init(COM3_GPIO_PORT, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = baudRate; //设置波特率为baudRate
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //设置数据位为8位
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //设置停止位为1位
	USART_InitStructure.USART_Parity = USART_Parity_No;   //无奇偶校验
	USART_InitStructure.USART_HardwareFlowControl =
	USART_HardwareFlowControl_None; //没有硬件流控
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //发送与接收

	USART_ITConfig(COM3, USART_IT_IDLE, ENABLE); //接收中断使能
	USART_Init(COM3, &USART_InitStructure);	    //串口3相关寄存器的配置
	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
	USART_Cmd(COM3, ENABLE);			    //使能串口3
}

static void USART2_putc(uint8_t ch) {
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET) {
	}
	USART_SendData(USART2, (uint8_t) ch);
}

static void USART3_putc(uint8_t ch) {
	while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET) {
	}
	USART_SendData(USART3, (uint8_t) ch);
}

static void NVIC_Config(void) {

	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);  //设置中断向量表的基地址为0x08000000
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  //设置优先级分组：先占优先级2位,从优先级2位

	/*使能USART2中断*/
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn; //通道设置为串口1中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //中断占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //中断占优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;   //打开中断
	NVIC_Init(&NVIC_InitStructure);

	/*使能USART3中断*/
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn; //通道设置为串口1中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //中断占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //中断占优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;   //打开中断
	NVIC_Init(&NVIC_InitStructure);

}

void GPIO_Configuration(void) {

	/* Initialize Leds mounted on STM32 board */
	GPIO_InitTypeDef GPIO_InitStructure;
	/* Initialize LED which connected to PA1,2,3,4, Enable the Clock*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	/* Configure the GPIO_LED pin */

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6
			| GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

}

void Calendar_RTC_Init(void) {
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

	PWR_BackupAccessCmd(ENABLE);   //允许访问BKP数据
	BKP_DeInit();  		    //复位BPK
	RCC_LSEConfig(RCC_LSE_ON);  //打开LSE低速时钟
	while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
		;  //等待LSE稳定
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);  //选择LSE作为RTC的时钟源
	RCC_RTCCLKCmd(ENABLE);    //打开RTC时钟
	RTC_WaitForSynchro();      //等待RTC寄存器同步
	RTC_WaitForLastTask();     //等待最后一个写操作完成
	RTC_ITConfig(RTC_IT_SEC, ENABLE);     //打开RTC中断
	RTC_WaitForLastTask();     //等待最后写操作完成
	RTC_SetPrescaler(32767); //设置于分频，周期 period = RTCCLK/RTC_PR = (32.768kHz)/(32767 + 1) = 1s
	RTC_WaitForLastTask();     //等待最后的写操作完成
}

void USART3_DMA_Init() {
	DMA_InitTypeDef DMA_Initstructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/*开启DMA时钟*/
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	/*DMA配置*/
	DMA_DeInit(DMA1_Channel3);
	DMA_Initstructure.DMA_PeripheralBaseAddr = (u32) (&USART3->DR);
	DMA_Initstructure.DMA_MemoryBaseAddr = (u32) USART_Buf;
	DMA_Initstructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_Initstructure.DMA_BufferSize = 128;
	DMA_Initstructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_Initstructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_Initstructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_Initstructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_Initstructure.DMA_Mode = DMA_Mode_Normal;
	DMA_Initstructure.DMA_Priority = DMA_Priority_High;
	DMA_Initstructure.DMA_M2M = DMA_M2M_Disable;
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	DMA_Init(DMA1_Channel3, &DMA_Initstructure);

	//启动DMA
	DMA_Cmd(DMA1_Channel3, ENABLE);

}

void SPI2_Config() {
	//使能APB2上相关时钟
	//使能SPI时钟，使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB1Periph_SPI2 | RCC_APB2Periph_GPIOB, ENABLE);

	//定义一个GPIO结构体
	GPIO_InitTypeDef GPIO_InitStructure;

	//SPI SCK MOSI
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//推挽输出
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//SPI MISO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	//上拉输入
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//自定义SPI结构体
	SPI_InitTypeDef SPI_InitStructure;
	//双线双向全双工
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	//主机模式
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	//8位帧结构
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	//时钟空闲时为低
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	//第一个上升沿捕获数据。模式0,0
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	//MSS 端口软件控制，实际没有使用
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	//SPI时钟 72Mhz / 256 = 281.25K  < 400K
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	//数据传输高位在前
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;

	SPI_InitStructure.SPI_CRCPolynomial = 7;	//
	//初始化SPI2
	SPI_Init(SPI2, &SPI_InitStructure);
	//把使能SPI口的SS输出功能
	SPI_SSOutputCmd(SPI2, ENABLE);
	//使能SPI2
	SPI_Cmd(SPI2, ENABLE);
}

/**
 * @brief  	通过SPI2发送数据
 * @param    发送数据
 * @retval   返回数据
 */
uint8_t SPI2_SendByte(uint8_t byte) {
	//等待发送缓冲寄存器为空
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
		;
	//发送数据
	SPI_I2S_SendData(SPI2, byte);
	//等待接收缓冲寄存器为非空
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)
		;
	//返回从SPI通信中接收到的数据
	return SPI_I2S_ReceiveData(SPI2);
}

/**
 * @brief  	通过SPI2读取数据
 * @param    无
 * @retval   返回数据
 */
uint8_t SPI2_ReceiveByte() {
	//等待发送缓冲寄存器为空
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
		;
	//发送数据,通过发送0xff,获得返回数据
	SPI_I2S_SendData(SPI2, 0xff);
	//等待接收缓冲寄存器为非空
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)
		;
	//返回从SPI通信中接收到的数据
	return SPI_I2S_ReceiveData(SPI2);
}
