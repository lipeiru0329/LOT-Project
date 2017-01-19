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
#include "am2301.h"
void Delay_1us(void)  //延时1us
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
void Delay_nus(u32 x) {
	u16 i = 0;
	for (i = 0; i < x; i++) {
		Delay_1us();
	}
}
void Delay_1ms(void) {
	u16 i = 0;
	for (i = 0; i < 6500; i++) {
		;
	}
}

void BSP_Init(void) {
	RCC_APB2PeriphClockCmd(
			RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC
					| RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE
					| RCC_APB2Periph_GPIOF | RCC_APB2Periph_GPIOG, ENABLE);
	USART2_Config(9600);
	USART3_Config(9600);
	MUX_Init();
	MUX_Select(3);
	NVIC_Config();
	xdev_out(USART2_putc);
	GPIO_Configuration();
	//SPI_Configuration();
	//LCD_Init();
	STM32_SPI2_Initialize();
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

	USART_ITConfig(COM3, USART_IT_RXNE, ENABLE); //接收中断使能
	USART_Init(COM3, &USART_InitStructure);	    //串口3相关寄存器的配置
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
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  //设置优先级分组：先占优先级0位,从优先级4位

	/*使能USART2中断*/
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn; //通道设置为串口1中断
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //中断占优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;   //打开中断
	NVIC_Init(&NVIC_InitStructure);

	/*使能USART2中断*/
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn; //通道设置为串口1中断
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //中断占优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;   //打开中断
	NVIC_Init(&NVIC_InitStructure);

}

void GPIO_Configuration(void)
{

    /* Initialize Leds mounted on STM32 board */
    GPIO_InitTypeDef  GPIO_InitStructure;
    /* Initialize LED which connected to PA1,2,3,4, Enable the Clock*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    /* Configure the GPIO_LED pin */


    GPIO_InitStructure.GPIO_Pin =GPIO_Pin_4 | GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //Testing for GPIOB_5
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    /* Configure the GPIO_LED pin */

    GPIO_InitStructure.GPIO_Pin =GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

}

void SPI_Configuration(void)
{
	  SPI_InitTypeDef  SPI_InitStructure;
	  GPIO_InitTypeDef GPIO_InitStructure;


	  RCC_APB2PeriphClockCmd( RCC_APB2Periph_SPI1 | RCC_APB2Periph_OLED_PORT, ENABLE);

	  /* NSS---->GPIO(LED) */
	  SPI_SSOutputCmd(SPI1, ENABLE);

	  GPIO_InitStructure.GPIO_Pin = OLED_SCK_PIN | OLED_SDA_PIN ;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(OLED_PORT, &GPIO_InitStructure);

	  /* GPIO4 LED3 */
	  GPIO_InitStructure.GPIO_Pin = OLED_RST_PIN | OLED_DC_PIN | GPIO_Pin_4 ;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(OLED_PORT, &GPIO_InitStructure);

	  /* SPI1 Config -------------------------------------------------------------*/
	  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
	  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	  SPI_InitStructure.SPI_CRCPolynomial = 7;
	  SPI_Init(SPI1, &SPI_InitStructure);
	  /* Enable SPI1 */
	  SPI_Cmd(SPI1, ENABLE);
}

void OLED_WB(uint8_t data)
{
    /* Loop while DR register in not emplty */
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
    /* Send byte through the SPI2 peripheral */
    SPI_I2S_SendData(SPI1, data);
}
/*******************һ���ֽ�����д��***********************/
void LCD_WrDat(unsigned char dat)
{
    OLED_DC_H;
	OLED_WB(dat);
}

/********************һ��ָ��д��**********************/
void LCD_WrCmd(unsigned char cmd)
{
    OLED_DC_L;
    OLED_WB(cmd);
}

/**********************������ʾλ��**********************/
void LCD_Set_Pos(unsigned char x, unsigned char y)
{
	/* Page addressing mode */
    LCD_WrCmd(0xb0+(y&0x07));/* set page start address */
    LCD_WrCmd(x&0x0f);/* set lower nibble of the column address */
    LCD_WrCmd(((x&0xf0)>>4)|0x10); /* set higher nibble of the column address */
}

/**********************д��������**********************/
void LCD_Fill(unsigned char bmp_dat)
{
	unsigned char y,x;

    LCD_WrCmd(0x20);//-Set Page Addressing Mode (0x00/0x01/0x02)
    LCD_WrCmd(0x00);//

	LCD_WrCmd(0x21);//-Set Column Address
	LCD_WrCmd(0x00);
	LCD_WrCmd(0x7f);

	LCD_WrCmd(0x22);//-Set Page Address
	LCD_WrCmd(0x00);
	LCD_WrCmd(0x07);

	LCD_DLY_ms(1);/* �ȴ��ڲ��ȶ�   */

    for(y=0;y<Page;y++)
    {
    	for(x=0;x<X_WIDTH;x++)
    	{
    		LCD_WrDat(bmp_dat);
    	}
    }
//    LCD_WrCmd(0xaf);//--turn off oled panel
}

/*********************��������***********************/
void LCD_CLS(void)
{
	unsigned char y,x;
	for(y=0;y<8;y++)
	{
		LCD_WrCmd(0xb0+y);
		LCD_WrCmd(0x01);
		LCD_WrCmd(0x10);
		for(x=0;x<X_WIDTH;x++)
		LCD_WrDat(0);
		LCD_DLY_ms(200);
	}
}

/*********************��ʱ����***********************/
void LCD_DLY_ms(unsigned int ms)
{
    unsigned int a;
    while(ms)
    {
        a=1335;
        while(a--);
        ms--;
    }
    return;
}


/*********************12864��ʼ��***********************/
void LCD_Init(void)
{
	OLED_RST_L;
	LCD_DLY_ms(50);
	OLED_RST_H;
	//���ϵ絽���濪ʼ��ʼ��Ҫ���㹻��ʱ�䣬���ȴ�RC��λ����

    LCD_WrCmd(0xae);//--turn off oled panel

    LCD_WrCmd(0xa8);//--set multiplex ratio(1 to 64)
    LCD_WrCmd(0x3f);//--1/64 duty
    LCD_WrCmd(0xd3);//-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
    LCD_WrCmd(0x00);//-not offset
    LCD_WrCmd(0x40);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
    LCD_WrCmd(0xa0);//--Set SEG/Column Mapping     0xa0���ҷ��� 0xa1����
    LCD_WrCmd(0xc0);//Set COM/Row Scan Direction   0xc0���·��� 0xc8����
    LCD_WrCmd(0xda);//--set com pins hardware configuration
    LCD_WrCmd(0x12);
    LCD_WrCmd(0x81);//--set contrast control register
    LCD_WrCmd(0xcf); // Set SEG Output Current Brightness
    LCD_WrCmd(0xa4);// Disable Entire Display On (0xa4/0xa5)
    LCD_WrCmd(0xa6);// Disable Inverse Display On (0xa6/a7)
    LCD_WrCmd(0xd5);//--set display clock divide ratio/oscillator frequency
    LCD_WrCmd(0x80);//--set divide ratio, Set Clock as 100 Frames/Sec
    LCD_WrCmd(0x8d);//--set Charge Pump enable/disable
    LCD_WrCmd(0x14);//--set(0x10) disable
    LCD_WrCmd(0xaf);//--turn on oled panel

    LCD_WrCmd(0xd9);//--set pre-charge period
    LCD_WrCmd(0xf8);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock

    LCD_WrCmd(0xdb);//--set vcomh
    LCD_WrCmd(0x40);//Set VCOM Deselect Level

    LCD_Fill(0x00);  //��ʼ����
}
