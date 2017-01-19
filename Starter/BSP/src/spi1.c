/**
  ******************************************************************************
  * @file     SPI1.c 
  * @author   xukai
  * @version  V0.1
  * @date     2012-3-29
  * @brief    初始化SPI1，SPI1读写基本操作       
  ******************************************************************************
  */  
/* Includes ------------------------------------------------------------------*/
#include "SPI1.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  	初始化SPI1
  * @param    无
  * @retval   无
  */
void SPI1_Config(void)
{
  //使能APB2上相关时钟
  //使能SPI时钟，使能GPIOA时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 |\
                         RCC_APB2Periph_GPIOA ,ENABLE );
    
  //定义一个GPIO结构体
  GPIO_InitTypeDef  GPIO_InitStructure; 
  
  //SPI SCK MOSI
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5  |  GPIO_Pin_7; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//复用推挽输出 
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
  
  //SPI MISO
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//上拉输入 
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
  
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
  
  SPI_InitStructure.SPI_CRCPolynomial = 7;//
  //初始化SPI1
  SPI_Init(SPI1, &SPI_InitStructure);
	//把使能SPI口的SS输出功能
	SPI_SSOutputCmd(SPI1,ENABLE);
  //使能SPI1 
  SPI_Cmd(SPI1, ENABLE); 
}

/**
  * @brief  	通过SPI1发送数据
  * @param    发送数据
  * @retval   返回数据
  */
uint8_t SPI1_SendByte(uint8_t byte)
{
  //等待发送缓冲寄存器为空
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
  //发送数据
  SPI_I2S_SendData(SPI1, byte);		
  //等待接收缓冲寄存器为非空
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
  //返回从SPI通信中接收到的数据
  return SPI_I2S_ReceiveData(SPI1);
}

/**
  * @brief  	通过SPI1读取数据
  * @param    无
  * @retval   返回数据
  */
uint8_t SPI1_ReceiveByte()
{
  //等待发送缓冲寄存器为空
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
  //发送数据,通过发送0xff,获得返回数据
  SPI_I2S_SendData(SPI1, 0xff);		
  //等待接收缓冲寄存器为非空
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
  //返回从SPI通信中接收到的数据
  return SPI_I2S_ReceiveData(SPI1);
}
/***************************************************************END OF FILE****/