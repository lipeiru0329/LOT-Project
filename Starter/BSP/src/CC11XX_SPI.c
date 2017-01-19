#include "stm32f10x.h"
#include "stm32f10x_spi.h"
#include "CC11XX_SPI.h"
//#include "CPU.h"

/************************************************************************
函数原型：void STM32_SPI2_Initialize( void )
输    入：无
输    出：无
描    述：SPI2(CC1101)配置函数
************************************************************************/
void STM32_SPI2_Initialize( void )
{
  SPI_InitTypeDef  SPI_InitStructure;
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2 ,ENABLE); 
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; 
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;

  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;

  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_Init(SPI2, &SPI_InitStructure); 
  SPI_Cmd(SPI2, ENABLE); 
}

void STM32_SPI2_Disable( void )
{
	SPI_SSOutputCmd(SPI2, DISABLE);
}

void SPI2_MasterTransmit( unsigned char cData )
{
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
//	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET);
	SPI_I2S_SendData(SPI2, cData);
}

