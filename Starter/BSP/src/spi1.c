/**
  ******************************************************************************
  * @file     SPI1.c 
  * @author   xukai
  * @version  V0.1
  * @date     2012-3-29
  * @brief    ��ʼ��SPI1��SPI1��д��������       
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
  * @brief  	��ʼ��SPI1
  * @param    ��
  * @retval   ��
  */
void SPI1_Config(void)
{
  //ʹ��APB2�����ʱ��
  //ʹ��SPIʱ�ӣ�ʹ��GPIOAʱ��
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 |\
                         RCC_APB2Periph_GPIOA ,ENABLE );
    
  //����һ��GPIO�ṹ��
  GPIO_InitTypeDef  GPIO_InitStructure; 
  
  //SPI SCK MOSI
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5  |  GPIO_Pin_7; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//����������� 
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
  
  //SPI MISO
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//�������� 
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
  
  //�Զ���SPI�ṹ��
  SPI_InitTypeDef SPI_InitStructure;
  //˫��˫��ȫ˫��
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; 
	//����ģʽ
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master; 
  //8λ֡�ṹ
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b; 
  //ʱ�ӿ���ʱΪ��
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;        
  //��һ�������ز������ݡ�ģʽ0,0
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;      
  //MSS �˿�������ƣ�ʵ��û��ʹ��
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;         
  //SPIʱ�� 72Mhz / 256 = 281.25K  < 400K
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256; 
  //���ݴ����λ��ǰ
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; 
  
  SPI_InitStructure.SPI_CRCPolynomial = 7;//
  //��ʼ��SPI1
  SPI_Init(SPI1, &SPI_InitStructure);
	//��ʹ��SPI�ڵ�SS�������
	SPI_SSOutputCmd(SPI1,ENABLE);
  //ʹ��SPI1 
  SPI_Cmd(SPI1, ENABLE); 
}

/**
  * @brief  	ͨ��SPI1��������
  * @param    ��������
  * @retval   ��������
  */
uint8_t SPI1_SendByte(uint8_t byte)
{
  //�ȴ����ͻ���Ĵ���Ϊ��
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
  //��������
  SPI_I2S_SendData(SPI1, byte);		
  //�ȴ����ջ���Ĵ���Ϊ�ǿ�
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
  //���ش�SPIͨ���н��յ�������
  return SPI_I2S_ReceiveData(SPI1);
}

/**
  * @brief  	ͨ��SPI1��ȡ����
  * @param    ��
  * @retval   ��������
  */
uint8_t SPI1_ReceiveByte()
{
  //�ȴ����ͻ���Ĵ���Ϊ��
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
  //��������,ͨ������0xff,��÷�������
  SPI_I2S_SendData(SPI1, 0xff);		
  //�ȴ����ջ���Ĵ���Ϊ�ǿ�
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
  //���ش�SPIͨ���н��յ�������
  return SPI_I2S_ReceiveData(SPI1);
}
/***************************************************************END OF FILE****/