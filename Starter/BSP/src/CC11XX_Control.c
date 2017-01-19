#include "stm32f10x.h"
#include "stm32f10x_spi.h"
#include "CC11XX_SPI.h"
#include "CC11XX_Control.h"
//#include "CPU.h"

const RF_SETTINGS rfSettings =
{
	0x00,
	0x08, // FSCTRL1 Frequency synthesizer control.

	0x00, // FSCTRL0 Frequency synthesizer control.
	0x10, //0x1e,// FREQ2 Frequency control word, high byte.
	0xA7, //0xc4,// FREQ1 Frequency control word, middle byte.
	0x62, //0xec,// FREQ0 Frequency control word, low byte.

/*	0x5B,  //0x2D, // MDMCFG4 Modem configuration.
	0xF8,  //0x3B, // MDMCFG3 Modem configuration.
	0x02,   //0x73, // MDMCFG2 Modem configuration.
	0x22, // MDMCFG1 Modem configuration.
	0xF8, // MDMCFG0 Modem configuration.  */
	0x2D,  //0x2D, // MDMCFG4 Modem configuration.
	0x3b,//0x3B,  //0x3B, // MDMCFG3 Modem configuration.
	0x73,   //0x73, // MDMCFG2 Modem configuration.
	0x22, // MDMCFG1 Modem configuration.
	0xF8, // MDMCFG0 Modem configuration.
	0x00, // CHANNR Channel number.
	0x47, // DEVIATN Modem deviation setting (when FSK modulation is enabled).

	0xB6, // FREND1 Front end RX configuration.
	0x10, // FREND0 Front end RX configuration.
	0x18, // MCSM0 Main Radio Control State Machine configuration.
	0x1D, // FOCCFG Frequency Offset Compensation Configuration.
	0x1C, // BSCFG Bit synchronization Configuration.
	0x07, // AGCCTRL2 AGC control.
	0x00, //0x00, // AGCCTRL1 AGC control.
	0xB2, // AGCCTRL0 AGC control.
	0xEA, // FSCAL3 Frequency synthesizer calibration.

	0x2A, // FSCAL2 Frequency synthesizer calibration.

	0x00, // FSCAL1 Frequency synthesizer calibration.
	0x1F, // FSCAL0 Frequency synthesizer calibration.
	0x59, // FSTEST Frequency synthesizer calibration.
	0x88, // TEST2 Various test settings.
	0x31, // TEST1 Various test settings.
	0x0B, // TEST0 Various test settings.
	0x0B, // IOCFG2 GDO2 output pin configuration.
	0x06, // IOCFG0D GDO0 output pin configuration.
	0x05, // PKTCTRL1 Packet automation control.
	0x05, // PKTCTRL0 Packet automation control.
//	0x34, // ADDR Device address.
	0x01, // ADDR Device address.
	0xFF // PKTLEN Packet length.
};
/*************************************************************************/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : SPI_FLASH_Init
* Description    : Initializes the peripherals used by the SPI FLASH driver.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_CC1101_Init(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;


	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA , ENABLE);

	/* Configure SPI2 pins: SCK (PA6), MISO (PA7) and MOSI(PB8) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_SCLK | GPIO_Pin_SO;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_SI;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Configure I/O for Flash Chip select */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_CS; //PA5
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIO_CS, &GPIO_InitStructure);

	/* Configure I/O for PB9,PB10  */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_GD2| GPIO_Pin_GD0; //PB9|PB10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	/* Deselect the FLASH: Chip Select high */
	//SPI_FLASH_CS_HIGH();

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15|GPIO_Pin_14 ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  // �����������
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Enable SPI2 and GPIO clocks */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	/* SPI2 configuration */
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;//SPI_Direction_2Lines_RxOnly;//SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI2, &SPI_InitStructure);

	/* Enable SPI2  */
	SPI_Cmd(SPI2, ENABLE);
}

/**********************  *********************************************************
* Function Name  : SPI_FLASH_ReadByte
* Description    : Reads a byte from the SPI Flash.
*                  This function must be used only if the Start_Read_Sequence
*                  function has been previously called.
* Input          : None
* Output         : None
* Return         : Byte Read from the SPI Flash.
*******************************************************************************/
u8 SPI_FLASH_ReadByte(void)
{
  return (SPI_FLASH_SendByte(Dummy_Byte));
}

/*******************************************************************************
* Function Name  : SPI_FLASH_SendByte
* Description    : Sends a byte through the SPI interface and return the byte
*                  received from the SPI bus.
* Input          : byte : byte to send.
* Output         : None
* Return         : The value of the received byte.
*******************************************************************************/
u8 SPI_FLASH_SendByte(u8 byte)
{
  /* Loop while DR register in not emplty */
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);

  /* Send byte through the SPI2 peripheral */
  SPI_I2S_SendData(SPI2, byte);

  /* Wait to receive a byte */
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);

  /* Return the byte read from the SPI bus */
  return SPI_I2S_ReceiveData(SPI2);
}

/**********************************CC1101********************/

void Delay(vu32 nCount)
{
  int i,j;
  for(j=0;j<nCount;j++)
  {
     for(i=0;i<10;i++);
  }
}


u8 SPI_CC1101_ReadID(void)
{
	 u8 id;
	 SPI_FLASH_CS_LOW();
	 SPI_FLASH_SendByte(0x30);
	 id = SPI_FLASH_SendByte(0xff);
	 //SPI_FLASH_SendByte(0x31|0xc0);
	 SPI_FLASH_SendByte(0x31);
	 id = SPI_FLASH_SendByte(0xff);
	 SPI_FLASH_CS_HIGH();

	 return id;
}

void CC1101_POWER_RESET(void)
{

 /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();
  GPIO_SetBits(GPIOA, GPIO_Pin_SCLK); //SCLK=1
  GPIO_ResetBits(GPIOB, GPIO_Pin_SI); //SI=0
  /* Select the FLASH: Chip Select low */
  Delay(5000);
  SPI_FLASH_CS_HIGH();
  Delay(1);
  SPI_FLASH_CS_LOW();
  Delay(1);
  SPI_FLASH_CS_HIGH();
  Delay(41);
  SPI_FLASH_CS_LOW();
  while (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_SO) );//waite SO =0
  SPI_FLASH_SendByte(CCxxx0_SRES);
  while (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_SO) );//waite SO =0 again
  SPI_FLASH_CS_HIGH();
}

//*****************************************************************************************
//��������void halSpiWriteReg(u8 addr, u8 value)
//���룺��ַ��������
//�������
//����������SPIд�Ĵ���
//*****************************************************************************************
void halSpiWriteReg(u8 addr, u8 value)
{
    SPI_FLASH_CS_LOW();
    while (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_SO) );
    SPI_FLASH_SendByte(addr);		//д��ַ
    SPI_FLASH_SendByte(value);		//д������
    SPI_FLASH_CS_HIGH();
}

//*****************************************************************************************
//��������void halSpiWriteBurstReg(u8 addr, u8 *buffer, u8 count)
//���룺��ַ��д�뻺������д�����
//�������
//����������SPI����д���üĴ���
//*****************************************************************************************
void halSpiWriteBurstReg(u8 addr, u8 *buffer, u8 count)
{
    u8 i, temp;
	temp = addr | WRITE_BURST;
    SPI_FLASH_CS_LOW();
    while (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_SO) );
    SPI_FLASH_SendByte(temp);
    for (i = 0; i < count; i++)
 	{
        SPI_FLASH_SendByte(buffer[i]);
    }
    SPI_FLASH_CS_HIGH();
}

//*****************************************************************************************
//��������void halSpiStrobe(u8 strobe)
//���룺����
//�������
//����������SPIд����
//*****************************************************************************************
void halSpiStrobe(u8 strobe)
{
    SPI_FLASH_CS_LOW();
    while (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_SO) );
    SPI_FLASH_SendByte(strobe);		//д������
    SPI_FLASH_CS_HIGH();
}

//*****************************************************************************************
//��������void halRfSendPacket(INT8U *txBuffer, INT8U size)
//���룺���͵Ļ��������������ݸ���
//�������
//����������CC1100����һ������
//*****************************************************************************************

void halRfSendPacket(u8 *txBuffer, u8 size)
{

	//halSpiWriteReg(CCxxx0_TXFIFO, size); //д�볤��
	//halSpiWriteReg(CCxxx0_TXFIFO, 0x12);//д����ܵ�ַ
    halSpiWriteBurstReg(CCxxx0_TXFIFO, txBuffer, size);	//д��Ҫ���͵�����

    halSpiStrobe(CCxxx0_STX);		//���뷢��ģʽ��������

    // Wait for GDO0 to be set -> sync transmitted
    while (!GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_GD0) );//while (!GDO0);
    // Wait for GDO0 to be cleared -> end of packet
    while (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_GD0) );// while (GDO0);
	halSpiStrobe(CCxxx0_SFTX);
//	*PC11 ^= 1	;
	//*PA3 ^= 1	;
}

//*****************************************************************************************
//��������u8 halSpiReadReg(u8 addr)
//���룺��ַ
//������üĴ�����������
//����������SPI���Ĵ���
//*****************************************************************************************
u8 halSpiReadReg(u8 addr)
{
	u8 temp, value;
    temp = addr|READ_SINGLE;//���Ĵ�������
	SPI_FLASH_CS_LOW();
	while (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_SO) );//MISO
	SPI_FLASH_SendByte(temp);
	value = SPI_FLASH_SendByte(0);
	 SPI_FLASH_CS_HIGH();
	return value;
}

//*****************************************************************************************
//��������void halSpiReadBurstReg(u8 addr, u8 *buffer, u8 count)
//���룺��ַ���������ݺ��ݴ�Ļ��������������ø���
//�������
//����������SPI����д���üĴ���
//*****************************************************************************************
void halSpiReadBurstReg(u8 addr, u8 *buffer, u8 count)
{
    u8 i,temp;
	temp = addr | READ_BURST;		//д��Ҫ�������üĴ�����ַ�Ͷ�����
    SPI_FLASH_CS_LOW();
     while (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_SO));
	SPI_FLASH_SendByte(temp);
    for (i = 0; i < count; i++)
	{
        buffer[i] = SPI_FLASH_SendByte(0);
    }
    SPI_FLASH_CS_HIGH();
}

//*****************************************************************************************
//��������u8 halSpiReadReg(u8 addr)
//���룺��ַ
//�������״̬�Ĵ�����ǰֵ
//����������SPI��״̬�Ĵ���
//*****************************************************************************************
u8 halSpiReadStatus(u8 addr)
{
    u8 value,temp;
	temp = addr | READ_BURST;		//д��Ҫ����״̬�Ĵ����ĵ�ַͬʱд�������
    SPI_FLASH_CS_LOW();
    while (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_SO) );
    SPI_FLASH_SendByte(temp);
	value = SPI_FLASH_SendByte(0);
	SPI_FLASH_CS_HIGH();
	return value;
}

u8 halRfReceivePacket(u8 *rxBuffer, u8 *length)
{
    u8 status[2];
    u8 packetLength;
	u8 address;
	u8 i=(*length)*4;  // �������Ҫ����datarate��length������

    halSpiStrobe(CCxxx0_SRX);		//�������״̬
	//delay(5);
    //while (!GDO1);
    //while (GDO1);
	Delay(5);
	while (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_GD0) )//while (GDO0)
	{
		Delay(2);
		--i;
		if(i<1)
		   return 0;
	}
    if ((halSpiReadStatus(CCxxx0_RXBYTES) & BYTES_IN_RXFIFO)) //����ӵ��ֽ�����Ϊ0
	{
	    packetLength = halSpiReadReg(CCxxx0_RXFIFO);//������һ���ֽڣ����ֽ�Ϊ��֡���ݳ���

        if(packetLength <= *length) 		//�����Ҫ����Ч���ݳ���С�ڵ��ڽ��յ������ݰ��ĳ���
		{
            halSpiReadBurstReg(CCxxx0_RXFIFO, rxBuffer, packetLength); //�������н��յ�������
            *length = packetLength;				//�ѽ������ݳ��ȵ��޸�Ϊ��ǰ���ݵĳ���

            // Read the 2 appended status bytes (status[0] = RSSI, status[1] = LQI)
            halSpiReadBurstReg(CCxxx0_RXFIFO, status, 2); 	//����CRCУ��λ
			halSpiStrobe(CCxxx0_SFRX);		//��ϴ���ջ�����
            return (status[1] & CRC_OK);			//���У��ɹ����ؽ��ճɹ�
        }
		 else
		{
            *length = packetLength;
            halSpiStrobe(CCxxx0_SFRX);		//��ϴ���ջ�����
            return 0;
        }
    }
	else
 	return 0;
}

//*****************************************************************************************
//��������void halRfWriteRfSettings(RF_SETTINGS *pRfSettings)
//���룺��
//�������
//��������������CC1100�ļĴ���
//*****************************************************************************************
void halRfWriteRfSettings(void)
{

	halSpiWriteReg(CCxxx0_FSCTRL0,  rfSettings.FSCTRL2);//���Ѽӵ�
    // Write register settings
    halSpiWriteReg(CCxxx0_FSCTRL1,  rfSettings.FSCTRL1);
    halSpiWriteReg(CCxxx0_FSCTRL0,  rfSettings.FSCTRL0);
    halSpiWriteReg(CCxxx0_FREQ2,    rfSettings.FREQ2);
    halSpiWriteReg(CCxxx0_FREQ1,    rfSettings.FREQ1);
    halSpiWriteReg(CCxxx0_FREQ0,    rfSettings.FREQ0);
    halSpiWriteReg(CCxxx0_MDMCFG4,  rfSettings.MDMCFG4);
    halSpiWriteReg(CCxxx0_MDMCFG3,  rfSettings.MDMCFG3);
    halSpiWriteReg(CCxxx0_MDMCFG2,  rfSettings.MDMCFG2);
    halSpiWriteReg(CCxxx0_MDMCFG1,  rfSettings.MDMCFG1);
    halSpiWriteReg(CCxxx0_MDMCFG0,  rfSettings.MDMCFG0);
    halSpiWriteReg(CCxxx0_CHANNR,   rfSettings.CHANNR);
    halSpiWriteReg(CCxxx0_DEVIATN,  rfSettings.DEVIATN);
    halSpiWriteReg(CCxxx0_FREND1,   rfSettings.FREND1);
    halSpiWriteReg(CCxxx0_FREND0,   rfSettings.FREND0);
    halSpiWriteReg(CCxxx0_MCSM0,   rfSettings.MCSM0);
    halSpiWriteReg(CCxxx0_FOCCFG,   rfSettings.FOCCFG);
    halSpiWriteReg(CCxxx0_BSCFG,    rfSettings.BSCFG);
    halSpiWriteReg(CCxxx0_AGCCTRL2, rfSettings.AGCCTRL2);
	halSpiWriteReg(CCxxx0_AGCCTRL1, rfSettings.AGCCTRL1);	//
    halSpiWriteReg(CCxxx0_AGCCTRL0, rfSettings.AGCCTRL0);
    halSpiWriteReg(CCxxx0_FSCAL3,   rfSettings.FSCAL3);
	halSpiWriteReg(CCxxx0_FSCAL2,   rfSettings.FSCAL2);
	halSpiWriteReg(CCxxx0_FSCAL1,   rfSettings.FSCAL1);  ///
    halSpiWriteReg(CCxxx0_FSCAL0,   rfSettings.FSCAL0);
    halSpiWriteReg(CCxxx0_FSTEST,   rfSettings.FSTEST);
    halSpiWriteReg(CCxxx0_TEST2,    rfSettings.TEST2);
    halSpiWriteReg(CCxxx0_TEST1,    rfSettings.TEST1);
    halSpiWriteReg(CCxxx0_TEST0,    rfSettings.TEST0);
    halSpiWriteReg(CCxxx0_IOCFG2,   rfSettings.IOCFG2);
    halSpiWriteReg(CCxxx0_IOCFG0,   rfSettings.IOCFG0);
    halSpiWriteReg(CCxxx0_PKTCTRL1, rfSettings.PKTCTRL1);
    halSpiWriteReg(CCxxx0_PKTCTRL0, rfSettings.PKTCTRL0);
    halSpiWriteReg(CCxxx0_ADDR,     rfSettings.ADDR);
    halSpiWriteReg(CCxxx0_PKTLEN,   rfSettings.PKTLEN);
}

/*void R_S_Byte(u8 R_Byte)
{
	 SBUF = R_Byte;
	 /*while( TI == 0 );				//��ѯ��	sbit TI    = SCON^1;
  	 TI = 0;*/
     /*while( (SCON & 0x02) == 0 );				//��ѯ��
  	 SCON |= 0xDF;

}*/

/*************************************************************/

