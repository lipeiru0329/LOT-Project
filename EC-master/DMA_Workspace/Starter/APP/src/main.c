#include "stm32f10x.h"
#include "xprintf.h"
#include "BSP.h"
#include "scheduler.h"
#include "stm32f10x_spi.h"
#include "string.h"
#include "stm32f10x_usart.h"
SEM USART3_Ready;
uint8_t flag = 0;
int main()
{
	printf("Start\n");
	RCC_Config();
	BSP_Init();
	InitTasks();
	Calendar_RTC_Init();
	Systick_Init();
	InitSem(USART3_Ready);
	u16 ms =5000;
	delay_nus(ms);

	int j;
	uint8_t Data[7] = {0xb4,0xc0,0xa8,0x01,0x01,0x00,0x1e};
	for(j=0;j<7;j++)
	{
		xprintf("%c",Data[j]);
	}

	while(!flag){}

	uint8_t Data_1[7] = {0xb0,0xc0,0xa8,0x01,0x01,0x00,0x1a};
	for(j=0;j<7;j++)
		{
			xprintf("%c",Data_1[j]);
		}


	while(1)
	{

	}
}

void SysTick_Handler(void)
{
	printf("1\n");
	UpdateTimers();
}

void USART3_IRQHandler(void)
{
    unsigned char num=0;
    if(USART_GetITStatus(USART3,USART_IT_IDLE) == SET)
    {
       num = USART3->SR;
       num = USART3->DR; //清USART_IT_IDLE标志
       DMA_Cmd(DMA1_Channel3,DISABLE);    //关闭DMA
       num = 128 -  DMA_GetCurrDataCounter(DMA1_Channel3);      //得到真正接收数据个数
       USART_Buf[num] = '\0';
       int i;
       for(i=0;i<num;i++)
       {
    	   printf("%x",USART_Buf[i]);
       }
       DMA1_Channel3->CNDTR=128;       //重新设置接收数据个数
       DMA_Cmd(DMA1_Channel3,ENABLE);  //开启DMA
       SendSem(USART3_Ready);           //接收数据标志位置1
       flag=1;
    }
}
