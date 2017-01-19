#include "stm32f10x.h"
#include "xprintf.h"
#include "BSP.h"
#include "am2301.h"
#include "scheduler.h"
#include "stm32f10x_it.h"
#include "antilib_gpio.h"
#include <stdio.h>

const uint8_t PMS5003S_port = 2;
char data[32];
int ME2_counter = 0;
int ME2_CollectDataFlag= 0;
int PMS_counter = 0;
int PMS_CollectDataFlag= 0;
int newdata_flag = 0;
int muxport = 1; //ME2 = 1; PMS = 2;
int ME2_dataID = 0;
int PMS_dataID = 0;
TM_AM2301_Data_t tempData;

/*

void ME2_CollectData();
void getData_ME2(void);
void PMS_CollectData();
void getData_PMS(void);
//char int_char(int n);
//void USART2_IRQHandler(void);
*/

char out[20];
int j = 0;
int check[100] = {0};
int n = 0;

// Scheduler
//Config
#define MAXTASKS 255
volatile unsigned char timers[MAXTASKS];

#define _SS static unsigned char _lc=0; switch(_lc){default:
#define _EE ;}; _lc=0; return 255;
#define WaitX(tickets) do {_lc=(__LINE__%255)+1; return tickets ;} while(0); case (__LINE__%255)+1:
#define RunTask(TaskName,TaskID) do { if (timers[TaskID]==0) timers[TaskID]=TaskName(); } while(0);
#define RunTaskA(TaskName,TaskID) { if (timers[TaskID]==0) {timers[TaskID]=TaskName(); continue;} } //前面的任务优先保证执行
#define CallSub(SubTaskName) do {unsigned char currdt; _lc=(__LINE__%255)+1; return 0; case (__LINE__%255)+1: currdt=SubTaskName();if(currdt!=255) return currdt;} while(0); //和上面是同一行，因为一行显示不下了，里面有 return 0，在执行子函数前释放了一下 CPU
#define InitTasks() {unsigned char i; for(i=MAXTASKS;i>0 ;i--) timers[i-1]=0; }
#define UpdateTimers() {unsigned char i; for(i=MAXTASKS;i>0 ;i--){if((timers[i-1]!=0)&&(timers[i-1]!=255)) timers[i-1]--;}}
#define SEM unsigned int
//初始化信号量
#define InitSem(sem) sem=0;
//等待信号量
#define WaitSem(sem) do{ sem=1; WaitX(0); if (sem>0) return 1;} while(0); //里面有 WaitX(0)，执行的时候释放了一下 CPU
//等待信号量或定时器溢出， 定时器 tickets 最大为 0xFFFE
#define WaitSemX(sem,tickets) do { sem=tickets+1; WaitX(0); if(sem>1){ sem--; return 1;} } while(0); //里面有 WaitX(0)，执行的时候释放了一下 CPU
//发送信号量
#define SendSem(sem) do {sem=0;} while(0);
//#define SEM unsigned int;//信号量的类型是 int，可以定义 65536 个，是个全局变量。
unsigned int sm1, sm2;


unsigned char task1(){
    _SS
    InitSem(sm1);
    while(1){
    	//WaitX(3);
        WaitSem(sm1);
        printf("Task1\n");//Function
        }
    _EE
}
unsigned char task2(){
    _SS
    while(1){
    	WaitSem(sm2);
        //WaitX(2);
        printf("Task2\n");
        SendSem(sm1);
        }
    _EE
}
unsigned char task3(){
    _SS
    while(1){
        WaitX(1);     //wait 1s
        printf("Task3\n");//Function1
        SendSem(sm2);
    }
    _EE
}

int ledPos = 0;                                   // current led position from 0..7
int ledDir = 1;
#define LED_BLUE_GPIO	GPIOC
#define LED_BLUE_PIN	8

void TIM1_UP_IRQHandler (void) {

  if ((TIM1->SR & 0x0001) != 0) {                 // check interrupt source

        if (ledPos == 0)                              // lower limit reached ?
          ledDir = 1;                                 // move up

        if (ledPos == 7)                              // upper limit reached ?
          ledDir = -1;                                // move down

        //GPIOB->ODR &= ~(1 << (ledPos+8));             // switch off old LED position
        printf("1\n");
        ledPos += ledDir;
        //GPIOB->ODR |=  (1 << (ledPos+8));             // switch on  new LED position

        TIM1->SR &= ~(1<<0);                          // clear UIF flag
 }
} // end TIM1_UP_IRQHandler

void Time_Config()
{
		RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPCEN;
		RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

		TIM3->PSC = 23999;	        // Set prescaler to 24 000 (PSC + 1)
		TIM3->ARR = 3000;	          // Auto reload value 1000
		TIM3->DIER = TIM_DIER_UIE; // Enable update interrupt (timer level)
		TIM3->CR1 = TIM_CR1_CEN;   // Enable timer

		NVIC_EnableIRQ(TIM3_IRQn); // Enable interrupt from TIM3 (NVIC level)
}

void TIM3_IRQHandler(void)
{
	if(TIM3->SR & TIM_SR_UIF) // if UIF flag is set
	  {
		TIM3->SR &= ~TIM_SR_UIF; // clear UIF flag
		printf("%d\n", n); // toggle LED state
		n++;
		UpdateTimers();
	  }
}

int main()
{
	InitTasks();
	Time_Config();
	while(1){
		RunTaskA(task3,1);
		RunTaskA(task2,2);
		RunTaskA(task1,3);
		//RunTaskA(task1,3);
	     //RunTaskA(task1,2);//任务 1 具有比任务 2 高的运行权限   没有WAITX(0)
	     //RunTaskA(task2,1);//任务 2 具有低的运行权限
	}
	//stm32_Init ();                                  // STM32 setup

	//GPIOB->ODR |=  (1 << (ledPos+8));
	/*
	uint16_t vo[7] = {0xB0, 0xC0, 0xA8, 0x01, 0x01, 0x00, 0x1A};  // B0 C0 A8 01 01 00 1A
	uint16_t cu[7] = {0xB1, 0xC0, 0xA8, 0x01, 0x01, 0x00, 0x1B};   //B1 C0 A8 01 01 00 1B
	uint16_t po[7] = {0xA2, 0xC0, 0xA8, 0x01, 0x01, 0x00, 0x42};   //A2 08 98 00 00 00 42
	uint16_t en[7] = {0xB3, 0xC0, 0xA8, 0x01, 0x01, 0x00, 0x1D};   //B3 C0 A8 01 01 00 1D
	int i = 0;
	int ch = 0;
	double num;
	BSP_Init();
	MUX_Select(3); // ME2(3) PMS(1)
	printf("Start\n");*/

	/*for(i = 0 ; i < 1000 ; i++)                 //delay 1s
	{
		Delay_1ms();
	}*/


	/*
	for(i = 0 ; i < 7 ; i++)                 // Send USART ch = 0 when require current and voltage;
	{										  // ch = 1 when require power and energy
		ch = 0;
		printf("%x ", vo[i]);
		xprintf("%c", vo[i]);
		//USART2_IRQHandler();
	}
	printf("\n");
	//USART2_IRQHandler();
	for(i = 0 ; i < 14 ; i++)
		{
			printf("%x ", check[i]);
		}
	if (ch == 0)
		num = check[1] * 256 + check[2] * 256 + check[3]/100;                 // get final output
	else
		num = check[1] * 65536 + check[2] * 256 + check[3];					  // get final output
	j = 0;
	*/



	/*while(1){
		//data[0] = USART_ReceiveData(USART3);
		//if(data[0]!= 58 && data[0]!=data_prev[0])
		//{
		//xprintf("%04x\n",data[0]);
		//data_prev[0] = data[0];
		//}
		if(TM_AM2301_Read(&tempData) == TM_AM2301_OK)
		{
			printf("H:%d T:%d C\n", tempData.Hum, tempData.Temp);
		}
		Delay_nus(1000);
		getData_ME2();
		getData_PMS();
	}*/
}






char int_char(int n)
{

	if(n >= 48 && n <= 57)
		return '0' + n - 48;
	else if(n >= 97 && n <= 122)
		return 'a' + n - 97;

	else if(n >= 65 && n <= 90)
		return 'A' + n - 65;
	else return n;
}


void USART2_IRQHandler(void)
{
	printf("UART2\n%d\n", j);
//    unsigned char num=0;
   // if(USART_GetITStatus(USART2,USART_IT_RXNE) != RESET)
   // {
    	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    		{
   				//printf("\nReceive\n");
    			check[j] = USART_ReceiveData(USART2);
    			//printf("%s\n", (int)USART_ReceiveData(USART2));
    			//printf("%d\n", (int)USART_ReceiveData(USART2));
    			//Delay_1us();
    			out[j] = int_char(USART_ReceiveData(USART2));
    			j++;
    			//USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET;
    			//USART_SendData(USART2,USART_ReceiveData(USART2));//发送收到的字节
    			//while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);//等待发送完成
    		}

//       num = USART2->SR;
//       num = USART2->DR; //清USART_IT_IDLE标志
//       DMA_Cmd(DMA1_Channel5,DISABLE);    //关闭DMA
//       num = 128 -  DMA_GetCurrDataCounter(DMA1_Channel5);      //得到真正接收数据个数
//
//       USART_Buf[num] = '\0';
//       DMA1_Channel3->CNDTR=128;       //重新设置接收数据个数
//       DMA_Cmd(DMA1_Channel5,ENABLE);  //开启DMA
//       SendSem(USART2_Ready);           //接收数据标志位置1
//       flag = 1;
//       //USART_ITConfig(COM3, USART_IT_IDLE, DISABLE);

    	//printf("%x ",USART_ReceiveData(USART2));
    	//check[j] = USART_ReceiveData(USART2);
    	//out[j] = int_char(USART_ReceiveData(USART2));
    	//j++;

    	//xprintf("%c",(char)USART_ReceiveData(USART2));
    			//flag = 1;

    //}
}
/*void USART2_IRQHandler(void)
{
	//for(j = 0 ; j < 7 ; j++)
	//{
	//printf("\nReceive1111\n");
				if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
				{
					//printf("\nReceive\n");
					check[j] = USART_ReceiveData(USART2);
					printf(" %x ", USART_ReceiveData(USART2));

					//Delay_1us();
					out[j] = int_char(USART_ReceiveData(USART2));
					j++;
					//USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET;
					//USART_SendData(USART2,USART_ReceiveData(USART2));//发送收到的字节
					while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);//等待发送完成
				}

	//}

}*/

void getData_ME2(void)
{
	int k;
	double co_concentration;
	unsigned char checksum =0;
	if(ME2_CollectDataFlag == 1)
	{
			for(k = 1;k <= 7 ;k++)
			{
				checksum = checksum+data[k];
			}
			checksum = (~checksum)+1;

			if(data[8]==checksum)
			{
				co_concentration = (data[4]*256 + data[6])*0.1;
				ME2_counter =0;
				muxport=2;
				ME2_CollectDataFlag=0;
				MUX_Select(1);
				for(k=0;k<=8;k++)
								{
									data[k] = '\0';
								}

			}
			else
			{
				ME2_counter =0;
				ME2_CollectDataFlag=0;
				for(k=0;k<=8;k++)
				{
					data[k] = '\0';
				}
				return;
			}
			ME2_dataID++;
			printf("CO concentration: %lf [%d]\n", co_concentration,ME2_dataID);
	}
	return;
}

void getData_PMS(void)
{
		int j;
		uint16_t PM2_5 = 0;
		uint16_t checksum_from_data;
		uint16_t checksum =0;
		uint16_t Formaldehyde=0;
		//double Formaldehyde_mg;
		if(PMS_CollectDataFlag == 1)
		{
				for(j = 0;j <= 29 ;j++)
				{
					checksum = checksum+data[j];
				}
				checksum_from_data = data[30]<<8;
				checksum_from_data = checksum_from_data +data[31];
				if(checksum_from_data==checksum)
				{
					PM2_5 = data[6]<<8;
					PM2_5 = PM2_5+data[7];
					Formaldehyde =data[28]<<8;
					Formaldehyde = Formaldehyde + data[29];
					PMS_counter =0;
					muxport=1;
					PMS_CollectDataFlag=0;
					MUX_Select(3);
					for(j=0;j<=31;j++)
									{
										data[j] = '\0';
									}

				}
				else
				{
					PMS_counter =0;
					PMS_CollectDataFlag=0;
					for(j=0;j<=31;j++)
					{
						data[j] = '\0';
					}
					return;
				}
				PMS_dataID++;
				printf("PM2.5: %04x [%d]\n", PM2_5,PMS_dataID);
				printf("Formaldehyde: %04x [%d]\n",Formaldehyde,PMS_dataID);
		}
		return;

}

void ME2_CollectData()
{
	//wait for start byte
	if(USART_ReceiveData(USART3) == 0xff && muxport ==1 && ME2_counter == 0 && ME2_CollectDataFlag ==0)
	{
		data[0] = USART_ReceiveData(USART3);
		ME2_counter++;
		return;
	}
	//collect data after recognizing start byte
	if(data[0] == 0xff && ME2_counter <=8 && muxport ==1 && ME2_CollectDataFlag ==0)
	{
	data[ME2_counter]= USART_ReceiveData(USART3);
	ME2_counter++;
	}
	//indicate that the data is ready
	if(ME2_counter > 8 && muxport == 1 && ME2_CollectDataFlag == 0)
	{
		ME2_CollectDataFlag =1;
	}
}

void PMS_CollectData()
{
	//wait for 1st start byte
	if(USART_ReceiveData(USART3) == 0x42 && muxport ==2 && PMS_counter ==0 && PMS_CollectDataFlag ==0)
	{
		data[0] = USART_ReceiveData(USART3);
		PMS_counter ++;
		return;
	}

	//check for the 2nd start byte
	if(data[0]==0x42 && data[1]!= 0x4d && muxport ==2 && PMS_CollectDataFlag ==0 && PMS_counter > 1)
	{
		data[0] = '\0';
		data[1] = '\0';
		PMS_counter = 0;
	}

	//collect data
	if(data[0] == 0x42 && PMS_counter <=31 && muxport == 2 && PMS_CollectDataFlag ==0)
	{
		data[PMS_counter] = USART_ReceiveData(USART3);
		PMS_counter++;
	}

	//indicate data ready for collection
	if(PMS_counter >31 && muxport==2 && PMS_CollectDataFlag ==0)
	{
		PMS_CollectDataFlag =1;
	}
}
