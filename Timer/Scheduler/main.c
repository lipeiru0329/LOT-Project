/************************************************************
***********************************************************/
#include <stc89c51.h>
#include <stdio.h>

#define  _SS   static char lc=0; switch(lc){   case 0: lc=0;
#define  _EE   }; lc=0;
#define  WaitX(a,b)  settimer(&lc,__LINE__,a,b); return ; case __LINE__:

struct TASK {
  char td;
  void (*fp)();
};

#define MAXTASKS 10
struct TASK tasks[MAXTASKS];

//设置定时器
void settimer(char *lc,char  line,char  tmrid,int d){
  *lc=line;
  tasks[tmrid].td=d;
}

//逻辑定时器处理，在定时器中断里调用
void dectimers() {
    unsigned char i;
    for (i=0;i<maxtasks;i++){
       if (tasks.td>0)  tasks.td--;
    }
}

//任务调度函数，在main里面运行
void runtasks() {
   unsigned char i;
   for(i=0;i<maxtasks;i++)
   {
     if (tasks.fp!=0){
           if (tasks.td==0){
             tasks.td=-1;
             tasks.fp();
                }
         }
        }
}

/****************小小调度器部分结束*******************************************************/


sbit KEY = P3^2;
unsigned char code numtab[16]={0x24,0x6F,0xE0,0x62,0x2B,0x32,0x30,0x67,0x20,0x22,0x21,0x38,0xB4,0x68,0xB0,0xB1};


sfr IAP_CONTR = 0xC7;
sfr WDT_CONTR = 0xC1;

//清除看门狗
void clr_wdt()
{
    WDT_CONTR =0x3C;
}

//初始化定时器
void InitT0()
{
        TMOD = 0x21;
        IE |= 0x82;  // 12t
        TL0=0Xff;
        TH0=0Xb7;
        /*
        TL0=0XF0; //100ms 重装     Detail: http://www.21ic.com/jichuzhishi/mcu/timer/2014-05-05/345616.html
        TH0=0XD8;
        */
        TR0 = 1;
}

void TIM6_Init(void)  //    CPU:72MHz
{
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
    TIM6->PSC = 72000 - 1; // 分频之后的时钟频率为1KHz
    TIM6->ARR = 2000 - 1; // 2s 定时频率
    TIM6->CR1 = 0x0004 + 0x0001; // URS = 1 CEN = 1
    TIM6->DIER = 0x0001; // 使能更新中断
}
void TIM6_IRQHandler(void)
{
    LED_Flash(GPIO_LED2);
    TIM6->SR=0x0000;//清标志位
}

//定时器中断
void INTT0(void) interrupt 1 using 1
{
        TL0=0Xff;    //10ms 重装
        TH0=0Xb7;
        /*
        TL0=0XF0; //100ms 重装     Detail: http://www.21ic.com/jichuzhishi/mcu/timer/2014-05-05/345616.html
        TH0=0XD8;
        */
        dectimers();
}

//任务一，状态机写法
void ontimer0(){

    //function

  //重装定时器
  if ( /*     */  ) tasks[0].td=45;  //运行450ms
  else tasks[0].td=5;  //运行50ms
}

//任务二，状态机写法
char keycount=0;
void task1(){
    if(KEY==0) {
       keycount++;
       if (keycount>20) //function;  //持续1s按键或者。。。。
    }
    else{
        keycount=0;
    }
    //重装定时器
    tasks[1].td=5;  // 每50ms检查
}


//任务三，伪线程写法
void  task2()
{
    static char i;
    _SS

    while(1){

        for(i=0;i<=9;i++){   //从0--9快速显示，间隔200mS
           WaitX(2,20);         //    等待200mS
           //function
        }
        for(i=0;i<=9;i++){ //从0--9慢速显示，间隔500mS
           WaitX(2,50);       //    等待500mS
           //function
        }
    }

    _EE
}



void main()
{
        unsigned char         KeyNum;
        P3M0 = 0x00;
        P3M1 =0x00;
        //WDT_CONTR= 0x00;   //关闭看门狗
        P1 = 0xff;         //关显示

        clr_wdt();

        InitT0();

        KEY =1;                                //按键IO口
        KeyNum=0;                        //按下次数

    //装载任务:
        tasks[0].fp=ontimer0;
        tasks[1].fp=task1;
        tasks[2].fp=task2;

    //循环调度
        while(1){
             runtasks();
             clr_wdt();
        }
}

/*

    Reminder:  Delay 2ms
        for(i=0;i<20;i++){
            delay_us(100);
            waitx(0);
            }

*/
