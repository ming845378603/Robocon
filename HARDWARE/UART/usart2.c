#include "sys.h"
#include "usart2.h"
#include "My_Flag.h"
#include "stdio.h"
#include "math.h"
#include "robot.h"
#include "my_math.h"
#include "PID_forward.h"
#include "Route_Control.h"
#include "PID_Parameter.h"
#include "led.h"
#include "relay.h"
#include "rmds.h"
#include "delay.h"
#include "Stepper.h"
#include "vect.h"
#include "Robot.h"
#include "Basal_Move.h"
#include "Control_Arc.h"

extern int32_t Stop_flag;
extern s16 targetX, targetY1, targetA;
extern int32_t Vxx,Vyy;
u8 Start_flag=0;
extern int32_t spd[4];
int32_t Vx=0,Vy=0;
u8 up_flag = 0;
void USART2_IRQHandler(void)
{
    if (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) != RESET)  //接收到数据
    {
        u32 res = 0;
        res = USART_ReceiveData(USART2);
        if(res==0x07)                //电机急停
        {
            Stop_flag=1;
            USART_SendData(USART3,0x00);	//上层电机急停
        }
        if((res==0x05)&&(Start_flag==0))             //开始任务
        {
            Start_flag = 1;
        }
        if((res==0x05)&&(Start_flag==2))
        {
            Start_flag = 3;
        }
        if((res==0x05)&&(Start_flag==4))
        {
            Start_flag = 5;
        }
        if((res==0x05)&&(Start_flag==6))
        {
            Start_flag = 7;
        }
        if((res==0x05)&&(Start_flag==8))
        {
            Start_flag = 9;
        }
        if((res==0x05)&&(Start_flag==10))
        {
            Start_flag = 11;
        }
        if((res==0x05)&&(Start_flag==12))
        {
            Start_flag = 13;
        }
        if((res==0x05)&&(Start_flag==14))
        {
            Start_flag = 15;
        }
        /***********微调***********/
        if(res==0x03)
        {
            targetY1 = targetY1 + 10;
					  setup_PID(250,50,400,400,
                      250,50,400,400,
                      250,50,400,400);
        }
        if(res==0x06)
        {
            targetY1 = targetY1 - 10;
					  setup_PID(250,50,400,400,
                      250,50,400,400,
                      250,50,400,400);
        }
        if(res==0x04)
        {
            targetX = targetX - 10;
						setup_PID(250,50,400,400,
                      250,50,400,400,
                      250,50,400,400);
        }
        if(res==0x10)
        {
            targetX = targetX + 10;
					  setup_PID(250,50,400,400,
                      250,50,400,400,
                      250,50,400,400);
        }
        /************回原点**************/
        if(res==0x11)            //回原点
        {
            Start_flag = 20;
            Stop_flag=0;
        }							
			  if(res == 0x08)
				{
				 up_flag+=1;
				 if(up_flag>9)
				 up_flag=4;
				 USART_SendData(USART3,up_flag);
				}

    }

}



//初始化IO 串口2
//bound:波特率
void usart2_init(u32 bound)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;


    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //使能GPIOA时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);//使能USART2时钟

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; //PA2,PA3,复用功能,上拉输出
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure); //初始化PA2，PA3

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2); //GPIOA2复用为USART2
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2); //GPIOA3复用为USART2

    USART_InitStructure.USART_BaudRate = bound;//一般设置为9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式;	// 发模式
    USART_Init(USART2, &USART_InitStructure); //初始化串口
    USART_Cmd(USART2, ENABLE);  //使能串口

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启中断

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =0;//抢占优先级2
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器

}



// 发送一个字节
static void u2_putchar(u8 ch)
{
    while ((USART2->SR & 0X40) == 0);//循环发送,直到发送完毕
    USART2->DR = (u8)ch;
    //return ch;
}

void u2_putbuff(u8 *buff, u32 len)
{
    while (len--)
    {
        u2_putchar(*buff);
        buff++;
    }
}

