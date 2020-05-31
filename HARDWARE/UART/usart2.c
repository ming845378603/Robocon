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
    if (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) != RESET)  //���յ�����
    {
        u32 res = 0;
        res = USART_ReceiveData(USART2);
        if(res==0x07)                //�����ͣ
        {
            Stop_flag=1;
            USART_SendData(USART3,0x00);	//�ϲ�����ͣ
        }
        if((res==0x05)&&(Start_flag==0))             //��ʼ����
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
        /***********΢��***********/
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
        /************��ԭ��**************/
        if(res==0x11)            //��ԭ��
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



//��ʼ��IO ����2
//bound:������
void usart2_init(u32 bound)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;


    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //ʹ��GPIOAʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);//ʹ��USART2ʱ��

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; //PA2,PA3,���ù���,�������
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
    GPIO_Init(GPIOA, &GPIO_InitStructure); //��ʼ��PA2��PA3

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2); //GPIOA2����ΪUSART2
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2); //GPIOA3����ΪUSART2

    USART_InitStructure.USART_BaudRate = bound;//һ������Ϊ9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ;	// ��ģʽ
    USART_Init(USART2, &USART_InitStructure); //��ʼ������
    USART_Cmd(USART2, ENABLE);  //ʹ�ܴ���

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//�����ж�

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =0;//��ռ���ȼ�2
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//�����ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���

}



// ����һ���ֽ�
static void u2_putchar(u8 ch)
{
    while ((USART2->SR & 0X40) == 0);//ѭ������,ֱ���������
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

