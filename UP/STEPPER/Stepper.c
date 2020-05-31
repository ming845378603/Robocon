#include "Stepper.h"
#include "usart.h"
 #include"stm32f4xx.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//��ʱ��PWM ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/4
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 


//��ͨIO��ʼ��
//��PD13��ʼ��Ϊ��ͨIO�����Ʋ��������ת������
void io_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��GPIODʱ��
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��
	GPIO_ResetBits(GPIOA,GPIO_Pin_8);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//ʹ��GPIODʱ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��
	GPIO_ResetBits(GPIOC,GPIO_Pin_8);	 
}


//TIM4 PWM���ֳ�ʼ�� 
//PWM�����ʼ��
//arr���Զ���װֵ
//psc��ʱ��Ԥ��Ƶ��
void Stepper1_PWM_Init(u32 arr, u32 psc)
{
	//�˲������ֶ��޸�IO������
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);  	//TIM4ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); 	//ʹ��PORTFʱ��	

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4); //GPIOF9����Ϊ��ʱ��14

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;           //GPIOF9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
	GPIO_Init(GPIOD, &GPIO_InitStructure);              //��ʼ��PF9

	TIM_TimeBaseStructure.TIM_Prescaler = psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period = arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;

	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);//��ʼ����ʱ��4

	//��ʼ��TIM44 Channel3 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ե�
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM4OC3
	TIM_OC4PreloadConfig(TIM4,TIM_OCPreload_Enable);  //ʹ��TIM14��CCR3�ϵ�Ԥװ�ؼĴ���

	TIM_ARRPreloadConfig(TIM4, ENABLE);//ARPEʹ�� 
	TIM_Cmd(TIM4, ENABLE);  //ʹ��TIM4

}


void Stepper_Init(void)//������ʼ��
{
	io_Init();	
	Stepper1_PWM_Init(1500-1,84-1);	//84M/84=1Mhz�ļ���Ƶ��,��װ��ֵ500������PWMƵ��Ϊ 1M/500=2Khz.
	TIM_SetCompare4(TIM4,999);
}


/***********************************
*��������:BallRack_control
*�������:dir��1Ϊ�½���0Ϊ������
*��������:ͨ�����Ʋ������������ܵ�������dir���Ʒ���u8,step_key����ʹ��
***********************************/
void BallRack_control(u8 dir,u8 step_key)
{
		if(step_key == 0)
		{
		    ENA = 0;
		}
		if(step_key == 1)
		{
		    ENA = 1;		
		}
	  if(dir == 0)
		{
		    DIR = 0;
		}
		if(dir == 1)
		{
		    DIR = 1;
		}
}
