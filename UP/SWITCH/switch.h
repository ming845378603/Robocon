#ifndef __SWITCH_H
#define __SWITCH_H
#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//LED��������
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/2
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved
//////////////////////////////////////////////////////////////////////////////////


//LED�˿ڶ���
#define SW1  GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_1)//��ȡ����0
#define SW2  GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_2)//��ȡ����1
#define SW3  GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3)//��ȡ����3(WK_UP) 
#define SW4  GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_4)//��ȡ����0
#define SW5  GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_5)//��ȡ����1
#define SW6  GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_6)//��ȡ����3(WK_UP) 


void SWITCH_Init(void);//��ʼ��
void SW_MOTOR(void);
void SW_Under(void);  //��λ���ذ��µ�ɨ�躯��
void Position_scan(void);
#endif
