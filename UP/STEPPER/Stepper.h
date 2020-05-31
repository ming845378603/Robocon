#ifndef __STEPPER_H
#define __STEPPER_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//��ʱ�� ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/6/16
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

#define DIR PAout(8) //PD13��������ͨIO������������DIR2����
#define ENA PCout(8) //PD13��������ͨIO������������DIR2����
#define BallRackUP   1 //����
#define BallRackDOWN 0 //�½�
#define step_key_on  1 //��ʹ��
#define step_key_off 0 //��ʹ��

void BallRack_control(u8 dir,u8 step_key);
void Stepper_Init(void);
#endif

