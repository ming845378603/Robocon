#ifndef __MOTOR_H
#define __MOTOR_H	 
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK��ӢSTM32������
//LED��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/2
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 
#define MOTOR1 PDout(12)// P0
#define MOTOR2 PDout(11)// P1	
#define RELAY3 PDout(10)// P2
#define RELAY4 PDout(9)// PE5	2345
#define RELAY5 PDout(8)// PB5
#define RELAY6 PBout(15)// PE5	2345

void RELAY_Init(void);//��ʼ��

		 				    
#endif
