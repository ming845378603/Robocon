#ifndef __LED_H
#define __LED_H
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


#define LED1 PDout(8)	// DS0
#define LED2 PBout(14)	// DS1	 
#define LED3 PBout(13)	// DS0
#define LED4 PDout(9)	// DS1
#define LED5 PDout(10)	// DS0
#define LED6 PFout(11)	// DS1

void LED_Init(void);//��ʼ��		 				    
#endif
