#ifndef __BEEP_H
#define __BEEP_H	 
#include "sys.h" 
 
#define BUZZER_ON   PAout(4) = 1
#define BUZZER_OFF  PAout(4) = 0	// ����������IO 

void BEEP_Init(void);//��ʼ��		 				    
#endif

















