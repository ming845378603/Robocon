#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h" 
 

/*����ķ�ʽ��ͨ��ֱ�Ӳ����⺯����ʽ��ȡIO*/
#define KEY0 		GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13) //PE4
#define KEY1 		GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_14)	//PE3 
#define KEY2 		GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_15) //PE2

void KEY_Init(void);	//IO��ʼ��
u8 KEY_Scan(u8);  		//����ɨ�躯��	

#endif
