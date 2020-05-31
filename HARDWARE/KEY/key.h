#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h" 
 

/*下面的方式是通过直接操作库函数方式读取IO*/
#define KEY0 		GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13) //PE4
#define KEY1 		GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_14)	//PE3 
#define KEY2 		GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_15) //PE2

void KEY_Init(void);	//IO初始化
u8 KEY_Scan(u8);  		//按键扫描函数	

#endif
