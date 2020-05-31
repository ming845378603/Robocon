#ifndef __MOTOR_H
#define __MOTOR_H	 
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK精英STM32开发板
//LED驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/2
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 
#define MOTOR1 PDout(12)// P0
#define MOTOR2 PDout(11)// P1	
#define RELAY3 PDout(10)// P2
#define RELAY4 PDout(9)// PE5	2345
#define RELAY5 PDout(8)// PB5
#define RELAY6 PBout(15)// PE5	2345

void RELAY_Init(void);//初始化

		 				    
#endif
