#ifndef __STEPPER_H
#define __STEPPER_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//定时器 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/6/16
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

#define DIR PAout(8) //PD13引脚作普通IO，控制驱动的DIR2引脚
#define ENA PCout(8) //PD13引脚作普通IO，控制驱动的DIR2引脚
#define BallRackUP   1 //上升
#define BallRackDOWN 0 //下降
#define step_key_on  1 //开使能
#define step_key_off 0 //关使能

void BallRack_control(u8 dir,u8 step_key);
void Stepper_Init(void);
#endif

