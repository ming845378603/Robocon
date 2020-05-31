#ifndef __SWITCH_H
#define __SWITCH_H
#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//LED驱动代码
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/2
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved
//////////////////////////////////////////////////////////////////////////////////


//LED端口定义
#define SW1  GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_1)//读取按键0
#define SW2  GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_2)//读取按键1
#define SW3  GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3)//读取按键3(WK_UP) 
#define SW4  GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_4)//读取按键0
#define SW5  GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_5)//读取按键1
#define SW6  GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_6)//读取按键3(WK_UP) 


void SWITCH_Init(void);//初始化
void SW_MOTOR(void);
void SW_Under(void);  //限位开关按下的扫描函数
void Position_scan(void);
#endif
