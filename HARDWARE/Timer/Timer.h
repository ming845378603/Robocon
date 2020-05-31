#ifndef _TIMER_H
#define _TIMER_H
#include "sys.h"

/******Timer_2*****************************************************************/
//#define TIMER_2_INTERRUPT_FUNCTION_ENABLE    //使能本文件内的Timer_2中断函数

// 通用定时器Tiemr2初始化 无中断
void Timer2_Timekeeping_Init(u16 arr, u16 psc);


/******Timer_3*****************************************************************/	
 //通用定时器Tiemr3初始化 带中断
void Timer3_Interrupt_Init(u16 arr, u16 psc);


/******Timer_6*****************************************************************/
#define TIMER_6_INTERRUPT_FUNCTION_ENABLE    //使能本文件内的Timer_6中断函数

// 通用定时器Tiemr6初始化 带中断
void Timer6_Interrupt_Init(u16 arr, u16 psc);

// 开启Timer6定时器
void Timer6_ON(void);

// 关闭Timer6定时器
void Timer6_OFF(void);


#endif
