#include "My_BSP.h"
#include "usart.h"
#include "delay.h"
#include "sys.h"
#include "can.h"
#include "beep.h"
#include "led.h"
#include "encoder.h"
#include "Time.h"
#include "usart3.h"
#include "GYRO_Lib.h"
#include "uart4.h"
#include <stdint.h>
#include "Time.h"
#include "Timer.h"
#include "usart2.h"
#include "epos_N.h"
#include "relay.h"
#include "rmds.h"
#include "switch.h"
#include "Stepper.h"
#include "elmo_drive.h"
/*****************************************************************************\
* Function Name   : Init
* Input           : uint32_t uart_bound     [初始化UART波特率]
*                   uint8_t delay_sysclk    [系统时钟频率]
* Output          : None
* Return          : None
* Description     : 板级初始化，初始化硬件
*                   默认uart波特率为115200
*                   默认系统时钟为168MHz
\*****************************************************************************/
void BSP_Init(uint32_t uart_bound,uint8_t delay_sysclk)
{
    
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置系统中断优先级分组2

    System_Tiem_Clear();		//清空系统（滴答定时器）时间
    SysTick_Init();			//初始化滴答定时器 
	
    Cycle_Time_Init();		//前馈确定周期开始计时
	
    uart1_init(uart_bound); 			//用printf打印数据
    usart2_init(uart_bound);	//用作遥控器
		usart3_init(uart_bound);	//用作给上层发命令
	
    LED_Init();
	
    BEEP_Init();
    BUZZER_ON;
	
		Encoder_Init_TIM5();
		Encoder_Init_TIM2();

    Timer3_Interrupt_Init(100, 840-1);  //分频系数840，所以84M/840=100Khz的计数频率,1ms进一次中断
		
		CAN_Config();						//CAN初始化
		delay_ms(500);
    GYRO_Init();										//陀螺仪初始化
		printf(" * GYRO_Ready!\r\n");		//陀螺仪初始化成功
		
		ELMO_Init();
		delay_ms(2000);	
    BUZZER_OFF;			//所有硬件设备初始化成功

}


//软件复位函数
void SoftReset(void)
{
    NVIC_SystemReset();// 软件复位 
}


