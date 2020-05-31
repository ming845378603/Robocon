#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "beep.h"
#include "key.h"
#include "My_BSP.h"
#include "Basal_Move.h"
#include "GYRO_Lib.h"
#include "My_Flag.h"
#include "PID_Control.h"
#include "Robot.h"
#include "ANO_DT.h"
#include "Time.h"
#include "Route_Control.h"
#include "uart4.h"
#include "PID_forward.h"
#include "usart2.h"
#include "relay.h"
#include "rmds.h"
#include "delay.h"
#include "switch.h"
#include "stepper.h"
#include "Control_Arc.h"
#include "task.h"

int main(void)
{
    /*  硬件资源初始化，包括：LED灯、蜂鸣器、串口、滴答定时器、TIM6、编码器、电机驱动EPOS、CAN通信、陀螺仪等*/
    BSP_Init(115200, 168);	
    Argument_Load();		//将上位机的数据存入flash	 
    while (1)
    {
			main_loop();			
    }
}

