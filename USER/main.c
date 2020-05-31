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
    /*  Ӳ����Դ��ʼ����������LED�ơ������������ڡ��δ�ʱ����TIM6�����������������EPOS��CANͨ�š������ǵ�*/
    BSP_Init(115200, 168);	
    Argument_Load();		//����λ�������ݴ���flash	 
    while (1)
    {
			main_loop();			
    }
}

