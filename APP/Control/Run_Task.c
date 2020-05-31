#include "Global.h"
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
#include "Task_Loop.h"
#include "ANO_DT.h"
#include "Time.h"
#include "PID_Control.h"
#include "PID_forward.h"
#include "PID_Parameter.h"
#include "Route_Control.h"
#include <stdio.h>
#include "Robot.h"
#include "can.h"
#include "uart4.h"
#include "Run_task.h"
#include "uart4.h"

extern forward_ctl_t RouteToSet;

#include "GYRO_Lib.h"
// 锁定点任务
extern int32_t lock_type, Fuck_Go_flag;
extern Flag Protect_Angle;
extern Flag Protect_X;
extern Flag Protect_Y;
extern int32_t Junction_x[][9];
extern int32_t Junction_y[][9];
int32_t Task_Go_Flag;
extern int32_t start_pos_X, start_pos_Y;
u8 can_buf[8];


void GO_GetBallPoint_Task(void)
{
	switch (Task_Go_Flag)
	{
	case 0:
		if (nloc.Coords.y > 430)
		{
			CameraData_UseFlag = CameraData_NotUse;
		}
		if (460 -nloc.Coords.y > 35)
		{
		PID_Angle_Loop_Test();    //PID角度调节
		control_Yline();           //前馈式y直线控制
		PID_LineX_Loop_Test();    //PID直线X控制
		}
		else
		{
			PID_Set_LineY_Value(460, ROUTE_TRACE_Y);
			PID_Set_LineY_Value(460, ROUTE_LINE);
			PID_Lock_LineYPoint();
			if ((my_abs(nloc.Coords.y - 460) < 1))
			{
				Y_Offset_CameraTarget = 460;
				CameraData_UseFlag = CameraData_Use;
				USART_SendData(UART5, 'X');
				Task_Go_Flag = 1;
				Get_Cycle_T(0);
				USART_SendData(UART5, '1');
				setup_Xline(4500);
			}
		}
		break;
	case 1:

		if (my_abs(nloc.Coords.x - 1000) < 10)
		{
			Y_Offset_CameraTarget = 475;
		}
		if (my_abs(nloc.Coords.x - 2000) < 10)
		{
			Y_Offset_CameraTarget = 485;
		}
		if (my_abs(nloc.Coords.x - 3000) < 10)
		{
			Y_Offset_CameraTarget = 495;
		}
		if (my_abs(nloc.Coords.x - 4000) < 10)
		{
			Y_Offset_CameraTarget = 520;
		}

		if (nloc.Coords.x > 4420)
		{
			CameraData_UseFlag = CameraData_NotUse;
		}
		if ( 4500 - nloc.Coords.x  > 40)//这里要留这么点距离用pid来调，这个线路过渡这里，不能实现0秒切换，车有可能飘着走
		{
			PID_Angle_Loop_Test();    //PID角度调节
			control_Xline();           //前馈式X直线控制
			PID_LineY_Loop_Test();
		}
		else
		{
			PID_Set_LineX_Value(4500, ROUTE_TRACE_X);
			PID_Set_LineX_Value(4500, ROUTE_LINE);
			PID_Lock_LineXPoint();
			if (my_abs(nloc.Coords.x - 4500) < 1)
			{
				CameraData_UseFlag = CameraData_Use;
				Y_OffsetCameraCross = -nloc.Coords.y + 460;
				USART_SendData(UART5, 'Y');
				Task_Go_Flag = 2;
				Get_Cycle_T(0);
				X_Offset_CameraTarget = 4520;
				USART_SendData(UART5, '3');
				setup_Yline(3480);
			}
		}
		break;
	case 2:
		if (3480 - nloc.Coords.y  > 120)
	   {
		PID_Angle_Loop_Test();    //PID角度调节
		control_Yline();           //前馈式X直线控制
		PID_LineX_Loop_Test();    //PID直线X控制
		}
		else 
		{
			PID_Set_LineY_Value(3480, ROUTE_TRACE_Y);
			PID_Set_LineY_Value(3480, ROUTE_LINE);
			PID_Lock_LineYPoint();
			if ((my_abs(nloc.Coords.y - 3480) < 1))
			{
				Task_Go_Flag = 3;
				USART_SendData(UART5, 'Y');
			}
		}
		break;
	case 3:
		if (nloc.Coords.y  < 2800)
		{
			USART_SendData(UART5, '8');
		}
		PID_Set_LineY_Value(3480, ROUTE_TRACE_Y); 
		PID_Set_LineY_Value(3480, ROUTE_LINE);
		PID_Lock_LineYPoint();
		if (Fuck_Go_flag == 1)
		{
			Task_Go_Flag = 4;
		}
		CAN1_SendMsg(can_buf, 0xA1);
		break;
	case 4:
		PID_BackTo_StartPoint();
		break;
	default:
		break;
	}
	SetMotorSpeed();
}

