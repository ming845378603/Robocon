#include "PID_Control.h"
#include "PID_Parameter.h"
#include "GYRO_Lib.h"
#include "Robot.h"
#include <stdio.h>
#include "Basal_Move.h"
#include "My_Math.h"
#include <stdint.h>
#include "uart4.h"
#include "usart.h"	
#include "beep.h"
#include "Route_Control.h"
#include "Global.h"

#define USE_ANGLE


// 设定目标值
void PID_Set_Angle_Value(int32_t t_angle, int32_t Route_type)
{
	G_PID_Parameter[Route_type].PID_Angle.Calculate.set_value = t_angle;
}
void PID_Set_LineY_Value(int32_t t_y, int32_t Route_type)
{
    G_PID_Parameter[Route_type].PID_LineY.Calculate.set_value = t_y;
}
void PID_Set_LineX_Value(int32_t t_x, int32_t Route_type)
{
    G_PID_Parameter[Route_type].PID_LineX.Calculate.set_value = t_x;
}



s16 targetX=0, targetY1=0, targetA=0;

//直线环Y测试
void PID_LineY_Loop_Test(void)
{
    int32_t pid_Line_out = 0;	
	
		PID_Set_LineY_Value(targetY1, ROUTE_LINE);
		pid_Line_out = PID_Calculate(&G_PID_Parameter[ROUTE_LINE].PID_LineY, nloc.Coords.y);
		linear_speed(0, 1, nloc.Angle, pid_Line_out);
}

//直线环X测试
void PID_LineX_Loop_Test(void)
{
    int32_t pid_Line_out = 0;

		PID_Set_LineX_Value(targetX, ROUTE_LINE);
		pid_Line_out = PID_Calculate(&G_PID_Parameter[ROUTE_LINE].PID_LineX, nloc.Coords.x);
    linear_speed(1,0,nloc.Angle,pid_Line_out);
}

// 角度环测试
void PID_Angle_Loop_Test(void)
{
    int32_t pid_angle_out = 0;
	
	  PID_Set_Angle_Value(targetA,ROUTE_LINE);
		pid_angle_out =  PID_Calculate(&G_PID_Parameter[ROUTE_LINE].PID_Angle, (int32_t)(nloc.Angle));
    rotate_speed(pid_angle_out);
}


/*****************************************************************************\
* Function Name   : PID_Calculate
* Input           : double pid_angule_speed          [角速度]
*                   double pid_angle                 [角度]
*                   double pid_angle_acceleration    [角加速度]
* Output          : None
* Return          : double pid_angle_out             [pid角度环输出]
* Description     : PID角度环，根据当前的角度参数，计算下一个时间的角速度设定值。
*                   由于角速度的积分为角度、微分为角加速度
*                   所以角度环的i参数取当前角度值，d参数取角加速度值
\*****************************************************************************/
int32_t PID_Calculate(sPID_Loop_TypeDef * pid_loop, int32_t real_value)
{
    int32_t up = 0; /*比例运算输出*/
    int32_t ui = 0; /*积分运算输出*/
    int32_t ud = 0; /*微分运算输出*/
    int32_t Iteg_F; /*连续变速积分系数*/

    pid_loop->Calculate.real_value[NOW] = real_value;                       /*系统当前的实际值*/
    pid_loop->Calculate.err[NOW] = pid_loop->Calculate.set_value - pid_loop->Calculate.real_value[NOW];   /*获取误差*/
    /*死区控制*/
    if (my_abs(pid_loop->Calculate.err[NOW]) < pid_loop->Parameter.Dead_Zone)
        pid_loop->Calculate.err[NOW] = 0;

    /*连续变速积分*/
    else if (my_abs(pid_loop->Calculate.err[NOW]) < pid_loop->Parameter.Iteg_B)
        Iteg_F = 1;
    else if (my_abs(pid_loop->Calculate.err[NOW]) < (pid_loop->Parameter.Iteg_A + pid_loop->Parameter.Iteg_B))
        Iteg_F = (pid_loop->Parameter.Iteg_A - my_abs(pid_loop->Calculate.err[NOW]) + pid_loop->Parameter.Iteg_B) / pid_loop->Parameter.Iteg_A;
    else
        Iteg_F = 0;
    pid_loop->Calculate.err_iteg += (pid_loop->Calculate.err[NOW] * Iteg_F);        /*误差积分*/

    /*积分限幅*/
    if (pid_loop->Calculate.err_iteg > pid_loop->Parameter.Iteg_Max)       pid_loop->Calculate.err_iteg = pid_loop->Parameter.Iteg_Max;
    else if (pid_loop->Calculate.err_iteg < -pid_loop->Parameter.Iteg_Max) pid_loop->Calculate.err_iteg = -pid_loop->Parameter.Iteg_Max;

    /*PID运算*/
    up = pid_loop->Calculate.err[NOW] * pid_loop->Parameter.P;
    ui = pid_loop->Calculate.err_iteg * pid_loop->Parameter.I;
    ud = (pid_loop->Calculate.real_value[LAST] - pid_loop->Calculate.real_value[NOW]) * pid_loop->Parameter.D; /*微分先行PID*/

    pid_loop->Calculate.pid_out = up / 100 + ui / 100 + ud/100;
		
    /*对PID输出进行限幅*/
    if (pid_loop->Calculate.pid_out > pid_loop->Parameter.Out_Max)      pid_loop->Calculate.pid_out = pid_loop->Parameter.Out_Max;
    else if (pid_loop->Calculate.pid_out < pid_loop->Parameter.Out_Min) pid_loop->Calculate.pid_out = pid_loop->Parameter.Out_Min;

    pid_loop->Calculate.real_value[LAST] = pid_loop->Calculate.real_value[NOW];            /*系统实际迭代*/
    pid_loop->Calculate.err[LAST] = pid_loop->Calculate.err[NOW];                          /*系统偏差迭代*/

    return pid_loop->Calculate.pid_out;
}


/******************* (C) COPYRIGHT 2016 DGVY ***********END OF FILE***********/
