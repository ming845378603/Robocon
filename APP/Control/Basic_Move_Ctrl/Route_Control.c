#include "Route_Control.h"
#include "Basal_Move.h"
#include "Vect.h"
#include "PID_forward.h"
#include "My_Math.h"
#include "PID_Parameter.h"
#include "PID_Control.h"
#include "GYRO_Lib.h"
#include "Robot.h"
#include <stdint.h>
#include "Time.h"
#include "usart.h"
#define GetCurSpd() (G_Robot_Master.cur_speed)             /*获取当前速度的指针*/
#define GetCurLocationPointer() (&G_Robot_Master.Now_Position) /*获取当前位置指针*/

/*直线控制相关*/
forward_ctl_t RouteToSet;
/*前馈参数*/
extern int32_t forwardParam[7];
extern int32_t L_Speed[4];
extern int32_t H_Speed[4];
/*记录起始位置*/
int32_t start_pos_X, start_pos_Y;

/*设定Y直线式前馈参数*/
void setup_Xline(int32_t Xtarget)
{
    /*记录起始位置*/
    start_pos_X = nloc.Coords.x;

    memset(&RouteToSet, 0, sizeof(forward_ctl_t));
	
    /*设置路程（可正可负）*/
    RouteToSet.vect_v = Xtarget- nloc.Coords.x;
     
	  /*当前车速在Y直线上的分量*/
	  int32_t cur_spd; 
    cur_spd = GetCurSpd() * my_cos(nloc.Angle);
	  
	  /*设置初速度*/
    RouteToSet.start_v = my_abs(cur_spd);
    /*设置总路程（正值）*/
    RouteToSet.total_len = my_abs(RouteToSet.vect_v);
   
		/*匀速速度、末速度、起步加速度、刹车减速度*/
    update_g_speed_param(&RouteToSet, X_Speed);

    /*设定前馈参数，包括（Kf Kp_a Kp_d Kd_a Kd_d Max Min）*/
    update_forward_param(&RouteToSet, X_forwardParam);

    /*初始化前馈控制*/
    update_forward_ctl(&RouteToSet, GetSysTime_us() / 1000.0f);
		
#if defined(DEBUG_PRINTF)
    printf("Start_pos:  %d\tXtarget:  %d\r\n", start_pos_X, Xtarget);	
    printf("Kf: %d\tKp_a: %d\tKp_d: %d\r\n", RouteToSet.kf, RouteToSet.kp_aclt, RouteToSet.kp_decr);
    printf("Kd_a: %d\tKd_d: %d\tMax: %d\tMin: %d\r\n", RouteToSet.kd_aclt, RouteToSet.kd_decr, RouteToSet.max_out, RouteToSet.min_out);
#endif
}


/*设定Y直线式前馈参数*/
void setup_Yline(int32_t Ytarget)
{
    /*记录起始位置*/
    start_pos_Y = nloc.Coords.y;

    memset(&RouteToSet, 0, sizeof(forward_ctl_t));
    /*设置路程（可正可负）*/
    RouteToSet.vect_v = Ytarget- nloc.Coords.y;
	
		/*当前车速在Y直线上的分量*/	
    int32_t cur_spd;    
    cur_spd = GetCurSpd() * my_cos(nloc.Angle);
		
	  /*设置初速度*/
    RouteToSet.start_v = my_abs(cur_spd);
	
    /*设置总路程（正值）*/
    RouteToSet.total_len = my_abs(RouteToSet.vect_v);

		 /*设定匀速速度、末速度、起步加速度、刹车减速度*/
    update_g_speed_param(&RouteToSet, Y_Speed);
	
    /*设定前馈参数，包括（Kf Kp_a Kp_d Kd_a Kd_d Max Min）*/
    update_forward_param(&RouteToSet, Y_forwardParam1);

    /*初始化前馈控制*/
    update_forward_ctl(&RouteToSet,GetSysTime_us() / 1000.0f);
		
#if defined(DEBUG_PRINTF)
    printf("& Forward &  Start_pos:  %d\tEnd_pos:  %d\r\n", start_pos_Y,Ytarget);
    printf("& Kf: %d\tKp_a: %d\tKp_d: %d\r\n", RouteToSet.kf, RouteToSet.kp_aclt, RouteToSet.kp_decr);
    printf("& Kd_a: %d\tKd_d: %d\tMax: %d\tMin: %d\r\n", RouteToSet.kd_aclt, RouteToSet.kd_decr, RouteToSet.max_out,RouteToSet.min_out);
#endif
}


//直线控制
void control_Xline(void)
{
    int32_t          real_pos;							    /*当前的实际位置*/
    int32_t          ctl_v;								    /*前馈速度控制量*/

    real_pos = my_abs(nloc.Coords.x - start_pos_X);

    ctl_v = forward_ctl(&RouteToSet, real_pos, GetSysTime_us() / 1000.0f);

    if (ctl_v > RouteToSet.max_out) ctl_v = RouteToSet.max_out;
    else if (ctl_v < RouteToSet.min_out) ctl_v = RouteToSet.min_out;

    /*调用走直线函数计算轮子速度*/
    linear_speed(RouteToSet.vect_v, 0, nloc.Angle, ctl_v);
}
//直线控制
void control_Yline(void)
{
    int32_t          real_pos;							    /*当前的实际位置*/
    int32_t          ctl_v;								    /*前馈速度控制量*/

    real_pos = my_abs(nloc.Coords.y - start_pos_Y);

    ctl_v = forward_ctl(&RouteToSet, real_pos,  GetSysTime_us() / 1000.0f);

    if (ctl_v > RouteToSet.max_out) ctl_v = RouteToSet.max_out;
    else if (ctl_v < RouteToSet.min_out) ctl_v = RouteToSet.min_out;
	
    /*调用走直线函数计算轮子速度*/
    linear_speed(0,RouteToSet.vect_v, nloc.Angle,ctl_v);
}

	
	
	
	

	










