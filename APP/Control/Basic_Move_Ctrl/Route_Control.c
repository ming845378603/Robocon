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
#define GetCurSpd() (G_Robot_Master.cur_speed)             /*��ȡ��ǰ�ٶȵ�ָ��*/
#define GetCurLocationPointer() (&G_Robot_Master.Now_Position) /*��ȡ��ǰλ��ָ��*/

/*ֱ�߿������*/
forward_ctl_t RouteToSet;
/*ǰ������*/
extern int32_t forwardParam[7];
extern int32_t L_Speed[4];
extern int32_t H_Speed[4];
/*��¼��ʼλ��*/
int32_t start_pos_X, start_pos_Y;

/*�趨Yֱ��ʽǰ������*/
void setup_Xline(int32_t Xtarget)
{
    /*��¼��ʼλ��*/
    start_pos_X = nloc.Coords.x;

    memset(&RouteToSet, 0, sizeof(forward_ctl_t));
	
    /*����·�̣������ɸ���*/
    RouteToSet.vect_v = Xtarget- nloc.Coords.x;
     
	  /*��ǰ������Yֱ���ϵķ���*/
	  int32_t cur_spd; 
    cur_spd = GetCurSpd() * my_cos(nloc.Angle);
	  
	  /*���ó��ٶ�*/
    RouteToSet.start_v = my_abs(cur_spd);
    /*������·�̣���ֵ��*/
    RouteToSet.total_len = my_abs(RouteToSet.vect_v);
   
		/*�����ٶȡ�ĩ�ٶȡ��𲽼��ٶȡ�ɲ�����ٶ�*/
    update_g_speed_param(&RouteToSet, X_Speed);

    /*�趨ǰ��������������Kf Kp_a Kp_d Kd_a Kd_d Max Min��*/
    update_forward_param(&RouteToSet, X_forwardParam);

    /*��ʼ��ǰ������*/
    update_forward_ctl(&RouteToSet, GetSysTime_us() / 1000.0f);
		
#if defined(DEBUG_PRINTF)
    printf("Start_pos:  %d\tXtarget:  %d\r\n", start_pos_X, Xtarget);	
    printf("Kf: %d\tKp_a: %d\tKp_d: %d\r\n", RouteToSet.kf, RouteToSet.kp_aclt, RouteToSet.kp_decr);
    printf("Kd_a: %d\tKd_d: %d\tMax: %d\tMin: %d\r\n", RouteToSet.kd_aclt, RouteToSet.kd_decr, RouteToSet.max_out, RouteToSet.min_out);
#endif
}


/*�趨Yֱ��ʽǰ������*/
void setup_Yline(int32_t Ytarget)
{
    /*��¼��ʼλ��*/
    start_pos_Y = nloc.Coords.y;

    memset(&RouteToSet, 0, sizeof(forward_ctl_t));
    /*����·�̣������ɸ���*/
    RouteToSet.vect_v = Ytarget- nloc.Coords.y;
	
		/*��ǰ������Yֱ���ϵķ���*/	
    int32_t cur_spd;    
    cur_spd = GetCurSpd() * my_cos(nloc.Angle);
		
	  /*���ó��ٶ�*/
    RouteToSet.start_v = my_abs(cur_spd);
	
    /*������·�̣���ֵ��*/
    RouteToSet.total_len = my_abs(RouteToSet.vect_v);

		 /*�趨�����ٶȡ�ĩ�ٶȡ��𲽼��ٶȡ�ɲ�����ٶ�*/
    update_g_speed_param(&RouteToSet, Y_Speed);
	
    /*�趨ǰ��������������Kf Kp_a Kp_d Kd_a Kd_d Max Min��*/
    update_forward_param(&RouteToSet, Y_forwardParam1);

    /*��ʼ��ǰ������*/
    update_forward_ctl(&RouteToSet,GetSysTime_us() / 1000.0f);
		
#if defined(DEBUG_PRINTF)
    printf("& Forward &  Start_pos:  %d\tEnd_pos:  %d\r\n", start_pos_Y,Ytarget);
    printf("& Kf: %d\tKp_a: %d\tKp_d: %d\r\n", RouteToSet.kf, RouteToSet.kp_aclt, RouteToSet.kp_decr);
    printf("& Kd_a: %d\tKd_d: %d\tMax: %d\tMin: %d\r\n", RouteToSet.kd_aclt, RouteToSet.kd_decr, RouteToSet.max_out,RouteToSet.min_out);
#endif
}


//ֱ�߿���
void control_Xline(void)
{
    int32_t          real_pos;							    /*��ǰ��ʵ��λ��*/
    int32_t          ctl_v;								    /*ǰ���ٶȿ�����*/

    real_pos = my_abs(nloc.Coords.x - start_pos_X);

    ctl_v = forward_ctl(&RouteToSet, real_pos, GetSysTime_us() / 1000.0f);

    if (ctl_v > RouteToSet.max_out) ctl_v = RouteToSet.max_out;
    else if (ctl_v < RouteToSet.min_out) ctl_v = RouteToSet.min_out;

    /*������ֱ�ߺ������������ٶ�*/
    linear_speed(RouteToSet.vect_v, 0, nloc.Angle, ctl_v);
}
//ֱ�߿���
void control_Yline(void)
{
    int32_t          real_pos;							    /*��ǰ��ʵ��λ��*/
    int32_t          ctl_v;								    /*ǰ���ٶȿ�����*/

    real_pos = my_abs(nloc.Coords.y - start_pos_Y);

    ctl_v = forward_ctl(&RouteToSet, real_pos,  GetSysTime_us() / 1000.0f);

    if (ctl_v > RouteToSet.max_out) ctl_v = RouteToSet.max_out;
    else if (ctl_v < RouteToSet.min_out) ctl_v = RouteToSet.min_out;
	
    /*������ֱ�ߺ������������ٶ�*/
    linear_speed(0,RouteToSet.vect_v, nloc.Angle,ctl_v);
}

	
	
	
	

	









