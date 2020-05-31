#ifndef _PID_FORWARD_H_
#define _PID_FORWARD_H_
#include "stm32f4xx.h"
#include "My_Math.h"
#include "Time.h"
#ifdef __cplusplus
extern "C" {
#endif

/********************************直线控制参数设定*****************************/
typedef struct
{
	int32_t kf;				/*前馈系数*/
	int32_t kp_aclt;		/*反馈系数(非减速阶段)*/
	int32_t kp_decr;		/*反馈系数(减速阶段)*/
	int32_t kd_aclt;		/*微分系数(加速阶段)*/
	int32_t kd_decr;		/*微分系数(减速阶段)*/
	int32_t min_out;		/*最小控制量输出*/
	int32_t max_out;		/*最大控制量输出*/
	int32_t vect_v;         /*输出速度方向*/

	float    cmd_pos[3];	/*指令位置(mm)*/
	float    real_pos;		/*真实位置(mm)*/
	float  	 total_len;		/*总路程(mm)*/

	float start_time;    /*本路段的开始时间(ms)*/
	float end_time;		/*本路段的结束时间(ms)*/
	float aclt_time;		/*加速结束时间点(ms)*/
	float unif_time;		/*匀速结束时间点，uniform_time(ms)*/
	float decr_time;		/*减速过程的时间长度(ms)*/

	float    start_v;		/*初速度(mm/s)*/
	float    unif_v;		/*匀速速度(mm/s)*/
	float    end_v;			/*末速度(mm/s)*/
	float    aclt;			/*加速度(mm/s/s)*/
	float    decr;			/*减速度(mm/s/s)*/

}
forward_ctl_t;


/*********************************弧度控制参数设定*****************************/
typedef struct 
{
    int32_t x;
    int32_t y;
	  float   ang;
}
point_t;

/*向量*/
typedef struct
{
    point_t s;
    point_t e;
}
vect_t;


//#define FWD_CTL_PERIOD	(Get_Cycle_T(0)) /*前馈控制周期10ms*/
//#define FWD_CTL_PERIOD	(CONTROL_PERIOD_MS) /*前馈控制周期10ms*/
/*控制周期ms*/
#define CONTROL_PERIOD_MS (20)
#define CONTROL_PERIOD_S  ((double)CONTROL_PERIOD_MS / 1000.0)

void update_forward_param(forward_ctl_t * p, int32_t * param);
void update_g_speed_param(forward_ctl_t * p, int32_t * param);

void update_forward_ctl(forward_ctl_t * p, float start_time);
int32_t forward_ctl(forward_ctl_t *p, int32_t current_pos, float current_time);
extern int32_t X_Speed[4];
extern int32_t Y_Speed[4];      /*设置速度:unif end aclt decr，单位: mm/s*/
extern int32_t X_forwardParam[7];	/*Kf Kp_a Kp_d Kd_a Kd_d Max Min*/
extern int32_t Y_forwardParam1[7];	/*Kf Kp_a Kp_d Kd_a Kd_d Max Min*/
extern int32_t Y_forwardParam2[7] ;	/*Kf Kp_a Kp_d Kd_a Kd_d Max Min*/
#ifdef __cplusplus
}
#endif

#endif 



