#ifndef _PID_FORWARD_H_
#define _PID_FORWARD_H_
#include "stm32f4xx.h"
#include "My_Math.h"
#include "Time.h"
#ifdef __cplusplus
extern "C" {
#endif

/********************************ֱ�߿��Ʋ����趨*****************************/
typedef struct
{
	int32_t kf;				/*ǰ��ϵ��*/
	int32_t kp_aclt;		/*����ϵ��(�Ǽ��ٽ׶�)*/
	int32_t kp_decr;		/*����ϵ��(���ٽ׶�)*/
	int32_t kd_aclt;		/*΢��ϵ��(���ٽ׶�)*/
	int32_t kd_decr;		/*΢��ϵ��(���ٽ׶�)*/
	int32_t min_out;		/*��С���������*/
	int32_t max_out;		/*�����������*/
	int32_t vect_v;         /*����ٶȷ���*/

	float    cmd_pos[3];	/*ָ��λ��(mm)*/
	float    real_pos;		/*��ʵλ��(mm)*/
	float  	 total_len;		/*��·��(mm)*/

	float start_time;    /*��·�εĿ�ʼʱ��(ms)*/
	float end_time;		/*��·�εĽ���ʱ��(ms)*/
	float aclt_time;		/*���ٽ���ʱ���(ms)*/
	float unif_time;		/*���ٽ���ʱ��㣬uniform_time(ms)*/
	float decr_time;		/*���ٹ��̵�ʱ�䳤��(ms)*/

	float    start_v;		/*���ٶ�(mm/s)*/
	float    unif_v;		/*�����ٶ�(mm/s)*/
	float    end_v;			/*ĩ�ٶ�(mm/s)*/
	float    aclt;			/*���ٶ�(mm/s/s)*/
	float    decr;			/*���ٶ�(mm/s/s)*/

}
forward_ctl_t;


/*********************************���ȿ��Ʋ����趨*****************************/
typedef struct 
{
    int32_t x;
    int32_t y;
	  float   ang;
}
point_t;

/*����*/
typedef struct
{
    point_t s;
    point_t e;
}
vect_t;


//#define FWD_CTL_PERIOD	(Get_Cycle_T(0)) /*ǰ����������10ms*/
//#define FWD_CTL_PERIOD	(CONTROL_PERIOD_MS) /*ǰ����������10ms*/
/*��������ms*/
#define CONTROL_PERIOD_MS (20)
#define CONTROL_PERIOD_S  ((double)CONTROL_PERIOD_MS / 1000.0)

void update_forward_param(forward_ctl_t * p, int32_t * param);
void update_g_speed_param(forward_ctl_t * p, int32_t * param);

void update_forward_ctl(forward_ctl_t * p, float start_time);
int32_t forward_ctl(forward_ctl_t *p, int32_t current_pos, float current_time);
extern int32_t X_Speed[4];
extern int32_t Y_Speed[4];      /*�����ٶ�:unif end aclt decr����λ: mm/s*/
extern int32_t X_forwardParam[7];	/*Kf Kp_a Kp_d Kd_a Kd_d Max Min*/
extern int32_t Y_forwardParam1[7];	/*Kf Kp_a Kp_d Kd_a Kd_d Max Min*/
extern int32_t Y_forwardParam2[7] ;	/*Kf Kp_a Kp_d Kd_a Kd_d Max Min*/
#ifdef __cplusplus
}
#endif

#endif 



