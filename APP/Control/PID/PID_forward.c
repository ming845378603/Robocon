#include "PID_Control.h"
#include "PID_forward.h"
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include "Robot.h"
/*设定前馈参数*/
void update_forward_param(forward_ctl_t * p, int32_t * param)
{
    p->kf      = param[0];
    p->kp_aclt = param[1];
    p->kp_decr = param[2];
    p->kd_aclt = param[3];
    p->kd_decr = param[4];	
    p->max_out = param[5];
    p->min_out = param[6];
}
void update_g_speed_param(forward_ctl_t * p, int32_t * param)
{

    p->unif_v = (float)param[0];
    p->end_v  = (float)param[1];
    p->aclt   = (float)param[2];
    p->decr   = (float)param[3];
}

float FWD_CTL_PERIOD=16.f;
/*
* 更新直线控制信息
* 程序仅运行一次，计算时间
*/
void update_forward_ctl(forward_ctl_t * p, float start_time)
{
    float aclt_len, decr_len, unif_len;

    /*加速度判断*/
    if (p->start_v == p->unif_v) p->aclt = 0;
    else                         p->aclt = (p->start_v < p->unif_v) ? fabs(p->aclt) : -fabs(p->aclt);
    if (p->end_v == p->unif_v)   p->decr = 0;
    else                         p->decr = (p->end_v < p->unif_v) ? -fabs(p->decr) : fabs(p->decr);

  	/*由滴答定时器确定初始时间*/
    p->start_time = start_time;

	  /*确定加速段的时间，单位（ms）*/
    if (p->aclt == 0) p->aclt_time = 0;
    else              p->aclt_time = 1000 * (p->unif_v - p->start_v) / p->aclt;/*加速时间长度：V=V0+aT*/
    /*确定减速段的时间，单位（ms）*/
  	if (p->decr == 0) p->decr_time = 0;
    else              p->decr_time = 1000 * (p->end_v - p->unif_v) / p->decr;/*减速时间长度：V=V0+aT*/

    aclt_len = ((p->start_v + p->unif_v) * p->aclt_time / 2) / 1000;/*加速距离，单位（mm）*/
    decr_len = ((p->unif_v + p->end_v)  * p->decr_time / 2) / 1000;	/*减速距离，单位（mm）*/
    unif_len = p->total_len - aclt_len - decr_len;					/*匀速距离，单位（mm）*/

    
		/*如果该路程不能达到满速，即速度未达到最大前就开始减速了*/
    if (unif_len < 0)
    {
        if (p->aclt > 0 && p->decr < 0) /*先加速再减速*/
        {
			      /*此时p->unif_v为过程速度的峰值vf，由公式
            2S1*a1=Vf2-V02，2S2*a2=Vf2-Vt2，S=S1+S2计算所得*/  
            p->unif_v = (float)(sqrt((p->aclt * square(p->end_v) - p->decr * square(p->start_v)
                                      - 2.0f * p->aclt * p->decr * p->total_len) / (p->aclt - p->decr)));
        }
        else/*先匀速再加速或者是再减速，忽略中间速度，直接看初速度和末速度*///需要加速度保护
        {
            float temp_aclt = (square(p->end_v) - square(p->start_v)) / 2 / p->total_len;
            /*加速度保护,如果算出来的加速度过大，那么需要改变末速度*/
            if (my_abs((temp_aclt)) <= (my_max(my_abs(p->aclt), my_abs(p->decr))))
            {
                p->aclt = temp_aclt;
                p->decr = p->aclt;
                p->unif_v = (p->start_v + p->end_v) / 2;
            }
            else
            {
                p->aclt = my_max(my_abs(p->aclt), my_abs(p->decr));
                if (p->end_v < p->start_v) p->aclt = -p->aclt;
                p->decr = p->aclt;
                p->end_v = sqrt(2 * p->aclt * p->total_len + square(p->start_v));
                p->unif_v = (p->start_v + p->end_v) / 2;
            }
        }
        /*重新计算参数*/
        if (p->aclt == 0) p->aclt_time = 0;
        else              p->aclt_time = 1000 * (p->unif_v - p->start_v) / p->aclt;/*加速时间长度，ms*/
        if (p->decr == 0) p->decr_time = 0;
        else              p->decr_time = 1000 * (p->end_v - p->unif_v) / p->decr;/*减速时间长度，ms*/

        aclt_len = ((p->start_v + p->unif_v) * p->aclt_time / 2) / 1000;	/*加速距离*/
        decr_len = ((p->unif_v + p->end_v)  * p->decr_time / 2) / 1000;	/*减速距离*/
        unif_len = p->total_len - aclt_len - decr_len;					/*匀速距离*/
    }

    p->unif_time = 1000 * unif_len / p->unif_v;					/*匀速时间长度，单位ms*/
    p->aclt_time += p->start_time;								/*计算加速过程结束时当前的时间*/
    p->unif_time += p->aclt_time;								/*计算匀速过程结束时当前的时间*/
    p->end_time = p->unif_time + p->decr_time;			/*结束时当前的时间*/

    /*更新指令位置记录*/
    p->cmd_pos[0] = -(float)(2 * p->start_v - p->aclt * FWD_CTL_PERIOD / 1000) * FWD_CTL_PERIOD / 2 / 1000;
    p->cmd_pos[1] = 0;
    p->cmd_pos[2] = 0;


    /*指令位置
    * real_pos:      当前实际位置
    * cmd_pos[0]:    上一指令位置
    * cmd_pos[1]:    当前指令位置
    * cmd_pos[2]:    下一指令位置
    */
}



/*    程序不断更新
*	forward_ctl:	前馈控制
*	p:				前馈控制结构体
*	current_pos:	当前位置
*	current_time:	当前时间
*	返回值：		速度控制量
*/
extern int32_t Target_Y;
int32_t Forward_Adjust_Flag_Y = 0, Forward_Adjust_Flag_X = 0;
extern int32_t Go_Flag;
u8 Print_Flag2 = 0;
int32_t forward_ctl(forward_ctl_t *p, int32_t current_pos, float current_time)
{
    float    v0;			/*本周期初速度*/
    float    vt;			/*本周期末速度*/
    float   time;			/*计时变量*/
    float    backward_out;	/*反馈项输出*/
    float    forward_out;   /*前馈项输出*/
    float    derive_out;    /*微分项输出*/
    float    ctl_v;			/*总的速度控制量输出*/

    FWD_CTL_PERIOD = Get_Cycle_T(0); /*前馈控制周期10ms*/
	  Print_Flag2++;
	  if(Print_Flag2 == 30)
		{
    printf("FWD_CTL_PERIOD:  %d\r\n", (int)FWD_CTL_PERIOD);
		Print_Flag2 = 0;
		}
    /*更新当前实际位置*/
    p->real_pos = current_pos;

    /*加速过程*/
    if (current_time < p->aclt_time)
    {
//        printf("# actl\r\n");
        time = current_time - p->start_time;
        v0 = time * p->aclt / 1000 + p->start_v;
        time += FWD_CTL_PERIOD;
        vt = time * p->aclt / 1000 + p->start_v;

    }
    /*匀速过程*/
    else if (current_time < p->unif_time)
    {
//        printf("# unif\r\n");
        v0 = p->unif_v;
        vt = p->unif_v;
    }
    /*减速过程*/
    else
    {
        if (current_time < p->end_time) //如果没有超时
        {
            time = current_time - p->unif_time;
            v0 = p->unif_v + time*p->decr / 1000;
            time += FWD_CTL_PERIOD;

            /*防止时间不是FWD_CTL_PERIOD整数倍出现负数的情况*/
            if (time > p->decr_time)
            {
                time = p->decr_time;
            }
            vt = p->unif_v + time * p->decr / 1000;

        }
        else //if (current_time >= p->end_time) 如果超时说明这条路径是最后一条路径了
        {
            v0 = 0;
            vt = 0;
        }
    }

    /*指令位置
    * real_pos:      当前实际位置
    * cmd_pos[0]:    上一指令位置
    * cmd_pos[1]:    当前指令位置
    * cmd_pos[2]:    下一指令位置
    */
    p->cmd_pos[2] += ((v0 + vt) / 2) * FWD_CTL_PERIOD / 1000;
    if (p->cmd_pos[2] > p->total_len) p->cmd_pos[2] = p->total_len;                //防止运算超过最大路程
//    printf("cmd_pos[2]:  %d\r\n", (int)p->cmd_pos[2]);
//    printf("cmd_pos[1]:  %d\r\n", (int)p->cmd_pos[1]);
		
    forward_out = p->kf * (p->cmd_pos[2] - p->cmd_pos[1]);							/*前馈控制输出项*/
		
    if (current_time < p->unif_time)
    {
        backward_out = p->kp_aclt * (p->cmd_pos[1] - p->real_pos);					    /*反馈比例项*/  //根据你当前的位置修正反馈输出 如果实际位置和指令位置一样则反馈调节为0
        derive_out = p->kd_aclt * (p->cmd_pos[2] - 2 * p->cmd_pos[1] + p->cmd_pos[0]);	/*指令位置微分项*/
    }
    else
    {
        backward_out = p->kp_decr * (p->cmd_pos[1] - p->real_pos) / 10.f;					    /*反馈比例项*/
        derive_out = p->kd_decr * (p->cmd_pos[2] - 2 * p->cmd_pos[1] + p->cmd_pos[0]);	/*指令位置微分项*/
    }

    ctl_v = backward_out + forward_out + derive_out;  							       	/*总控制量*/

    /*更新指令位置*/
    p->cmd_pos[0] = p->cmd_pos[1];
    p->cmd_pos[1] = p->cmd_pos[2];

    return ctl_v;
}


uint32_t get_vect_len(vect_t *p_vect)
{
	int64_t x, y, l;
	x = p_vect->e.x - p_vect->s.x;
	y = p_vect->e.y - p_vect->s.y;
	l = my_sqrt(square(x)+square(y));
	return l;
}











