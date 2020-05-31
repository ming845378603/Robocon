#include "move.h"
static int32_t wheel_spd[4] = {0};
void ClearWheelSpeed(void);

void ClearWheelSpeed(void)
{
    memset(wheel_spd, 0, sizeof(wheel_spd));
}
/**
 * @function    : rotate_speed
 * @Description : 以线速度exp_v自转
**/
void Rotate_Speed(int32_t exp_v)
{
    int32_t spd[4] = {0};
    spd[0] = -exp_v;
    spd[1] = -exp_v;
    spd[2] = exp_v;
    spd[3] = exp_v;
    //将速度添加到底盘速度中
    wheel_spd[0] += spd[0];
    wheel_spd[1] += spd[1];
    wheel_spd[2] += spd[2];
    wheel_spd[3] += spd[3];
}

/**
 * @function    : linear_speed
 * @Description : 以速度exp_v沿向量(x,y)的方向移动，alpha是车身与世界坐标y轴正方向的夹角
**/
void Linear_Speed(int32_t x, int32_t y, float alpha, int32_t exp_v)
{
    double spd_gain = 1;
    int32_t spd[4] = {0};
    int64_t Vx = x,
            Vy = y,
            V  = 0;
    alpha = -alpha + OFFSET_ANG; //获取车正方向的夹角
    V= my_sqrt(square(Vx) + square(Vy));
    if (V == 0) spd_gain = 0;
    else spd_gain = (double)exp_v / (double)V;
    Vx = x * spd_gain;
    Vy = y * spd_gain;

    spd[0] = Vx * my_sin(45 + alpha) + Vy * my_cos(45 + alpha);
    spd[2] = spd[0];
    spd[1] = Vy * my_cos(45 - alpha) - Vx * my_sin(45 - alpha);
    spd[3] = spd[1];
    //将速度添加到底盘速度中
    wheel_spd[0] += spd[0];
    wheel_spd[1] += spd[1];
    wheel_spd[2] += spd[2];
    wheel_spd[3] += spd[3];
}

/**
 * @function    :
 * @Description : 将速度发送到EPOS
**/
void SetMotorSpeed(void)
{
    int32_t spd[4] = {0};
    int32_t max_spd;		    //最大转速
    float   percent = 0.0;	    //缩放比例
    /*转化成电机的线速度*/
    spd[0] = wheel_spd[0] * 1.414;
    spd[1] = wheel_spd[1] * 1.414;
    spd[2] = wheel_spd[2] * 1.414;
    spd[3] = wheel_spd[3] * 1.414;
    //清空轮子速度
    ClearWheelSpeed();//清空了wheel_spd中存储的速度

    //求得轮子速度的绝对值的最大值
    max_spd = my_max(my_max(my_max(my_abs(spd[0]),my_abs(spd[1])),my_abs(spd[2])),my_abs(spd[3]));
    //限速
    if (max_spd > Max_Speed)
    {
        //根据比例缩放电机速度
        percent = (double)Max_Speed / (double)max_spd;
        spd[0] *= percent;
        spd[1] *= percent;
        spd[2] *= percent;
        spd[3] *= percent;
    }
    Profile_Velocity_Change(spd[0],-spd[1],-spd[2],spd[3]);
}

/**
 * @function    :
 * @Description : 电机急停
**/
void Motor_Stop(void)
{
    ClearWheelSpeed();
    Profile_Velocity_Stop();
}





