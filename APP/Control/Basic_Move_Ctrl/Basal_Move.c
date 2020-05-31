#include "Basal_Move.h"
#include "My_Math.h"
#include "PID_Parameter.h"
#include "vect.h"
#include "Time.h"
#include "Robot.h"
#include "sys.h"
#include "uart4.h"
#include "PID_Control.h"
#include "usart.h"
#include "uart4.h"
static int32_t wheel_spd[4] = {0};
extern s16 targetX, targetY1, targetA;
extern u8 Sircle_flag;
extern int DT35_DataA,DT35_DataB,DT35_DataC;//数据接收
void ClearWheelSpeed(void);

void ClearWheelSpeed(void)
{
    memset(wheel_spd, 0, sizeof(wheel_spd));
}
/**
 * @function    : rotate_speed
 * @Description : 以线速度exp_v自转,正数逆时针，负数顺时针
**/
void Camera_rotate_speed(int32_t exp_v)
{
    int32_t spd[4] = { 0 };

    spd[0] = exp_v;
    spd[1] = exp_v;
    spd[2] = 0;
    spd[3] = 0;

    //将速度添加到底盘速度中
    wheel_spd[0] += spd[0];
    wheel_spd[1] += spd[1];
    wheel_spd[2] += spd[2];
    wheel_spd[3] += spd[3];
}



void rotate_speed(int32_t exp_v)
{
    int32_t spd[4] = {0};

    spd[0] = exp_v;
    spd[1] = exp_v;
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
void linear_speed(int32_t x, int32_t y, float alpha, int32_t exp_v)
{
    double spd_gain = 1;
    int32_t spd[4] = {0};
    int64_t Vx = x,
            Vy = y,
            V  = 0;
    alpha = alpha + OFFSET_ANG; //获取车正方向的夹角
    V  = my_sqrt(square(Vx) + square(Vy));
    if (V == 0) spd_gain = 0;
    else spd_gain = (double)exp_v / (double)V;
    Vx = x * spd_gain;
    Vy = y * spd_gain;

    /*带角度三轮*/
//    spd[0] = -Vy * my_sin(60 + alpha ) - Vx * my_cos(60 + alpha);
//    spd[1] = Vy * my_sin(60 - alpha ) - Vx * my_cos(60 - alpha );
//    spd[3] = Vy * my_sin(alpha) + Vx * my_cos(alpha );//2017.11.15.19.35 改了坐标方向最好再看看对不
		/*带角度四轮*/
	  spd[0] = my_cos(alpha-135)*Vx + my_sin(alpha-135)*Vy;
	  spd[1] = my_cos(alpha-45)*Vx + my_sin(alpha-45)*Vy;
	  spd[2] = my_cos(alpha+45)*Vx + my_sin(alpha+45)*Vy;
	  spd[3] = my_cos(alpha+135)*Vx + my_sin(alpha+135)*Vy;
		

    //将速度添加到底盘速度中
    wheel_spd[0] += spd[0];
    wheel_spd[1] += spd[1];
    wheel_spd[2] += spd[2];
    wheel_spd[3] += spd[3];
}

/**
 * @function    :
 * @Description : 将速度发送到elmo
 * 三轮，只用了spd0 1 3
**/
extern EncodePointTypeDef GYRO_Location;
int32_t Stop_flag=0;
extern int32_t Task_Go_Flag;;
int32_t spd[4] = {0};
int32_t Vxx=0,Vyy=0;
#define K_angle 0x00
u8 Print_Flag1 = 0;
void SetMotorSpeed(void)
{
    int32_t max_spd;		    //最大转速
    float   percent = 0.0;	    //缩放比例
    //*电机线速度转换成转速  因为你给EPOS发的指令是r/min  这次用的减速比是J=21的,轮子直径是125mm
    //转速n=wheel_spd[]*60*J/(R*2*3.14)   所以转速n=wheel_spd[]*60*21/(125*3.14)=wheel_spd[]*3.21*/
    spd[0] = wheel_spd[0] * 3.21;
    spd[1] = wheel_spd[1] * 3.21;
    spd[2] = wheel_spd[2] * 3.21;
    spd[3] = wheel_spd[3] * 3.21;

    /*电机线速度转换成转速  因为你给EPOS发的指令是r/min  这次用的减速比是J=12的,轮子直径是125mm
    转速n=wheel_spd[]*60*J/(R*2*3.14)   所以转速n=wheel_spd[]*60*12/(125*3.14)=wheel_spd[]*1.83*/
//    spd[0] = wheel_spd[0] * 1.83;
//    spd[1] = wheel_spd[1] * 1.83;
//    spd[2] = wheel_spd[2] * 1.83;
//    spd[3] = wheel_spd[3] * 1.83;
	
	
    //清空轮子速度
    ClearWheelSpeed();

    //求得轮子速度的绝对值的最大值
    max_spd = my_max(my_max(my_max(my_abs(spd[0]),my_abs(spd[1])),my_abs(spd[2])),my_abs(spd[3]));
    //限速
    if (max_spd > MAX_SPEED)
    {
        //根据比例缩放电机速度
        percent = (double)MAX_SPEED / (double)max_spd;
        spd[0] *= percent;
        spd[1] *= percent;
        spd[2] *= percent;
        spd[3] *= percent;
    }
		
     if (Stop_flag == 1)
    {
        Motor_Stop();
    }
    else
    {
        ELMO_Velocity(spd[0], spd[1], spd[2], spd[3]);
    }

    int v0 = spd[0], v1 = spd[1], v2 = spd[2], v3 = spd[3];
		Print_Flag1++;
		if(Print_Flag1 == 30)
		{
		printf("v1:  %d\tv2:  %d\tv3:  %d\tv4:  %d\t\r\n", v0,v1,v2,v3);
    printf("%d\t%d\t%d\r\n", nloc.Coords.x, nloc.Coords.y ,(int)nloc.Angle);
		printf("targetX:  %d\ttargetY1:  %d\ttargetA:  %d\t\r\n",targetX,targetY1,targetA);
		printf("DT35_DataA:  %d\tDT35_DataB:  %d\tDT35_DataC:  %d\t\r\n",
       			DT35_DataA,DT35_DataB,DT35_DataC); 
		Print_Flag1 = 0;
		}
}

/**
* @function    :
* @Description : 电机急停
**/
void Motor_Stop(void)
{
    ClearWheelSpeed();
		spd[0] = 0;  spd[1] = 0; spd[2] = 0; spd[3] = 0; 
	  ELMO_Velocity(spd[0], spd[1], spd[2], spd[3]);
}

