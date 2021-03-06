#ifndef _BASAL_MOVE_H_
#define _BASAL_MOVE_H_

#include "stm32f4xx.h"
#include "epos_N.h"
#include "elmo_drive.h"
/*
    以麦克纳姆轮为例，其余的底盘类型是一样的
    默认电机编号是逆时针排列如下图，电机正方向转动的时候底盘向前运动
    左上角第一个电机为1号，电机正方向如箭头

    Y正方向
    ↑
    ↑
    ↑
    ↑
    ↑      ___车头方向___
    |      | 1 ↑    4 ↑ |
    |      |            |
    |      | 2 ↑    3 ↑ |
    |      ---------------
    |
    |――→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→ X正方向
*/
/*车正方向的偏离角度,逆正顺负*/
#define OFFSET_ANG (0)
/*麦克纳姆轮周长 mm*/
#define WHEEL4_PERIMETER (150 * 3.1415926)
/*电机减速比*/
#define REDUCTION_RATIO (21.0)
//车实际速度和控制速度的转化，x是mm/s
//#define SPEED(x) (((double)(x) / WHEEL4_PERIMETER) * REDUCTION_RATIO * 2000)
/*其他宏*/
//#define VECT(a) (a).End.Coords.x - (a).Start.Coords.x,(a).End.Coords.y - (a).Start.Coords.y /*将向量变成以原点为起点的向量*/
#define VECT(a) (a).e.x - (a).s.x,(a).e.y - (a).s.y /*将向量变成以原点为起点的变量*/

//车轮的最大速度  最大转速9120
#define MAX_SPEED ((double)15000)



void rotate_speed(int32_t exp_v);
void linear_speed(int32_t x, int32_t y, float alpha, int32_t exp_v);

void SetMotorSpeed(void);
void UnderpanBrake(void);
void Motor_Stop(void);

#endif /*_BASAL_MOVE_H_*/
