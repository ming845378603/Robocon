#ifndef __MOVE_H_
#define __MOVE_H_

#include "stm32f4xx.h"
#include "epos_N.h"
/*
	省赛使用全向轮


	Y正方向
	↑
	↑
	↑
	↑
	↑      ___车头方向___
	|      | 1 /     4 \  |
	|      |              |
	|      | 2 \     3 /  |
	|       --------------
	|
	|――→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→ X正方向

*/

#ifdef __cplusplus
extern "C" {
#endif

/*车正方向的偏离角度,逆正顺负*/
#define OFFSET_ANG (90)
/*全向轮周长 （mm）*/
//#define Omni_Wheel (127.0*3.1415926)
/*电机减速比*/
#define Reduction_Ratio (12.0)
/*车轮的实际速度x（mm/s（最大3989））与控制速度y（rpm）的转换*/
#define Speed(x) (((double)(x)/Omni_Wheel)*Reduction_Ratio*60)
#define VECT(a) (a).End.Coords.x - (a).Start.Coords.x,(a).End.Coords.y - (a).Start.Coords.y /*将向量变成以原点为起点的变量*/

#define Max_Speed 7200

void Rotate_Speed(int32_t exp_v);
void Linear_Speed(int32_t x,int32_t y,float alpha,int32_t exp_v);

void SetMotorSpeed(void);
void Motor_Stop(void);
void ClearWheelSpeed(void);

#ifdef __cplusplus
}
#endif

#endif

