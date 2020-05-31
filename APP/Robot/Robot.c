#include "Robot.h"
#include "GYRO_Lib.h"
#include "delay.h"
#include <stdio.h>
#include "Vect.h"
#include "Basal_Move.h"
#include "DataConverTypeDef.h"
#include <stdint.h>
#include "My_Math.h"
#include "Time.h"


sRobotTypeDef G_Robot_Master;


void Robot_Speed_Upadte(void);
void Robot_Location_Update(void);

void Robot_Location_Update(void)
{
	nloc = GYRO_Get_Location();
}

/**
 * @function    : 
 * @Description : 根据两点设置一个向量
**/
static void set_vect(VectorTypeDef *p_vect, PointTypeDef *p_start, PointTypeDef *p_end)
{
	memcpy(&p_vect->Start, p_start, sizeof(PointTypeDef));
	memcpy(&p_vect->End, p_end,   sizeof(PointTypeDef));
}

////获取向量长度
static uint32_t get_vect_len(VectorTypeDef *p_vect)
{
	int64_t x, y, l;
	x = p_vect->End.Coords.x - p_vect->Start.Coords.x;
	y = p_vect->End.Coords.y - p_vect->Start.Coords.y;
	l = my_sqrt(square(x)+square(y));
//	printf("\r\n%d\t%d\t%d\t%d\r\n",p_vect->End.Coords.y,p_vect->Start.Coords.y,(int32_t)y,(int32_t)l);
	return l;
}


float Cycle_T_1;

// 更新速度及速度向量
void Robot_Speed_Upadte(void)
{
    // 更新速度向量
    set_vect(&G_Robot_Master.spd_vect, &G_Robot_Master.Last_Position, &G_Robot_Master.Now_Position);
	   /*计算外环时间*/
	Cycle_T_1 = Get_Cycle_T(1)/1000.f;
	   /*计算当前速度*/
    G_Robot_Master.cur_speed = (double)get_vect_len(&G_Robot_Master.spd_vect) / (double)Cycle_T_1;
//	
   //printf("Cycle_T_1 : \r\n%d\r\n", (int)(Cycle_T_1*1000) );
		memcpy(&G_Robot_Master.Last_Position, &G_Robot_Master.Now_Position, sizeof(PointTypeDef));
//    /*计算当前速度*/
//    G_Robot_Master.cur_speed = (double)get_vect_len(&G_Robot_Master.spd_vect) / (0.005);
}


///******************* (C) COPYRIGHT 2016 DGVY ***********END OF FILE***********/
