//-----------------------------------------------------------------------------
// Robot.h
//
//  Created on	: 2017-1-8
//      Author	: DGVY
//		version	: V1.0
//		brief	:
//-----------------------------------------------------------------------------
// Attention:
//

//-----------------------------------------------------------------------------
// Define to prevent recursive inclusion
//-----------------------------------------------------------------------------
#ifndef __ROBOT_H
#define __ROBOT_H

#ifdef __cplusplus  
extern "C" {
#endif  

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include "Vect.h"
#include "My_Flag.h"

//-----------------------------------------------------------------------------
// Private Typedef
//-----------------------------------------------------------------------------

typedef enum        // 模式选择
{
    Lock_Mode = 0x00,
	Runto_Junction = 0xA1,
	Line_Adjust = 0xA4,
}eRobot_Mode_TypeDef;

typedef enum 
{
    Running,        //正在运动
    Pause,          //暂停
    Stop,           //停止
    SoftReseting    //软件复位
}eRobot_State_TypeDef;

typedef enum
{
    No_Set,         //未设置
    Ground_Red,     //红场
    Ground_Blue     //蓝场
}eGround_Set_TypeDef;

typedef struct      // 机器人属性
{
	/*系统信息*/
	uint32_t      cur_time_ms; /*当前系统时间ms*/
	double       cur_speed;   /*当前速度m/s*/
	VectorTypeDef spd_vect;    /*上一个周期的坐标指向当前周期坐标的速度向量*/
    eGround_Set_TypeDef Ground_Set;

    // 位置信息
    PointTypeDef Now_Position;      // 当前位置
    PointTypeDef Last_Position;     // 上一个位置
	PointTypeDef Start_Point;     /*起点坐标*/
    PointTypeDef Target_Location;   //目标位置
	PointTypeDef Next_Jun_pos;    //交叉点位置

    // 模式信息
    eRobot_Mode_TypeDef Mode;

    // 状态信息
    eRobot_State_TypeDef State;

    // 工作台连接标志
    eConnectStatues WorkPanel_Connect;
}sRobotTypeDef;

//-----------------------------------------------------------------------------
// Private Define
//-----------------------------------------------------------------------------
#define Address_UnderPan  (uint8_t)0x11
#define Address_Shoot     (uint8_t)0xA1
#define Address_Remote    (uint8_t)0xA2
#define Address_WorkPanel (uint8_t)0xA3

#define DEBUG_PRINTF     //串口打印

extern sRobotTypeDef G_Robot_Master;
/******************************************************************************/

void Robot_Speed_Upadte(void);
void Robot_Location_Update(void);
void Robot_Send_Location_to_Remote(void);
void Line_Adjust_Task_Mode_Set(void);
void Lock_Point_Task_Set(void);
void Runto_Junction_Task_Set(void);
void DelTask(void);
void FS_Emend(void);
#define GetCurTimeMS(a) ((OSTimeGet(a))*5)              /*获取当前时间*/
#define nloc            G_Robot_Master.Now_Position     //当前位置
#define jun_pos         G_Robot_Master.Next_Jun_pos     //交叉点位置

#ifdef __cplusplus  
}
#endif 

#endif /* __ROBOT_H */
/******************* (C) COPYRIGHT 2016 DGVY ***********END OF FILE***********/

