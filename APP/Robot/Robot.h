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

typedef enum        // ģʽѡ��
{
    Lock_Mode = 0x00,
	Runto_Junction = 0xA1,
	Line_Adjust = 0xA4,
}eRobot_Mode_TypeDef;

typedef enum 
{
    Running,        //�����˶�
    Pause,          //��ͣ
    Stop,           //ֹͣ
    SoftReseting    //�����λ
}eRobot_State_TypeDef;

typedef enum
{
    No_Set,         //δ����
    Ground_Red,     //�쳡
    Ground_Blue     //����
}eGround_Set_TypeDef;

typedef struct      // ����������
{
	/*ϵͳ��Ϣ*/
	uint32_t      cur_time_ms; /*��ǰϵͳʱ��ms*/
	double       cur_speed;   /*��ǰ�ٶ�m/s*/
	VectorTypeDef spd_vect;    /*��һ�����ڵ�����ָ��ǰ����������ٶ�����*/
    eGround_Set_TypeDef Ground_Set;

    // λ����Ϣ
    PointTypeDef Now_Position;      // ��ǰλ��
    PointTypeDef Last_Position;     // ��һ��λ��
	PointTypeDef Start_Point;     /*�������*/
    PointTypeDef Target_Location;   //Ŀ��λ��
	PointTypeDef Next_Jun_pos;    //�����λ��

    // ģʽ��Ϣ
    eRobot_Mode_TypeDef Mode;

    // ״̬��Ϣ
    eRobot_State_TypeDef State;

    // ����̨���ӱ�־
    eConnectStatues WorkPanel_Connect;
}sRobotTypeDef;

//-----------------------------------------------------------------------------
// Private Define
//-----------------------------------------------------------------------------
#define Address_UnderPan  (uint8_t)0x11
#define Address_Shoot     (uint8_t)0xA1
#define Address_Remote    (uint8_t)0xA2
#define Address_WorkPanel (uint8_t)0xA3

#define DEBUG_PRINTF     //���ڴ�ӡ

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
#define GetCurTimeMS(a) ((OSTimeGet(a))*5)              /*��ȡ��ǰʱ��*/
#define nloc            G_Robot_Master.Now_Position     //��ǰλ��
#define jun_pos         G_Robot_Master.Next_Jun_pos     //�����λ��

#ifdef __cplusplus  
}
#endif 

#endif /* __ROBOT_H */
/******************* (C) COPYRIGHT 2016 DGVY ***********END OF FILE***********/

