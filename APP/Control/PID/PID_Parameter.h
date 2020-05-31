//-----------------------------------------------------------------------------
// PID_Parameter.h
//
//  Created on	: 2017-1-7
//      Author	: DGVY
//		version	: V1.0
//		brief	:
//-----------------------------------------------------------------------------
// Attention:
//

//-----------------------------------------------------------------------------
// Define to prevent recursive inclusion
//-----------------------------------------------------------------------------
#ifndef __PID_PARAMETER_H
#define __PID_PARAMETER_H
#include "My_Math.h"
//#include "Global.h"

#ifdef __cplusplus
extern "C" {
#endif

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Private Typedef
//-----------------------------------------------------------------------------
/*PID*/
typedef struct
{
    int32_t P;
    int32_t I;
    int32_t D;

    int32_t Out_Max;      //����޷�
    int32_t Out_Min;
    int32_t Iteg_Max;     //�����޷�
    int32_t Dead_Zone;    //����
    int32_t Iteg_A;       //�������ٻ���A
    int32_t Iteg_B;       //�������ٻ���B
} sPID_Parameter_TypeDef;

enum _now_last
{
    NOW = 1,
    LAST = 0,
};

typedef struct
{
    int32_t set_value;       /*����ֵ*/
    int32_t real_value[2];   /*���ֵ(ʵ��ֵ)*/
    int32_t err[2];          /*ƫ��*/
    int32_t pid_out;         /*PID������*/
    int32_t err_iteg;        /*������*/
} sPID_Calculte_TypeDef;


typedef struct
{
    sPID_Parameter_TypeDef Parameter;
    sPID_Calculte_TypeDef Calculate;
} sPID_Loop_TypeDef;

typedef struct      //PID������
{

    sPID_Loop_TypeDef PID_Angle;
	sPID_Loop_TypeDef PID_LineX;
    sPID_Loop_TypeDef PID_LineY;

} sPID_TypeDef;   //PID������



//-----------------------------------------------------------------------------
// Private Define
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Private Macro
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Exported constant
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Extern Variables
//-----------------------------------------------------------------------------
extern sPID_TypeDef G_PID_Parameter[7];
extern int32_t forwardParam[7];
extern int32_t g_Speed[4];
extern int32_t X_Speed[4];
extern int32_t Y_Speed[4];      /*�����ٶ�:unif end aclt decr����λ: mm/s*/
extern int32_t X_forwardParam[7];	/*Kf Kp_a Kp_d Kd_a Kd_d Max Min*/
extern int32_t Y_forwardParam[7];	/*Kf Kp_a Kp_d Kd_a Kd_d Max Min*/
//-----------------------------------------------------------------------------
// Exported functions
//-----------------------------------------------------------------------------


#ifdef __cplusplus
}
#endif

#endif /* __PID_PARAMETER_H */
/******************* (C) COPYRIGHT 2016 DGVY ***********END OF FILE***********/

