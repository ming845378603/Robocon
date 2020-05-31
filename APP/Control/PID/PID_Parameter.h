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

    int32_t Out_Max;      //输出限幅
    int32_t Out_Min;
    int32_t Iteg_Max;     //积分限幅
    int32_t Dead_Zone;    //死区
    int32_t Iteg_A;       //连续变速积分A
    int32_t Iteg_B;       //连续变速积分B
} sPID_Parameter_TypeDef;

enum _now_last
{
    NOW = 1,
    LAST = 0,
};

typedef struct
{
    int32_t set_value;       /*给定值*/
    int32_t real_value[2];   /*输出值(实际值)*/
    int32_t err[2];          /*偏差*/
    int32_t pid_out;         /*PID运算结果*/
    int32_t err_iteg;        /*误差积分*/
} sPID_Calculte_TypeDef;


typedef struct
{
    sPID_Parameter_TypeDef Parameter;
    sPID_Calculte_TypeDef Calculate;
} sPID_Loop_TypeDef;

typedef struct      //PID环参数
{

    sPID_Loop_TypeDef PID_Angle;
	sPID_Loop_TypeDef PID_LineX;
    sPID_Loop_TypeDef PID_LineY;

} sPID_TypeDef;   //PID环参数



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
extern int32_t Y_Speed[4];      /*设置速度:unif end aclt decr，单位: mm/s*/
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

