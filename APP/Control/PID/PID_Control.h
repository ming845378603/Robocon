//-----------------------------------------------------------------------------
// PID_Angle.h
//
//  Created on	: 2016-1-7
//      Author	: DGVY
//		version	: V1.0
//		brief	:
//-----------------------------------------------------------------------------
// Attention:
//

//-----------------------------------------------------------------------------
// Define to prevent recursive inclusion
//-----------------------------------------------------------------------------
#ifndef __PID_ANGLE_H
#define __PID_ANGLE_H

#ifdef __cplusplus
extern "C" {
#endif

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------

#include <stdint.h>
#include "PID_Parameter.h"



void PID_Set_Angle_Value(int32_t t_angle,int32_t Route_type);
void PID_Set_LineY_Value(int32_t t_y,int32_t Route_type);
void PID_Set_LineX_Value(int32_t t_x, int32_t Route_type);
//void PID_Control(void);
void PID_Angle_Loop_Test(void);
void PID_LineY_Loop_Test(void);
void PID_LineX_Loop_Test(void);
//void update_pid_param(sPID_Loop_TypeDef * pid, double setvalue);
int32_t PID_Calculate(sPID_Loop_TypeDef * pid_loop, int32_t real_value);
void PID_Lock_LineYPoint(void);
void PID_Lock_LineXPoint(void);
void PID_BackTo_StartPoint(void);

#ifdef __cplusplus
}
#endif

#endif /* __PID_ANGLE_H */
/******************* (C) COPYRIGHT 2016 DGVY ***********END OF FILE***********/

