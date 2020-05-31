//-----------------------------------------------------------------------------
// GYRO_Lib.h
//
//  Created on	: 2016-9-7
//      Author	: DGVY
//		version	: V3.0
//		brief	: 陀螺仪控制头文件
//-----------------------------------------------------------------------------
// Attention:
//
/*
///////////////////////////////////////////////////////////////////////////////
* 修改日志：
*******************************************************************************
* v1.0
* 创建文件
*******************************************************************************
* v2.0
* 将原本的C文件改为C++文件
* 创建CGYRO类
*******************************************************************************
* v3.0
* 由于编译器有问题，要把C++改回C
* 我真tm智障
///////////////////////////////////////////////////////////////////////////////
*/

//-----------------------------------------------------------------------------
// Define to prevent recursive inclusion
//-----------------------------------------------------------------------------
#ifndef __GYRO_LIB_H
#define __GYRO_LIB_H

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include <stdint.h>
#include <string.h>
#include "DataConverTypeDef.h"
#include "My_Flag.h"
#include "vect.h"
// 宏定义选择初始化
//#define USE_INIT_INNER      //使用此文件内的CAN初始化函数
#define USE_INIT_OUTER    //使用此文件外的CAN初始化函数

#if defined (USE_INIT_INNER)
#elif defined (USE_INIT_OUTER)
#else
#error "GYRO_CAN can't inition!"
#endif


//通过宏定义选择通信的CAN总线
//#define GYRO_USE_CAN1
#define GYRO_USE_CAN2

#if defined (GYRO_USE_CAN1)
#define CAN_BUS CAN1
#elif defined (GYRO_USE_CAN2)
#define CAN_BUS CAN2
#else
#error "Please choose GYRO_CAN_Bus!"
#endif

//是否使用本文件内的CAN中断函数
#define GYRO_INTERRUPT_INNER_ENABLE


#define POS_CID 0x12		//用户控制器 ID
#define GYRO_ID 0x11        //模块 ID 

#define Coder_Param (double)0.0392699081698724   //编码器轮子直径50.8 * π / 脉冲数2000

// 陀螺仪读取的位置
extern __IO EncodePointTypeDef GYRO_Location;
extern Flag Flag_GYRO_Read;

// 陀螺仪模块初始化
void GYRO_Init(void);

// 设置角度
void GYRO_Set_Angle(float angle);

// 设置 x,y 坐标
void GYRO_Set_Position(int32_t pos_x, int32_t pos_y);
void GYRO_Set_Position_X(int32_t x);

// 取得GYRO坐标
PointTypeDef GYRO_Get_Location(void);


#endif /* __GYRO_LIB_H */
/******************* (C) COPYRIGHT 2016 DGVY **********END OF FILE***********/
