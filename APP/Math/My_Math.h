//-----------------------------------------------------------------------------
// My_Math.h
//
//  Created on	: 2016-10-7
//      Author	: DGVY
//		version	: V1.0
//		brief	: 常用的数学计算函数
//-----------------------------------------------------------------------------
// Attention:
//

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#ifndef __MY_MATH_H
#define __MY_MATH_H

//-----------------------------------------------------------------------------
// Define to prevent recursive inclusion
//-----------------------------------------------------------------------------
//计算用宏
#include <math.h>
#include "string.h"
#include "stdlib.h"
#include "stdint.h"
//#include "arm_math.h"


// 圆周率
#define pi 										3.141592653589793

// 弧度变角度
#define rad2ang(rad) 							( (double)(rad)/pi*180.0f )

// 角度变弧度
#define ang2rad(ang) 							( (double)(ang)*pi/180.0f )

// 平方
#define square(x)    							( (x)*(x) )

// 开方
#define my_sqrt(x)  							_sqrtf(x)

// 三角函数
#define my_sin(x)  								sinf(ang2rad(x))
#define my_cos(x)  								cosf(ang2rad(x))

// 反三角函数
#define my_acos(x) 								rad2ang(acosf(x))
#define my_asin(x) 								rad2ang(asinf(x))

// 最大值
#define my_max(a,b) 							( (a)>(b) ? (a) : (b) )

// 最小值
#define my_min(a,b)								( (a)<(b) ? (a) : (b) )

// 求绝对值
#define my_abs(num)  								(((num)>0)?(num):-(num)) 
// 求两个数差值的绝对值
#define my_minus_abs(num1,num2)                 ((num1>num2)?(num1-num2):(num2-num1))

//浮点数相关
#define float_equal(a,b) 					    ( fabsf((a)-(b))<0.000001f )
#define float_not_equal(a,b) 			        ( fabsf((a)-(b))>0.000001f )
#define float_greater(a,b) 				        ( (a)-(b)>0.000001f )
#define float_smaller(a,b) 				        ( (a)-(b)<-0.000001f )

#define f_equal(a,b)   (fabsf((a)-(b))<0.000001f)
#define f_unequal(a,b) (fabsf((a)-(b))>0.000001f)
#define f_greater(a,b) ((a)-(b)>0.000001f)
#define f_smaller(a,b) ((a)-(b)<-0.000001f)

void Bubble_Sort(uint32_t _data[], uint32_t size);


#endif /* __MY_MATH_H */
/******************* (C) COPYRIGHT 2016 DGVY **********END OF FILE***********/
