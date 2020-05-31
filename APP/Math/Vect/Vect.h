#ifndef _VECT_H_
#define _VECT_H_
#include "stm32f4xx.h"
#include <stdint.h>
/*编码器点坐标
*提高精度使用double型
*/
typedef struct 
{
    float x;
    float y;
	float  Angle;
}EncodePointTypeDef;

// 表示坐标
typedef struct
{
    int32_t x;
    int32_t y;
}CoordsTypeDef;

// 某一点的信息，包括坐标和角度
typedef struct
{
    CoordsTypeDef Coords;   //坐标
    float   Angle;  //角度
}PointTypeDef;

/*向量*/
typedef struct
{
    PointTypeDef Start; //起始点
    PointTypeDef End;   //终止点
}
VectorTypeDef;

/**
 * @function    : 
 * @Description : 根据两点设置一个向量
**/

#endif
