#ifndef _ROUTECONTROL_H_
#define _ROUTECONTROL_H_

#include "stm32f4xx.h"
#include "vect.h"
#include "PID_forward.h"
#include "PID_Parameter.h"

#define ROUTE_CONTROL_PERIOD_MS  (CONTROL_PERIOD_MS) /*路径控制周期ms*/
#define ROUTE_CONTROL_PERIOD_S   ((float)ROUTE_CONTROL_PERIOD_MS / 1000.0f)

//路径类型，直线，顺时针弧弯，逆时针弧弯
enum _Route_type
{
    ROUTE_LINE = 0,
    ROUTE_TRACE_X = 1, //巡线类型
    ROUTE_TRACE_Y = 2, //巡线类型
    Camera_Angle_Adjust = 3, //摄像头矫正角度类型
};


/*前馈式Y直线控制*/
void control_Xline(void);
void setup_Xline(int32_t total_len);

/*前馈式Y直线控制*/
void control_Yline(void);
void setup_Yline(int32_t total_len);


#endif

