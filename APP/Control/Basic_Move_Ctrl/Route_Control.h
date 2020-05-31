#ifndef _ROUTECONTROL_H_
#define _ROUTECONTROL_H_

#include "stm32f4xx.h"
#include "vect.h"
#include "PID_forward.h"
#include "PID_Parameter.h"

#define ROUTE_CONTROL_PERIOD_MS  (CONTROL_PERIOD_MS) /*·����������ms*/
#define ROUTE_CONTROL_PERIOD_S   ((float)ROUTE_CONTROL_PERIOD_MS / 1000.0f)

//·�����ͣ�ֱ�ߣ�˳ʱ�뻡�䣬��ʱ�뻡��
enum _Route_type
{
    ROUTE_LINE = 0,
    ROUTE_TRACE_X = 1, //Ѳ������
    ROUTE_TRACE_Y = 2, //Ѳ������
    Camera_Angle_Adjust = 3, //����ͷ�����Ƕ�����
};


/*ǰ��ʽYֱ�߿���*/
void control_Xline(void);
void setup_Xline(int32_t total_len);

/*ǰ��ʽYֱ�߿���*/
void control_Yline(void);
void setup_Yline(int32_t total_len);


#endif

