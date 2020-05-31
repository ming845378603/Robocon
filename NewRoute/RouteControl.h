#ifndef _ROUTECONTROL_H_
#define _ROUTECONTROL_H_ 

#include "stm32f4xx.h"
#include "NewVect.h"
//#include "Global.h"
#define ROUTE_CONTROL_PERIOD_MS  (CONTROL_PERIOD_MS) /*路径控制周期ms*/
#define ROUTE_CONTROL_PERIOD_S   ((float)ROUTE_CONTROL_PERIOD_MS / 1000.0f)

#define SEESAW_FORWARD_POS_KF (-22) 
#define SEESAW_FORWARD_ANG_KF (G_Param.ground_type == GROUND_BLUE? (25) : (-25))  

/*****************************************************************************/

/*位置式PID*/
typedef struct
{
    double set_value;       /*给定值*/
    double real_value[2];   /*输出值(实际值)*/
    double err[2];          /*偏差*/
    double pid_out;         /*PID运算结果*/
    double err_iteg;        /*误差积分*/
    /*Param*/
    double Kp;              /*KP*/
    double Ki;              /*Ki*/
    double Kd;              /*Kd*/
    double out_max;         /*PID运算结果的最大值*/
    double out_min;         /*PID运算结果的最小值*/
    double iteg_max;        /*积分限幅*/
    double dead_zone;       /*死区*/
}
pid_t;




/*前馈参数列表*/
enum forward_index
{
    NORMAL_LINE_FORWARD_PARAM = 0,
    NORMAL_ROT_FORWARD_PARAM,
    NORMAL_ARC_FORWARD_PARAM,
    POLE_ROT_FORWARD_PARAM,
    FORWARD_NUM,
};

/*PID参数列表*/
enum pid_index
{
    PID_LOCK_LINE_PARAM = 0, //锁定直线轨迹PID
    PID_LOCK_ARC_PARAM,      //锁定弧线轨迹PID
    PID_LOCK_ANG_PARAM,      //锁定角度PID
    PID_LOCK_TRACE_LOCUS,    //巡线锁定轨迹
    PID_LOCK_TRACE_ANGLE,    //巡线锁定角度
    PID_LOCK_POINT,          //锁点PID
    PID_SWING_TRACE_LOCUS,   //秋千锁点PID
    PID_POLE_TRACE_LOCUS,    //梅花桩DT50巡线
    PID_LIDAR_LOCK,          //激光雷达锁点
    
    PID_LOCK_TRACE_LOCUS_0SPEED,//跷跷板巡线停止
    PID_LOCK_TRACE_ANGLE_0SPEED, //跷跷板巡线停止
    
    PID_LOCK_TRACE_LOCUS_HANDOVER, //跷跷板交接
    PID_LOCK_TRACE_ANGLE_HANDOVER, //跷跷板交接
    PID_NUM,
};

/*****************************************************************************/

/*巡线信息*/
typedef struct
{
    uint8_t sensor[2];    /*巡线用的传感器*/
    int32_t set_value[2]; /*目标值*/
    vect_t  trace_vect;   /*减少误差的方向*/
    vect_t  move_vect;    /*相对车身的运动方向*/
}trace_inf_t;

/*速度参数*/
typedef struct
{
	int32_t unif_v;		   	/*匀速速度*/
	int32_t end_v;			/*末速度*/
	int32_t aclt;			/*加速度*/
	int32_t decr;			/*减速度*/
}							/*初速度为车的实际速度*/
spd_inf_t;

/*路径信息数组*/
typedef struct 
{
	point_t     start;			  /*起点坐标*/
	point_t     end;		      /*终点坐标*/
	point_t     center;			  /*圆心坐标*/
	int8_t      route_type;		  /*路径类型：直线or弧弯(方向),巡线(类型+方向),锁点*/
	spd_inf_t   spd_inf;		  /*路段的速度控制信息*/
	uint8_t     forward_param[2]; /*前馈控制参数*/
	uint8_t     pid_param[2];     /*PID控制参数*/
	trace_inf_t trace_inf;        /*巡线信息*/
}
route_inf_t;
/*****************************************************************************/
//路径类型，直线，顺时针弧弯，逆时针弧弯
enum _route_type
{
    ROUTE_LINE = 0,
    ROUTE_CLK = -1,
    ROUTE_ANTI_CLK = 1,
    ROUTE_LOCK = 2,
    ROUTE_TRACE = 3, //巡线类型
};

/*路径更新*/
void UpdateRouteCtl(route_inf_t * NewRoute);
/*路径控制*/
void RouteControl(void);
/*获得路径结束时间*/
uint32_t GetRouteEndTime(void);

void SetupLine(int32_t x0, int32_t y0, float a0,
               int32_t x1, int32_t y1, float a1,
               int32_t * spd,
               uint8_t fwd1, uint8_t fwd2,
               uint8_t pid1, uint8_t pid2);
               
void SetupSeesawTrace(int32_t x0, int32_t y0, float a0,
                      int32_t x1, int32_t y1, float a1,
                      int32_t cx, int32_t cy,
                      uint8_t sensor1, uint8_t sensor2,
                      int32_t set_value1, int32_t set_value2,
                      int32_t * spd,
                      uint8_t fwd1, uint8_t fwd2,
                      uint8_t pid1, uint8_t pid2);
                      
void SetupTrace(uint8_t sensor1, uint8_t sensor2,
              int32_t set_value1, int32_t set_value2,
              int32_t * spd,
              uint8_t pid1, uint8_t pid2);

void SetupArc(int32_t x0, int32_t y0, float a0,
              int32_t x1, int32_t y1, float a1,
              int32_t cx, int32_t cy, int8_t type,
              int32_t * spd,
              uint8_t fwd1, uint8_t fwd2,
              uint8_t pid1, uint8_t pid2);

void SetupLock(int32_t x0, int32_t y0, float a0,
               uint8_t pid1, uint8_t pid2);


/*走场坐标*/
/*蓝场*/
extern const point_t StartPoint_BLUE;
extern point_t SeesawPoint_BLUE; //*切圆的点 0x00
extern const uint8_t SeesawPoint_BLUE_addr;
extern point_t SwingPoint_BLUE;//*切入秋千巡线的点 0x00 + 12
extern const uint8_t SwingPoint_BLUE_addr;
extern point_t PolePutPoint_BLUE; //*梅花桩放交接巡线之前 0x00 + 12 * 2
extern const uint8_t PolePutPoint_BLUE_addr;

extern int32_t SeesawCam_BLUE[2]; // 0x00 + 12 * 2 + 4, 0x00 + 12* 2 + 8
extern const uint8_t SeesawCam_BLUE_addr[2];
extern int32_t SwingCam_BLUE;//0x00 + 12*2 + 8 + 4
extern const uint8_t SwingCam_BLUE_addr;
extern int32_t DT50_PUT_BLUE;//0x00 + 12*2 + 8 + 4*2
extern const uint8_t DT50_PUT_BLUE_addr;
extern int32_t DT50_GET_BLUE;//0x00 + 12*2 + 8 + 4*3
extern const uint8_t DT50_GET_BLUE_addr;

/*红场*/
extern const point_t StartPoint_RED;
extern point_t SeesawPoint_RED; //切圆的点 0x32
extern const uint8_t SeesawPoint_RED_addr;
extern point_t SwingPoint_RED;//切入秋千巡线的点 0x32 + 12
extern const uint8_t SwingPoint_RED_addr;
extern point_t PolePutPoint_RED; //梅花桩放交接巡线之前 0x32+12*2
extern const uint8_t PolePutPoint_RED_addr;

extern int32_t SeesawCam_RED[2];
extern const uint8_t SeesawCam_RED_addr[2];
extern int32_t SwingCam_RED;
extern const uint8_t SwingCam_RED_addr;
extern int32_t DT50_PUT_RED;
extern const uint8_t DT50_PUT_RED_addr;
extern int32_t DT50_GET_RED;
extern const uint8_t DT50_GET_RED_addr;
#endif
