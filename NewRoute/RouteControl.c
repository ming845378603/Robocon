#include "RouteControl.h"
#include "Basal_Move.h"
#include "NewVect.h"
//#include "forward.h"
#include "Global.h"
//#include "MyMath.h"
#include "My_Math.h"
//#include "Route.h"
#include "PID_forward.h"
/*常规路径控制*************************************************************/
//自旋方向
enum _rot_dir
{
    ROTATE_ANTI_CLK = 1,
    ROTATE_CLK = -1,
};

/*自旋控制结构体*/
typedef struct
{
	forward_ctl_t fwd_ctl;		/*前馈控制*/
	float         start_ang;	/*起始姿态*/
	int8_t        dir;			/*旋转方向*/
}
rot_ctl_t;										   	

/*位置控制结构体：直线弧弯通用*/
typedef struct
{
	forward_ctl_t fwd_ctl;			/*前馈控制*/
    /*直线特有*/
	vect_t        line;			    /*直线路段的向量*/	
	/*弧线特有*/
	point_t       center;			/*圆弧弯的中心坐标*/
	int8_t        dir;				/*绕圆心旋转的方向*/
	int32_t       arc_r;			/*期望的转弯半径*/
	float	      start_vect_ang;	/*圆弧弯的起始半径向量角度*/
				
}
pos_ctl_t;
////////////////////////////////////////////////////////////////////////////


/*巡线信息************************************************************/
/*跷跷板的位置控制*/
typedef struct
{
    forward_ctl_t fwd_ctl;
    point_t center;
    int8_t dir;
    int32_t arc_r;
    float start_vect_ang;
    int32_t end_v;
}
SeesawCtlTypeDef;

/*巡线信息结构体*/
typedef struct
{
    int32_t  real_value[2];     /*传感器实际值*/
    int32_t  sensor[2];         /*传感器编号*/
    int32_t set_value[2];      /*传感器设定值*/
    vect_t   trace_vect;        /*巡线方向(减少误差的方向)*/
    vect_t   move_vect;         /*移动方向，一般与巡线方向垂直*/
    int32_t  speed;             /*设定的巡线速度*/
    SeesawCtlTypeDef SeesawCtl; /*跷跷板巡线控制*/
}trace_ctl_t;  
/////////////////////////////////////////////////////////////////////////

/*路径控制结构体*/
typedef struct 
{
    /*直线弧线路径*/
    uint8_t     pos_flg;		/*位置控制标志位*/
	pos_ctl_t   pos;			/*位置控制*/
    /*自旋路径*/
    uint8_t     rot_flg;		/*自旋标记位*/
	rot_ctl_t   rot;			/*旋转控制*/
	/*锁点路径*/
    uint8_t     lock_flg;       /*锁点标志位*/
    point_t     set_point;      /*设定的坐标点*/
    /*巡线路径*/
    uint8_t     trace_flg;      /*巡线标志位*/
    trace_ctl_t trace;          /*巡线控制*/
    
    /*PID参数*/
    pid_t pid_pos; /*锁定轨迹用的PID*/ 
    pid_t pid_ang; /*锁定角度用的PID*/
    
    /*路径时间参数*/
    uint32_t    start_time;     /*开始时间点*/
	uint32_t    end_time;		/*结束时间点*/
}
route_ctl_t;

/*当前正在进行的路径控制*/
static route_ctl_t RouteCtl;

						   	
/*直线控制相关*/
void update_line(route_inf_t * route_inf);
void control_line(void);
void lock_line_ctl(void);

/*弧弯相应的函数*/
void update_arc(route_inf_t * route_inf);
void control_arc(void);
void lock_arc_ctl(void);

/*自旋控制相应的函数*/
void update_rotate(route_inf_t * route_inf);
void control_rotate(void);

/*锁点相关函数*/
void lock_pos_ctl(void);

/*锁定角度*/
void lock_angle_ctl(void);

/*巡线相关*/
void update_trace(route_inf_t * route_inf);
void trace_route_ctl(void);

/*PID参数*/
extern double pidParam[][7];
/*前馈参数*/
extern int32_t forwardParam[][7];

/*设置路径*/
void SetupLine(int32_t x0, int32_t y0, float a0,
               int32_t x1, int32_t y1, float a1,
               int32_t * spd,
               uint8_t fwd1, uint8_t fwd2,
               uint8_t pid1, uint8_t pid2)
{
    route_inf_t RouteToSet;
     memset(&RouteToSet,0,sizeof(route_inf_t));
    /*设置路径类型*/
    RouteToSet.route_type = ROUTE_LINE;
    /*设置起点*/
    RouteToSet.start.x = x0;
    RouteToSet.start.y = y0;
    RouteToSet.start.ang = a0;
    /*设置终点*/
    RouteToSet.end.x = x1;
    RouteToSet.end.y = y1;
    RouteToSet.end.ang = a1;
    /*设置速度参数*/
    RouteToSet.spd_inf.unif_v = spd[0];
    RouteToSet.spd_inf.end_v = spd[1];
    RouteToSet.spd_inf.aclt = spd[2];
    RouteToSet.spd_inf.decr = spd[3];
    /*设置路径参数*/
    RouteToSet.forward_param[0] = fwd1;
    RouteToSet.forward_param[1] = fwd2;
    RouteToSet.pid_param[0] = pid1;
    RouteToSet.pid_param[1] = pid2;
    /*更新到路径控制中去*/
    UpdateRouteCtl(&RouteToSet);
}

void SetupLock(int32_t x0, int32_t y0, float a0,
               uint8_t pid1, uint8_t pid2)
{
    route_inf_t RouteToSet;
     memset(&RouteToSet,0,sizeof(route_inf_t));
    /*设置路径类型*/
    RouteToSet.route_type = ROUTE_LOCK;
    /*设置起点*/
    RouteToSet.start.x = x0;
    RouteToSet.start.y = y0;
    RouteToSet.start.ang = a0;
    /*设置终点*/
    RouteToSet.end.x = x0;
    RouteToSet.end.y = y0;
    RouteToSet.end.ang = a0;
    /*设置路径参数*/
    RouteToSet.pid_param[0] = pid1;
    RouteToSet.pid_param[1] = pid2;
    /*更新到路径控制中去*/
    UpdateRouteCtl(&RouteToSet);
}
void SetupSeesawTrace(int32_t x0, int32_t y0, float a0,
                      int32_t x1, int32_t y1, float a1,
                      int32_t cx, int32_t cy,
                      uint8_t sensor1, uint8_t sensor2,
                      int32_t set_value1, int32_t set_value2,
                      int32_t * spd,
                      uint8_t fwd1, uint8_t fwd2,
                      uint8_t pid1, uint8_t pid2)
{
    route_inf_t RouteToSet;
    memset(&RouteToSet,0,sizeof(route_inf_t));
    /*设置路径类型*/
    RouteToSet.route_type = ROUTE_TRACE;
    /*设置起点*/
    RouteToSet.start.x = x0;
    RouteToSet.start.y = y0;
    RouteToSet.start.ang = a0;
    /*设置终点*/
    RouteToSet.end.x = x1;
    RouteToSet.end.y = y1;
    RouteToSet.end.ang = a1;
    /*设置圆心*/
    RouteToSet.center.x = cx;
    RouteToSet.center.y = cy;
    /*设置速度参数*/
    RouteToSet.spd_inf.unif_v = spd[0];
    RouteToSet.spd_inf.end_v  = spd[1];
    RouteToSet.spd_inf.aclt   = spd[2];
    RouteToSet.spd_inf.decr   = spd[3];
    /*设置路径参数*/
    RouteToSet.forward_param[0] = fwd1;
    RouteToSet.forward_param[1] = fwd2;
    RouteToSet.pid_param[0] = pid1;
    RouteToSet.pid_param[1] = pid2;
    
    /*巡线参数*/
    RouteToSet.trace_inf.sensor[0] = sensor1;
    RouteToSet.trace_inf.sensor[1] = sensor2;
    RouteToSet.trace_inf.set_value[0] = set_value1;
    RouteToSet.trace_inf.set_value[1] = set_value2;
    
    if (GetGroundType() == GROUND_BLUE)
    {
        RouteToSet.trace_inf.trace_vect.e.x = 0;
        RouteToSet.trace_inf.trace_vect.e.y =1;
        RouteToSet.trace_inf.move_vect.e.x = 1;
        RouteToSet.trace_inf.move_vect.e.y = 0;
    }
    else if (GetGroundType() == GROUND_RED)
    {
        RouteToSet.trace_inf.trace_vect.e.x = 0;
        RouteToSet.trace_inf.trace_vect.e.y = 1;
        RouteToSet.trace_inf.move_vect.e.x = -1;
        RouteToSet.trace_inf.move_vect.e.y = 0;
    }
    
    /*更新到路径控制中去*/
    UpdateRouteCtl(&RouteToSet);
}

void SetupTrace(uint8_t sensor1, uint8_t sensor2,
              int32_t set_value1, int32_t set_value2,
              int32_t * spd,
              uint8_t pid1, uint8_t pid2)
{
    route_inf_t RouteToSet;
    memset(&RouteToSet,0,sizeof(route_inf_t));
    /*设置路径类型*/
    RouteToSet.route_type = ROUTE_TRACE;
    /*设置速度参数*/
    RouteToSet.spd_inf.unif_v = spd[0];
    RouteToSet.spd_inf.end_v = spd[1];
    RouteToSet.spd_inf.aclt = spd[2];
    RouteToSet.spd_inf.decr = spd[3];
    /*设置路径参数*/
    RouteToSet.pid_param[0] = pid1;
    RouteToSet.pid_param[1] = pid2;
    /*巡线参数*/
    RouteToSet.trace_inf.sensor[0] = sensor1;
    RouteToSet.trace_inf.sensor[1] = sensor2;
    RouteToSet.trace_inf.set_value[0] = set_value1;
    RouteToSet.trace_inf.set_value[1] = set_value2;
    
    /*分情况给予巡线方向和运动方向*/
    RouteToSet.trace_inf.trace_vect.s.x = 0;
    RouteToSet.trace_inf.trace_vect.s.y = 0;
    RouteToSet.trace_inf.move_vect.s.x = 0;
    RouteToSet.trace_inf.move_vect.s.y = 0;
    switch (sensor1)
    {
        case SWING_CAMERA:
            if (GetGroundType() == GROUND_BLUE)
            {
                RouteToSet.trace_inf.trace_vect.e.x = 0;
                RouteToSet.trace_inf.trace_vect.e.y = 1;
                RouteToSet.trace_inf.move_vect.e.x = 1;
                RouteToSet.trace_inf.move_vect.e.y = 0;
            }
            else if (GetGroundType() == GROUND_RED)
            {
                RouteToSet.trace_inf.trace_vect.e.x = 0;
                RouteToSet.trace_inf.trace_vect.e.y = -1;
                RouteToSet.trace_inf.move_vect.e.x = -1;
                RouteToSet.trace_inf.move_vect.e.y = 0;
            }
            break;
        case POLE_PUT_DT50:
            
            if (GetGroundType() == GROUND_BLUE)
            {
                RouteToSet.trace_inf.trace_vect.e.x = 0;
                RouteToSet.trace_inf.trace_vect.e.y = -1;
                RouteToSet.trace_inf.move_vect.e.x = -1;
                RouteToSet.trace_inf.move_vect.e.y = 0;
            }
            else if (GetGroundType() == GROUND_RED)
            {
               RouteToSet.trace_inf.trace_vect.e.x = 0;
                RouteToSet.trace_inf.trace_vect.e.y = -1;
                RouteToSet.trace_inf.move_vect.e.x = 1;
                RouteToSet.trace_inf.move_vect.e.y = 0;
            }
            break;
        case POLE_GET_DT50:
             if (GetGroundType() == GROUND_BLUE)
            {
                RouteToSet.trace_inf.trace_vect.e.x = 1;
                RouteToSet.trace_inf.trace_vect.e.y = 0;
                RouteToSet.trace_inf.move_vect.e.x = 0;
                RouteToSet.trace_inf.move_vect.e.y = -1;
            }
            else if (GetGroundType() == GROUND_RED)
            {
               RouteToSet.trace_inf.trace_vect.e.x = -1;
                RouteToSet.trace_inf.trace_vect.e.y = 0;
                RouteToSet.trace_inf.move_vect.e.x = 0;
                RouteToSet.trace_inf.move_vect.e.y = -1;
            }
            break;
    }
     /*更新到路径控制中去*/
    UpdateRouteCtl(&RouteToSet);
}
void SetupArc(int32_t x0, int32_t y0, float a0,
              int32_t x1, int32_t y1, float a1,
              int32_t cx, int32_t cy, int8_t type,
              int32_t * spd,
              uint8_t fwd1, uint8_t fwd2,
              uint8_t pid1, uint8_t pid2)
{
    route_inf_t RouteToSet;
    memset(&RouteToSet,0,sizeof(route_inf_t));
    /*设置路径类型*/
    RouteToSet.route_type = type;
    /*设置起点*/
    RouteToSet.start.x = x0;
    RouteToSet.start.y = y0;
    RouteToSet.start.ang = a0;
    /*设置终点*/
    RouteToSet.end.x = x1;
    RouteToSet.end.y = y1;
    RouteToSet.end.ang = a1;
    /*设置圆心*/
    RouteToSet.center.x = cx;
    RouteToSet.center.y = cy;
    /*设置速度参数*/
    RouteToSet.spd_inf.unif_v = spd[0];
    RouteToSet.spd_inf.end_v = spd[1];
    RouteToSet.spd_inf.aclt = spd[2];
    RouteToSet.spd_inf.decr = spd[3];
    /*设置路径参数*/
    RouteToSet.forward_param[0] = fwd1;
    RouteToSet.forward_param[1] = fwd2;
    RouteToSet.pid_param[0] = pid1;
    RouteToSet.pid_param[1] = pid2;
    /*更新到路径控制中去*/
    UpdateRouteCtl(&RouteToSet);
}


void UpdateRouteCtl(route_inf_t * NewRoute)
{
    uint8_t ForwardParam1 = NewRoute->forward_param[0];
    uint8_t ForwardParam2 = NewRoute->forward_param[1];
    uint8_t PIDParam1 = NewRoute->pid_param[0];
    uint8_t PIDParam2 = NewRoute->pid_param[1];
    
    USART_printf("*路径更新:s(%d,%d,%.1f),e(%d,%d,%.1f) spd:%d,%d,%d,%d\r\n",
                 NewRoute->start.x,NewRoute->start.y,NewRoute->start.ang,
                 NewRoute->end.x,NewRoute->end.y,NewRoute->end.ang,
                    NewRoute->spd_inf.unif_v, NewRoute->spd_inf.end_v,
                    NewRoute->spd_inf.aclt, NewRoute->spd_inf.decr);
    USART_printf("*当前坐标:(%d,%d,%.1f)\r\n",
                    G_Param.cur_pos.x,
                    G_Param.cur_pos.y,
                    G_Param.cur_pos.ang);
    
    memset(&RouteCtl, 0, sizeof(RouteCtl));
    
    /*更新位置控制*/
    switch (NewRoute->route_type)
    {
        case ROUTE_LINE: 
            RouteCtl.pos_flg = 1;
            update_forward_param(&RouteCtl.pos.fwd_ctl, forwardParam[ForwardParam1]);  /*更新前馈参数*/
            update_line(NewRoute);/*更新直线路径*/
            
            update_pid_param(&RouteCtl.pid_pos, 0, pidParam[PIDParam1]);/*更新PID参数即可*/
            RouteCtl.end_time = RouteCtl.pos.fwd_ctl.end_time; /*记录结束时间*/
            
            USART_printf("LINE:\r\n  st:%d at:%d ut:%d et:%d\r\n",
                          RouteCtl.pos.fwd_ctl.start_time,
                          RouteCtl.pos.fwd_ctl.aclt_time, 
                          RouteCtl.pos.fwd_ctl.unif_time, 
                          RouteCtl.pos.fwd_ctl.end_time);
            USART_printf("  sv:%.1f,uv:%.1f,ev:%.1f,acc:%.1f,dec:%.1f\r\n",
                          RouteCtl.pos.fwd_ctl.start_v,
                          RouteCtl.pos.fwd_ctl.unif_v,
                          RouteCtl.pos.fwd_ctl.end_v,
                          RouteCtl.pos.fwd_ctl.aclt,
                          RouteCtl.pos.fwd_ctl.decr);
            break;
        case ROUTE_CLK:
        case ROUTE_ANTI_CLK:
            RouteCtl.pos_flg = 2;
            update_forward_param(&RouteCtl.pos.fwd_ctl, forwardParam[ForwardParam1]);  /*更新前馈参数*/
            update_arc(NewRoute);/*更新弧弯路径*/
            
            update_pid_param(&RouteCtl.pid_pos, RouteCtl.pos.arc_r, pidParam[PIDParam1]);/*更新PID参数即可*/
            RouteCtl.end_time = RouteCtl.pos.fwd_ctl.end_time; /*记录结束时间*/
            
            USART_printf("CLK:\r\n  st:%d at:%d ut:%d et:%d\r\n",
                          RouteCtl.pos.fwd_ctl.start_time,
                          RouteCtl.pos.fwd_ctl.aclt_time, 
                          RouteCtl.pos.fwd_ctl.unif_time, 
                          RouteCtl.pos.fwd_ctl.end_time);
            USART_printf("  sv:%.1f,uv:%.1f,ev:%.1f,acc:%.1f,dec:%.1f\r\n",
                          RouteCtl.pos.fwd_ctl.start_v,
                          RouteCtl.pos.fwd_ctl.unif_v,
                          RouteCtl.pos.fwd_ctl.end_v,
                          RouteCtl.pos.fwd_ctl.aclt,
                          RouteCtl.pos.fwd_ctl.decr);
            break;
        case ROUTE_LOCK:
            RouteCtl.lock_flg =1;
            memcpy(&RouteCtl.set_point, &NewRoute->start, sizeof(point_t));
            update_pid_param(&RouteCtl.pid_pos, 0, pidParam[PIDParam1]);/*锁点直接更新PID参数即可*/
            
            USART_printf("LOCK:\r\n SetPoint(%d,%d,%.1f)\r\n", 
                          RouteCtl.set_point.x, 
                          RouteCtl.set_point.y,
                          RouteCtl.set_point.ang);
            break;
        case ROUTE_TRACE:
            RouteCtl.trace_flg = 1;
            update_forward_param(&RouteCtl.trace.SeesawCtl.fwd_ctl, forwardParam[ForwardParam1]);  /*更新前馈参数*/ 
            update_trace(NewRoute);/*更新巡线路径*/
            RouteCtl.end_time = RouteCtl.trace.SeesawCtl.fwd_ctl.end_time;/*记录结束时间*/
            
            USART_printf("TRACE set(%d,%d)\r\n", (int32_t)RouteCtl.trace.set_value[0], (int32_t)RouteCtl.trace.set_value[1]);
            break;
            
    }
    if (RouteCtl.trace_flg == 0)
    {
        RouteCtl.start_time = RouteCtl.pos.fwd_ctl.start_time; //获取路径开始的时间
    
        /*更新角度控制*/
        /*角度发生变化并且不是锁点路径的时候说明是自旋路径*/
        if (NewRoute->start.ang != NewRoute->end.ang && NewRoute->route_type != ROUTE_LOCK)
        {
            RouteCtl.rot_flg = 1;
            update_forward_param(&RouteCtl.rot.fwd_ctl, forwardParam[ForwardParam2]);  /*更新前馈参数*/
            /*更新自旋控制*/
            update_rotate(NewRoute);
        }
        else
        {
            RouteCtl.rot_flg = 0;
            /*锁定角度也是直接更新PID参数*/
            update_pid_param(&RouteCtl.pid_ang, (int32_t)(NewRoute->start.ang * 10), pidParam[PIDParam2]);
        }  
    }
    
}

void RouteControl(void)
{
    
    if (RouteCtl.pos_flg == 1)/*如果是直线路径*/
    {
         control_line();   //直线控制
         lock_line_ctl();  //锁定直线
    }
    else if (RouteCtl.pos_flg == 2)
    {
        control_arc();     //圆弧控制
        lock_arc_ctl();    //锁定圆弧
    }
    else if (RouteCtl.lock_flg == 1)
    {
        lock_pos_ctl();    //锁定坐标
    }
    else if (RouteCtl.trace_flg == 1)
    {
        trace_route_ctl(); //巡线控制
    }
    /*非巡线路径有角度控制*/
    if (RouteCtl.trace_flg != 1)
    {
        if (RouteCtl.rot_flg == 1)
        {
            control_rotate();//自旋控制
        }
        else if (RouteCtl.rot_flg == 0)
        {
            lock_angle_ctl();//角度锁定
        }
    }
}

uint32_t GetRouteEndTime(void)
{
    return RouteCtl.end_time;
}

/*直线控制************************************************************************************/

//更新直线路径
void update_line(route_inf_t * route_inf)
{
    vect_t * spd_vect = GetCurSpdVectPointer();         /*当前的速度向量*/
    point_t * p_pos   = GetCurPosPointer();             /*当前坐标的指针*/
    point_t ft_point;        							/*垂足坐标*/
    uint32_t cur_spd;                                   /*当前车速在期望直线上的分量*/ 
	float   spd_vect_ang;								/*当前速度方向*/
	float   cur_line_ang;								/*期望的直线向量角度*/
	float   param[6];									/*保存传递给前馈的参数*/

	spd_inf_t *p_spd   = &route_inf->spd_inf;		/*速度信息结构体*/
	point_t   *p_start = &route_inf->start;	        /*起点坐标*/
	point_t   *p_end   = &route_inf->end;			/*终点坐标*/

	set_vect(&RouteCtl.pos.line, p_start, p_end);			/*设定的直线向量*/
	get_foot_point(&RouteCtl.pos.line, p_pos, &ft_point);   /*当前坐标到目标直线的垂足*/
	set_vect(&RouteCtl.pos.line, &ft_point, p_end);		    /*实际的直线向量*/

    /*计算当前车速在期望直线上的分量*/
  spd_vect_ang = get_v2x_ang(spd_vect);
	cur_line_ang = get_v2x_ang(&RouteCtl.pos.line);
	cur_spd = GetCurSpd() * my_cos(spd_vect_ang - cur_line_ang);
    
	param[0] = get_vect_len(&RouteCtl.pos.line);	    /*总路程*/
	param[1] = cur_spd;				                    /*初速度*/
	param[2] = p_spd->unif_v;				            /*匀速速度*/
	param[3] = p_spd->end_v;	               			/*末速度*/
	param[4] = p_spd->aclt;					            /*加速度*/
	param[5] = p_spd->decr;					            /*减速度*/
    
    /*初始化前馈控制*/
//	update_forward_ctl(&RouteCtl.pos.fwd_ctl, GetCurTimeMS(), param);	
	update_forward_ctl(&RouteCtl.pos.fwd_ctl, GetSysTime_us() / 1000.0f);	
}


//直线控制
void control_line(void)
{
	point_t * p_pos = GetCurPosPointer();
	vect_t        *p_exp_vect = &RouteCtl.pos.line;         /*设定的直线向量*/
	forward_ctl_t *p_fwd      = &RouteCtl.pos.fwd_ctl;      /*前馈控制结构体指针*/
	int32_t        real_pos;							    /*当前的实际位置*/
	int32_t          ctl_v;								    /*前馈速度控制量*/
			
	real_pos  = my_abs(p2v_parallel_dist(p_exp_vect,p_pos)); 
    
	ctl_v     = forward_ctl(p_fwd, real_pos, GetCurTimeMS());
	
    
    
	if (ctl_v > p_fwd->max_out) ctl_v = p_fwd->max_out;
	else if (ctl_v < p_fwd->min_out) ctl_v = p_fwd->min_out;
	
	linear_speed(VECT(*p_exp_vect),p_pos->ang,ctl_v);      /*调用走直线函数计算轮子速度*/
	
	//USART_printf("ctl_line real:%d ctl_v:%d\r\n",real_pos, ctl_v);
}

/*锁定直线轨迹*/
void lock_line_ctl(void)
{
	point_t * p_pos = GetCurPosPointer();   /*当前坐标的指针*/
	point_t ft_point;			            /*当前位置与预期的直线的垂线交点*/
	vect_t  vertical;			         	/*垂线向量*/
	int32_t dist;				        	/*当前坐标与直线的垂直距离*/
	int32_t ctl_v;				        	/*控制速度输出*/
	
	get_foot_point(&RouteCtl.pos.line, p_pos, &ft_point);       /*获取垂足*/
	set_vect(&vertical, p_pos, &ft_point);			        	/*设置垂线向量*/
	dist = get_vect_len(&vertical);			                    /*位置偏差*/
	ctl_v = pid_calc(&RouteCtl.pid_pos, dist);                  /*控制量*/
	linear_speed(VECT(vertical),p_pos->ang,my_abs(ctl_v)); 	
    //USART_printf("lock_line dist:%d ctl_v:%d\r\n",dist, ctl_v);
}
/********************************************************************************************/


/*弧弯控制************************************************************************************/
/*更新弧线路径*/
void update_arc(route_inf_t * route_inf)
{
	point_t * p_pos = GetCurPosPointer();  /*当前位置坐标的向量*/
	vect_t  rad_vect;		/*半径向量*/
	float   end_vect_ang;	/*终点向量角度*/
	float   param[6];		/*保存传递给前馈的参数*/
	float   spd_vect_ang;	/*当前速度方向*/
	uint32_t cur_speed;     /*当前车速在期望弧线的切线方向上的分量*/
	pos_ctl_t * p_pos_ctl = &RouteCtl.pos;     /*位置控制结构体*/
	spd_inf_t * p_spd    = &route_inf->spd_inf;		/*速度控制信息结构体*/
	point_t   * p_center = &route_inf->center;		/*圆心坐标指针*/
	point_t   * p_end    = &route_inf->end;			/*终点坐标指针*/
	int8_t     dir      =  route_inf->route_type;	/*圆弧弯方向*/

	memcpy(&p_pos_ctl->center, p_center, sizeof(point_t));	/*保存圆心坐标*/
	p_pos_ctl->dir = dir;									/*保存圆弧弯旋转方向*/
	
	set_vect(&rad_vect, p_center, p_end);				/*设置半径向量*/
	p_pos_ctl->arc_r = get_vect_len(&rad_vect);				/*期望的半径*/
	end_vect_ang = get_v2x_ang (&rad_vect);	   			/*终点向量角度*/

	set_vect(&rad_vect, p_center, p_pos);			        /*起点半径向量*/
	p_pos_ctl->start_vect_ang = get_v2x_ang(&rad_vect);		/*起点向量角度*/

	if (dir == ROUTE_CLK)								/*顺时针圆弧弯的起点向量角度应大于终点向量角度*/
	{
		while (p_pos_ctl->start_vect_ang < end_vect_ang)
			end_vect_ang -= 360;				
	}
	else												/*逆时针圆弧弯的起点向量角度应小于终点向量角度*/
	{
		while (p_pos_ctl->start_vect_ang > end_vect_ang)
			end_vect_ang += 360;
	}
    /*计算车速在初始切线方向上的分量*/
    spd_vect_ang = get_v2x_ang(GetCurSpdVectPointer());
    cur_speed = GetCurSpd() * my_cos(spd_vect_ang - (p_pos_ctl->start_vect_ang + dir * 90));	
	/*总路程 角度*/
	param[0] = fabsf(p_pos_ctl->start_vect_ang - end_vect_ang);	
    /*圆弧弯的弧度不会超过270°·· 超过270°貌似也没啥意义=。=*/
	if (param[0] > 270) 										
	{
		param[0] -= 360;
		param[0]  = fabsf(param[0]);
	}
	param[0] = ang2rad(param[0]);						            /*转换成弧度*/
	param[1] = (float)cur_speed  /p_pos_ctl->arc_r;		          	/*初速度*/
	param[2] = (float)p_spd->unif_v / p_pos_ctl->arc_r;			    /*匀速速度*/
	param[3] = (float)p_spd->end_v  / p_pos_ctl->arc_r;		     	/*末速度*/
	param[4] = (float)p_spd->aclt   / p_pos_ctl->arc_r;  		    /*加速度*/
	param[5] = (float)p_spd->decr   / p_pos_ctl->arc_r;		  	    /*减速度*/

//    update_forward_ctl(&p_pos_ctl->fwd_ctl, GetCurTimeMS(), param);	/*初始化前馈控制*/
	    update_forward_ctl(&p_pos_ctl->fwd_ctl, GetCurTimeMS());	/*初始化前馈控制*/
}


void control_arc(void)
{
    point_t cur_pos;
    point_t * p_pos = &cur_pos;             /*当前坐标的指针*/
    vect_t   rad_vect;		                /*当前半径向量*/
	point_t *p_center_pos;	                /*圆心坐标*/
	int32_t	 cur_radius;	                /*当前的转弯半径*/
	int8_t   dir;			                /*圆弧弯的旋转方向*/
    
	float    cur_ang;		/*当前半径向量角度*/
	float    real_pos;		/*当前位置*/
	float    cur_cmdpos;	/*本周期的指令位置*/
	float    next_ang;		/*下一时刻的半径向量角度*/
	float    ctl_v;			/*速度控制量*/
	float    linear_v;		/*线速度*/

    pos_ctl_t * p_pos_ctl = &RouteCtl.pos;              /*位置控制结构体*/
    
    memcpy(&cur_pos, &G_Param.cur_pos, sizeof(point_t));
	dir          =  p_pos_ctl->dir;						/*获取旋转方向*/
	p_center_pos = &p_pos_ctl->center;					/*获取圆心坐标*/
	set_vect(&rad_vect, p_center_pos, p_pos);	        /*设置当前半径向量*/
	cur_radius = get_vect_len(&rad_vect);
	cur_ang    = get_v2x_ang (&rad_vect);			    /*当前半径长度和角度*/

	if (dir == ROUTE_CLK)							    /*顺时针圆弧弯的起点向量角度应大于终点向量角度*/
	{
		while (p_pos_ctl->start_vect_ang < cur_ang)
			cur_ang -= 360;				
	}
	else											    /*逆时针圆弧弯的起点向量角度应小于终点向量角度*/
	{
		while (p_pos_ctl->start_vect_ang > cur_ang)
			cur_ang += 360;
	}
	
	/*将半径向量的角度转换为位置量*/
	real_pos   = fabsf(p_pos_ctl->start_vect_ang - cur_ang);
   
	if (real_pos > 270)						            /*运动范围不可能超过270°*/
	{
		real_pos -= 360;
	}
    
	real_pos   = ang2rad(real_pos);
	ctl_v      = forward_ctl(&p_pos_ctl->fwd_ctl, real_pos, GetCurTimeMS());	/*调用前馈控制计算速度控制量*/
    linear_v   = ctl_v * cur_radius;
  
    /*限幅输出*/
	if (linear_v < p_pos_ctl->fwd_ctl.min_out)
		linear_v = p_pos_ctl->fwd_ctl.min_out;
	else if (linear_v > p_pos_ctl->fwd_ctl.max_out)
		linear_v = p_pos_ctl->fwd_ctl.max_out;	
        
    /*计算速度方向*/
	cur_cmdpos = rad2ang(dir * p_pos_ctl->fwd_ctl.cmd_pos[2]);		           /*本周期的指令位置*/
	next_ang   = p_pos_ctl->start_vect_ang + cur_cmdpos;	   				   /*计算实际的角度*/
//    USART_printf("ArcCtl:cmd2%.3f,cmd1%.3f %.3f ",
//                p_pos_ctl->fwd_ctl.cmd_pos[2],
//                p_pos_ctl->fwd_ctl.cmd_pos[1],
//                linear_v);
    {
        vect_t spd_vect;                /*速度方向向量*/
        point_t pos_next;               /*下一时刻的目标点*/
        pos_next.x = p_center_pos->x + cur_radius * my_cos(next_ang);
        pos_next.y = p_center_pos->y + cur_radius * my_sin(next_ang);
        set_vect(&spd_vect, p_pos, &pos_next);
        	
        linear_speed(VECT(spd_vect),p_pos->ang,linear_v);
//        USART_printf("当前:%d,%d 方向:%d,%d\r\n",
//                    p_pos->x, p_pos->y,
//                    pos_next.x, pos_next.y);
    }
    
    
}
/*锁定弧弯轨迹*/
void lock_arc_ctl(void)
{
    point_t * p_pos = GetCurPosPointer();       /*当前坐标的指针*/
    vect_t  rad_vect;		/*当前半径向量*/
	int32_t cur_radius;		/*当前半径*/
	int32_t ctl_v;			/*速度控制量*/
	pos_ctl_t * p_pos_ctl = &RouteCtl.pos;     /*位置控制结构体*/
    
    
	set_vect(&rad_vect, p_pos,&p_pos_ctl->center);	/*设置当前点指向圆心的向量*/
	cur_radius = get_vect_len(&rad_vect);		/*计算当前半径长度*/
    
    ctl_v = pid_calc(&RouteCtl.pid_pos, cur_radius);
    
	if (cur_radius < p_pos_ctl->arc_r)					/*往偏离圆心方向运动*/
	{
        linear_speed(VECT(rad_vect),p_pos->ang,-my_abs(ctl_v)); 	
	}
	else										/*往靠近圆心方向运动*/
	{
        linear_speed(VECT(rad_vect),p_pos->ang,my_abs(ctl_v)); 	
	}
    //USART_printf("lock_arc dist:%d ctl_v:%d\r\n",(int32_t)RouteCtl.pid_pos.err[1], ctl_v);
}
/********************************************************************************************/

/*旋转控制相关函数*///*******************************************************************************
/*更新旋转控制*/
void update_rotate(route_inf_t * route_inf)
{
    point_t   * p_pos   = GetCurPosPointer();
    rot_ctl_t * rot_ctl = &RouteCtl.rot;
	float end_ang;				   /*结束时的车体姿态*/
    uint32_t start_time = RouteCtl.start_time;
    uint32_t end_time   = RouteCtl.end_time;
	rot_ctl->start_ang  = p_pos->ang; //起始姿态
	end_ang             = route_inf->end.ang; /*起点和终点的车体姿态*/
    
	rot_ctl->dir = (rot_ctl->start_ang < end_ang)? ROTATE_ANTI_CLK: ROTATE_CLK;	/*旋转方向*/		
	rot_ctl->fwd_ctl.total_len  = my_abs(end_ang - rot_ctl->start_ang); 		/*总长度*/
	rot_ctl->fwd_ctl.start_time = start_time;		   		/*开始时间*/
	rot_ctl->fwd_ctl.end_time   = end_time;			   		/*结束时间*/
	rot_ctl->fwd_ctl.aclt_time  = (start_time + end_time) / 2;/*加速段的结束时间点*/
	rot_ctl->fwd_ctl.decr_time  = (end_time - start_time) / 2;/*减速段的时间长度*/
	rot_ctl->fwd_ctl.unif_time  = rot_ctl->fwd_ctl.aclt_time;				/*匀速段的结束时间点*/          
	rot_ctl->fwd_ctl.aclt       = 4 * rot_ctl->fwd_ctl.total_len / square((float)(end_time - start_time)/1000);
	rot_ctl->fwd_ctl.decr       = -rot_ctl->fwd_ctl.aclt;				   	/*加减速度*/
	rot_ctl->fwd_ctl.start_v   = 0;								   		/*初速度*/
	rot_ctl->fwd_ctl.end_v      = 0;								   		/*末速度*/
	rot_ctl->fwd_ctl.unif_v     = rot_ctl->fwd_ctl.decr_time * rot_ctl->fwd_ctl.aclt / 1000;
																		/*匀速速度*/
	rot_ctl->fwd_ctl.cmd_pos[0] = rot_ctl->fwd_ctl.aclt * square((float)FWD_CTL_PERIOD/1000) / 2;
	rot_ctl->fwd_ctl.cmd_pos[1] = 0;
	rot_ctl->fwd_ctl.cmd_pos[2] = 0;
    
}
/*旋转控制*/
void control_rotate(void)
{
    point_t * p_pos = GetCurPosPointer();
    float real_pos;				/*当前位置*/
	float ctl_v;				/*速度控制量*/
	rot_ctl_t * rot_ctl = &RouteCtl.rot;
    
	real_pos = fabsf(p_pos->ang - rot_ctl->start_ang);	//获取当前角度
	ctl_v    = forward_ctl(&rot_ctl->fwd_ctl, real_pos, GetCurTimeMS());

	if (ctl_v < rot_ctl->fwd_ctl.min_out)
		ctl_v = rot_ctl->fwd_ctl.min_out;
	else if (ctl_v > rot_ctl->fwd_ctl.max_out)
		ctl_v = rot_ctl->fwd_ctl.max_out;		   /*限幅输出*/

	ctl_v *= rot_ctl->dir;
	rotate_speed(ctl_v);
}
/***************************************************************************************************/



/*巡线控制*******************************************************************************************/

void UpdateSeesawTraceControl(route_inf_t * route_inf);
/*根据路径信息更新巡线信息*/
void update_trace(route_inf_t * route_inf)
{
    uint8_t locus_param = route_inf->pid_param[0];
    uint8_t ang_param   = route_inf->pid_param[1];
    trace_ctl_t * trace_ctl = &RouteCtl.trace;
    
    /*一般的巡线都用匀速速度作为巡线速度*/
    trace_ctl->speed = route_inf->spd_inf.unif_v;
    
    /*复制相关巡线的信息*/
    memcpy(&trace_ctl->move_vect,  &route_inf->trace_inf.move_vect, sizeof(vect_t));
    memcpy(&trace_ctl->trace_vect, &route_inf->trace_inf.trace_vect,sizeof(vect_t));
    //memcpy(&trace_ctl->sensor, &route_inf->trace_inf.sensor, sizeof(trace_ctl->sensor));
    //memcpy(&trace_ctl->set_value, &route_inf->trace_inf.set_value, sizeof(trace_ctl->set_value));
    trace_ctl->sensor[0] = route_inf->trace_inf.sensor[0];
    trace_ctl->sensor[1] = route_inf->trace_inf.sensor[1];
    trace_ctl->set_value[0] = route_inf->trace_inf.set_value[0];
    trace_ctl->set_value[1] = route_inf->trace_inf.set_value[1];
    /*PID参数*/
//    if (trace_ctl->sensor[0] == SEESAW_CAMERA_LEFT && trace_ctl->sensor[1] == SEESAW_CAMERA_RIGHT)
//    {
//        
//        update_pid_param(&RouteCtl.pid_pos, (trace_ctl->set_value[0] + (-trace_ctl->set_value[1])), pidParam[locus_param]);
//        update_pid_param(&RouteCtl.pid_ang, (trace_ctl->set_value[0] - (-trace_ctl->set_value[1])), pidParam[ang_param]);
//        
//        update_forward_param(&trace_ctl->SeesawCtl.fwd_ctl, forwardParam[route_inf->forward_param[0]]);/*更新前馈参数*/
//        UpdateSeesawTraceControl(route_inf);//更新跷跷板巡线的前馈控制
//        RouteCtl.trace.SeesawCtl.end_v = route_inf->spd_inf.end_v;
//        USART_printf("TRACE SEESAW :(%d,%d)\r\n",  
//                    (int32_t)RouteCtl.pid_pos.set_value, (int32_t)RouteCtl.pid_ang.set_value);
//    }
//    else
//    {
//        update_pid_param(&RouteCtl.pid_pos, trace_ctl->set_value[0], pidParam[locus_param]);
//        update_pid_param(&RouteCtl.pid_ang, (int32_t)(trace_ctl->set_value[1] * 10), pidParam[ang_param]); //角度放大10倍进行控制
//        
//        USART_printf("TRACE OTHER :(%d,%d)\r\n",  
//                    (int32_t)RouteCtl.pid_pos.set_value, (int32_t)RouteCtl.pid_ang.set_value);
//    }
    
    
}
//跷跷板巡线专用速度控制
//void UpdateSeesawTraceControl(route_inf_t * route_inf)
//{
//    point_t * p_pos = GetCurPosPointer();  /*当前位置坐标的向量*/
//    vect_t  rad_vect;		/*半径向量*/
//	float   end_vect_ang;	/*终点向量角度*/
//	float   param[6];		/*保存传递给前馈的参数*/
//    float   spd_vect_ang;	/*当前速度方向*/
//    uint32_t cur_speed;     /*当前车速在期望弧线的切线方向上的分量*/
//	SeesawCtlTypeDef * p_pos_ctl = &RouteCtl.trace.SeesawCtl; /*位置控制结构体*/
//	spd_inf_t * p_spd    = &route_inf->spd_inf;		/*速度控制信息结构体*/
//	point_t   * p_center = &route_inf->center;		/*圆心坐标指针*/
//	point_t   * p_end    = &route_inf->end;			/*终点坐标指针*/
//	int8_t     dir;	                                /*圆弧弯方向*/
//    
//    /*根据红蓝场确定旋转方向*/
//    if (GetGroundType() == GROUND_BLUE) dir = ROUTE_ANTI_CLK;
//    else dir = ROUTE_CLK;

//	memcpy(&p_pos_ctl->center, p_center, sizeof(point_t));	/*保存圆心坐标*/
//	p_pos_ctl->dir = dir;									/*保存圆弧弯旋转方向*/
//	
//	set_vect(&rad_vect, p_center, p_end);				/*设置半径向量*/
//	p_pos_ctl->arc_r = get_vect_len(&rad_vect);				/*期望的半径*/
//	end_vect_ang = get_v2x_ang (&rad_vect);	   			/*终点向量角度*/

//	set_vect(&rad_vect, p_center, p_pos);			/*起点半径向量*/
//	p_pos_ctl->start_vect_ang = get_v2x_ang(&rad_vect);		/*起点向量角度*/

//	if (dir == ROUTE_CLK)								/*顺时针圆弧弯的起点向量角度应大于终点向量角度*/
//	{
//		while (p_pos_ctl->start_vect_ang < end_vect_ang)
//			end_vect_ang -= 360;				
//	}
//	else												/*逆时针圆弧弯的起点向量角度应小于终点向量角度*/
//	{
//		while (p_pos_ctl->start_vect_ang > end_vect_ang)
//			end_vect_ang += 360;
//	}
//    /*计算车速在初始切线方向上的分量*/
//    spd_vect_ang = get_v2x_ang(GetCurSpdVectPointer());
//    cur_speed = GetCurSpd() * my_cos(spd_vect_ang - (p_pos_ctl->start_vect_ang + dir * 90));	
//	/*总路程 角度*/
//	param[0] = fabsf(p_pos_ctl->start_vect_ang - end_vect_ang);	
//    /*圆弧弯的弧度不会超过270°·· 超过270°貌似也没啥意义=。=*/
//	if (param[0] > 270) 										
//	{
//		param[0] -= 360;
//		param[0]  = fabsf(param[0]);
//	}
//	param[0] = ang2rad(param[0]);						            /*转换成弧度*/
//	param[1] = (float)cur_speed  /p_pos_ctl->arc_r;		          	/*初速度*/
//	param[2] = (float)p_spd->unif_v / p_pos_ctl->arc_r;			    /*匀速速度*/
//	param[3] = (float)p_spd->end_v  / p_pos_ctl->arc_r;		     	/*末速度*/
//	param[4] = (float)p_spd->aclt   / p_pos_ctl->arc_r;  		    /*加速度*/
//	param[5] = (float)p_spd->decr   / p_pos_ctl->arc_r;		  	    /*减速度*/

//    update_forward_ctl(&p_pos_ctl->fwd_ctl, GetCurTimeMS(), param);	/*初始化前馈控制*/
//}


//void SeesawCtl(void)
//{
//    static float last_linear_v = 0;
//    point_t * p_pos = GetCurPosPointer();   /*当前坐标的指针*/
//    vect_t   rad_vect;		                /*当前半径向量*/
//	point_t *p_center_pos;	                /*圆心坐标*/
//	int32_t	 cur_radius;	                /*当前的转弯半径*/
//	int8_t   dir;			                /*圆弧弯的旋转方向*/
//    
//	float    cur_ang;		/*当前半径向量角度*/
//	float    real_pos;		/*当前位置*/
//	float    ctl_v;			/*速度控制量*/
//	float    linear_v;		/*线速度*/
//    
//    SeesawCtlTypeDef * p_pos_ctl = &RouteCtl.trace.SeesawCtl;     /*位置控制结构体*/
//    
//	dir          =  p_pos_ctl->dir;						/*获取旋转方向*/
//	p_center_pos = &p_pos_ctl->center;					/*获取圆心坐标*/
//	set_vect(&rad_vect, p_center_pos, p_pos);	      /*设置当前半径向量*/
//	cur_radius = get_vect_len(&rad_vect);
//	cur_ang    = get_v2x_ang (&rad_vect);			/*当前半径长度和角度*/

//	if (dir == ROUTE_CLK)							/*顺时针圆弧弯的起点向量角度应大于终点向量角度*/
//	{
//		while (p_pos_ctl->start_vect_ang < cur_ang)
//			cur_ang -= 360;				
//	}
//	else											/*逆时针圆弧弯的起点向量角度应小于终点向量角度*/
//	{
//		while (p_pos_ctl->start_vect_ang > cur_ang)
//			cur_ang += 360;
//	}
//	
//	/*将半径向量的角度转换为位置量*/
//	real_pos   = fabsf(p_pos_ctl->start_vect_ang - cur_ang);
//   
//	if (real_pos > 270)						/*运动范围不可能超过270°*/
//	{
//		real_pos -= 360;
//	}
//    
//	real_pos   = ang2rad(real_pos);
//	ctl_v      = forward_ctl(&p_pos_ctl->fwd_ctl, real_pos, GetCurTimeMS());	  		/*调用前馈控制计算速度控制量*/
//    linear_v   = ctl_v * cur_radius;
//  
//    /*限幅输出*/
//	if (linear_v < p_pos_ctl->fwd_ctl.min_out)
//		linear_v = p_pos_ctl->fwd_ctl.min_out;
//	else if (linear_v > p_pos_ctl->fwd_ctl.max_out)
//		linear_v = p_pos_ctl->fwd_ctl.max_out;	
//    if (GetCurTimeMS() >= RouteCtl.end_time)  //超过时间之维持最后一次的输出
//    {
//        linear_speed(VECT(RouteCtl.trace.move_vect),0,(int32_t)last_linear_v); 
////        USART_printf("SeesawHold out:%d\r\n",
////                    (int32_t)last_linear_v);
//    }
//    else
//    {
//        linear_speed(VECT(RouteCtl.trace.move_vect),0,(int32_t)linear_v); 
//        last_linear_v = linear_v;
////        USART_printf("SeesawFwd:%.3f out:%d\r\n",
////                RouteCtl.trace.SeesawCtl.fwd_ctl.cmd_pos[2],
////                (int32_t)linear_v);
//    }
//    
//       
//}

///*巡线控制算法*/
//void trace_route_ctl(void)
//{
//    static uint8_t err_cnt = 0;
//    point_t * p_pos = GetCurPosPointer();
//    int32_t ctl_v = 0;
//    int32_t sensor1;
//    int32_t sensor2;
//    vect_t rad_vect;
//    trace_ctl_t * trace = &RouteCtl.trace;   //巡线控制结构体
//    
//    /*获得传感器编号*/
//    sensor1 = trace->sensor[0];
//    sensor2 = trace->sensor[1];
//    /*分传感器类型进行控制*/
//    if (sensor1 == SEESAW_CAMERA_LEFT && sensor2 == SEESAW_CAMERA_RIGHT)
//    {
////        USART_printf("摄像头:(%d,%d)\r\n",(int32_t) GetPosSensorValue(sensor1),(int32_t) GetPosSensorValue(sensor2) );
//        if (GetPosSensorValue(sensor1) <= 320 && GetPosSensorValue(sensor1) >= 0 && GetPosSensorValue(sensor2) <= 320 && GetPosSensorValue(sensor2) >= 0)
//        {
//            err_cnt = 0;
//            trace->real_value[0] = GetPosSensorValue(sensor1);
//            trace->real_value[1] = GetPosSensorValue(sensor2);
//        }
//        else
//        {
//            err_cnt++;
//            if (err_cnt > 50) /*保护代码*/
//            {
//                UnderpanBrake(); 
//                USART_printf("摄像头丢失\r\n");
//                while(1);
//            }
//            
//        }
//        /*锁定轨迹的PID运算*/
//        ctl_v = -pid_calc(&RouteCtl.pid_pos, trace->real_value[0] + (-trace->real_value[1])) 
//                - GetCurSpd() * SEESAW_FORWARD_POS_KF;
//        set_vect(&rad_vect, p_pos, &RouteCtl.trace.SeesawCtl.center); //获取指向圆心的向量
//        linear_speed(VECT(rad_vect),p_pos->ang,ctl_v);
//        /*锁定角度的PID运算*/
//        ctl_v = pid_calc(&RouteCtl.pid_ang, trace->real_value[0] - (-trace->real_value[1])) 
//                + GetCurSpd() * SEESAW_FORWARD_ANG_KF;
//        rotate_speed(ctl_v);

//        //速度控制
//        SeesawCtl();
//    }
//    else
//    {
//        if (sensor1 == SWING_CAMERA)
//        {
//            if  (GetPosSensorValue(sensor1) >=  0 && GetPosSensorValue(sensor1) <= 320)
//            {
//                err_cnt = 0;
//                trace->real_value[0] = GetPosSensorValue(sensor1);
//            }
//            else
//            {
//                 err_cnt++;
//                if (err_cnt > 50) /*保护代码*/
//                {
//                    UnderpanBrake(); 
//                    USART_printf("摄像头丢失\r\n");
//                    while(1);
//                }
//                
//            }
//        }
//        else //两个DT50的保护都是在设定值正负800以内保护的
//        {
//            if  (GetPosSensorValue(sensor1) >  trace->set_value[0] - 800 && GetPosSensorValue(sensor1) < trace->set_value[0] + 800)
//            {
//                err_cnt = 0;
//                trace->real_value[0] = (int32_t)GetPosSensorValue(sensor1);
//            }
//            else
//            {
//                 err_cnt++;
//                if (err_cnt > 50) /*保护代码*/
//                {
//                    UnderpanBrake(); 
//                    LCD12864_Clear();
//                    LCD12864_Printf(3,0,"DT50 ERR");
//                    USART_printf("DT50保护\r\n");
//                    while(1);
//                }
//                
//            }
//        }
//        
//        trace->real_value[1] = (int32_t)(GetPosSensorValue(sensor2) * 10);
//        
//        ctl_v = -pid_calc(&RouteCtl.pid_pos, trace->real_value[0]);
//        linear_speed(VECT(trace->trace_vect),0,ctl_v);
//        
////        USART_printf("r:%d,(%d,%d), out %d\r\n",
////                    (int32_t)trace->real_value[0],
////                    trace->trace_vect.e.x, trace->trace_vect.e.y, ctl_v);
//        
//        /*角度保护*/
//        if (my_abs(trace->real_value[1] - (int32_t)(trace->set_value[1] * 10))> 400) 
//        {
//            UnderpanBrake();
//            LCD12864_Clear();
//            LCD12864_Printf(0,0,"ANGLE!");
//            LCD12864_Printf(1,0,"rea:%.1f", (float)(trace->real_value[1]) / 10.0f);
//            LCD12864_Printf(2,0,"set:%.1f", (float)(trace->set_value[1]) / 10.0f);
//            USART_printf("角度保护：real:%.1f,set:%.1f",
//                        (float)(trace->real_value[1]) / 10.0f, 
//                        (float)(trace->set_value[1])  / 10.0f);
//            while(1);
//        }
//        ctl_v = pid_calc(&RouteCtl.pid_ang, trace->real_value[1]) ;
//        rotate_speed(ctl_v);
//        linear_speed(VECT(trace->move_vect),0,SPEED(trace->speed));
//    }
//}

/***************************************************************************************************/



/*角度控制*/
void lock_angle_ctl(void)
{
    point_t * p_pos = GetCurPosPointer();               /*当前坐标的指针*/
    int32_t ctl_v;	
    
    ctl_v = pid_calc(&RouteCtl.pid_ang,(int32_t)(p_pos->ang * 10));
    if (my_abs(RouteCtl.pid_ang.real_value[1] - RouteCtl.pid_ang.set_value) > 500)
    {
//        UnderpanBrake();
//        LCD12864_Clear();
//        LCD12864_Printf(0,0,"ANGLE!");
//        LCD12864_Printf(1,0,"rea:%.1f", (float)(RouteCtl.pid_ang.real_value[1]) / 10.0f);
//        LCD12864_Printf(2,0,"set:%.1f", (float)(RouteCtl.pid_ang.set_value) / 10.0f);
        while(1);
    }
    rotate_speed(ctl_v);
}
/*锁点控制*/
void lock_pos_ctl(void)
{
    point_t * p_pos = GetCurPosPointer();               /*当前坐标的指针*/
    vect_t target_vect; /*当前点指向目标点的向量*/
    int32_t dist = 0;
    int32_t ctl_v;
    
    set_vect(&target_vect, p_pos,&RouteCtl.set_point);
    dist = get_vect_len(&target_vect);
    if (dist > 500)
    {
//        UnderpanBrake();
//        LCD12864_Clear();
//        LCD12864_Printf(0,0,"LockPosError!");
//        LCD12864_Printf(1,0,"rea:(%d,%d)", p_pos->x, p_pos->y);
//        LCD12864_Printf(2,0,"set:(%d,%d)", RouteCtl.set_point.x,RouteCtl.set_point.y);
        while(1);
    }
    ctl_v = pid_calc(&RouteCtl.pid_pos, dist);
    linear_speed(VECT(target_vect),p_pos->ang,my_abs(ctl_v));
    //USART_printf("dist:%d,out:%d\r\n",dist,ctl_v);
}






