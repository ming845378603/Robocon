#include "RouteControl.h"
#include "Basal_Move.h"
#include "NewVect.h"
//#include "forward.h"
#include "Global.h"
//#include "MyMath.h"
#include "My_Math.h"
//#include "Route.h"
#include "PID_forward.h"
/*����·������*************************************************************/
//��������
enum _rot_dir
{
    ROTATE_ANTI_CLK = 1,
    ROTATE_CLK = -1,
};

/*�������ƽṹ��*/
typedef struct
{
	forward_ctl_t fwd_ctl;		/*ǰ������*/
	float         start_ang;	/*��ʼ��̬*/
	int8_t        dir;			/*��ת����*/
}
rot_ctl_t;										   	

/*λ�ÿ��ƽṹ�壺ֱ�߻���ͨ��*/
typedef struct
{
	forward_ctl_t fwd_ctl;			/*ǰ������*/
    /*ֱ������*/
	vect_t        line;			    /*ֱ��·�ε�����*/	
	/*��������*/
	point_t       center;			/*Բ�������������*/
	int8_t        dir;				/*��Բ����ת�ķ���*/
	int32_t       arc_r;			/*������ת��뾶*/
	float	      start_vect_ang;	/*Բ�������ʼ�뾶�����Ƕ�*/
				
}
pos_ctl_t;
////////////////////////////////////////////////////////////////////////////


/*Ѳ����Ϣ************************************************************/
/*���ΰ��λ�ÿ���*/
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

/*Ѳ����Ϣ�ṹ��*/
typedef struct
{
    int32_t  real_value[2];     /*������ʵ��ֵ*/
    int32_t  sensor[2];         /*���������*/
    int32_t set_value[2];      /*�������趨ֵ*/
    vect_t   trace_vect;        /*Ѳ�߷���(�������ķ���)*/
    vect_t   move_vect;         /*�ƶ�����һ����Ѳ�߷���ֱ*/
    int32_t  speed;             /*�趨��Ѳ���ٶ�*/
    SeesawCtlTypeDef SeesawCtl; /*���ΰ�Ѳ�߿���*/
}trace_ctl_t;  
/////////////////////////////////////////////////////////////////////////

/*·�����ƽṹ��*/
typedef struct 
{
    /*ֱ�߻���·��*/
    uint8_t     pos_flg;		/*λ�ÿ��Ʊ�־λ*/
	pos_ctl_t   pos;			/*λ�ÿ���*/
    /*����·��*/
    uint8_t     rot_flg;		/*�������λ*/
	rot_ctl_t   rot;			/*��ת����*/
	/*����·��*/
    uint8_t     lock_flg;       /*�����־λ*/
    point_t     set_point;      /*�趨�������*/
    /*Ѳ��·��*/
    uint8_t     trace_flg;      /*Ѳ�߱�־λ*/
    trace_ctl_t trace;          /*Ѳ�߿���*/
    
    /*PID����*/
    pid_t pid_pos; /*�����켣�õ�PID*/ 
    pid_t pid_ang; /*�����Ƕ��õ�PID*/
    
    /*·��ʱ�����*/
    uint32_t    start_time;     /*��ʼʱ���*/
	uint32_t    end_time;		/*����ʱ���*/
}
route_ctl_t;

/*��ǰ���ڽ��е�·������*/
static route_ctl_t RouteCtl;

						   	
/*ֱ�߿������*/
void update_line(route_inf_t * route_inf);
void control_line(void);
void lock_line_ctl(void);

/*������Ӧ�ĺ���*/
void update_arc(route_inf_t * route_inf);
void control_arc(void);
void lock_arc_ctl(void);

/*����������Ӧ�ĺ���*/
void update_rotate(route_inf_t * route_inf);
void control_rotate(void);

/*������غ���*/
void lock_pos_ctl(void);

/*�����Ƕ�*/
void lock_angle_ctl(void);

/*Ѳ�����*/
void update_trace(route_inf_t * route_inf);
void trace_route_ctl(void);

/*PID����*/
extern double pidParam[][7];
/*ǰ������*/
extern int32_t forwardParam[][7];

/*����·��*/
void SetupLine(int32_t x0, int32_t y0, float a0,
               int32_t x1, int32_t y1, float a1,
               int32_t * spd,
               uint8_t fwd1, uint8_t fwd2,
               uint8_t pid1, uint8_t pid2)
{
    route_inf_t RouteToSet;
     memset(&RouteToSet,0,sizeof(route_inf_t));
    /*����·������*/
    RouteToSet.route_type = ROUTE_LINE;
    /*�������*/
    RouteToSet.start.x = x0;
    RouteToSet.start.y = y0;
    RouteToSet.start.ang = a0;
    /*�����յ�*/
    RouteToSet.end.x = x1;
    RouteToSet.end.y = y1;
    RouteToSet.end.ang = a1;
    /*�����ٶȲ���*/
    RouteToSet.spd_inf.unif_v = spd[0];
    RouteToSet.spd_inf.end_v = spd[1];
    RouteToSet.spd_inf.aclt = spd[2];
    RouteToSet.spd_inf.decr = spd[3];
    /*����·������*/
    RouteToSet.forward_param[0] = fwd1;
    RouteToSet.forward_param[1] = fwd2;
    RouteToSet.pid_param[0] = pid1;
    RouteToSet.pid_param[1] = pid2;
    /*���µ�·��������ȥ*/
    UpdateRouteCtl(&RouteToSet);
}

void SetupLock(int32_t x0, int32_t y0, float a0,
               uint8_t pid1, uint8_t pid2)
{
    route_inf_t RouteToSet;
     memset(&RouteToSet,0,sizeof(route_inf_t));
    /*����·������*/
    RouteToSet.route_type = ROUTE_LOCK;
    /*�������*/
    RouteToSet.start.x = x0;
    RouteToSet.start.y = y0;
    RouteToSet.start.ang = a0;
    /*�����յ�*/
    RouteToSet.end.x = x0;
    RouteToSet.end.y = y0;
    RouteToSet.end.ang = a0;
    /*����·������*/
    RouteToSet.pid_param[0] = pid1;
    RouteToSet.pid_param[1] = pid2;
    /*���µ�·��������ȥ*/
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
    /*����·������*/
    RouteToSet.route_type = ROUTE_TRACE;
    /*�������*/
    RouteToSet.start.x = x0;
    RouteToSet.start.y = y0;
    RouteToSet.start.ang = a0;
    /*�����յ�*/
    RouteToSet.end.x = x1;
    RouteToSet.end.y = y1;
    RouteToSet.end.ang = a1;
    /*����Բ��*/
    RouteToSet.center.x = cx;
    RouteToSet.center.y = cy;
    /*�����ٶȲ���*/
    RouteToSet.spd_inf.unif_v = spd[0];
    RouteToSet.spd_inf.end_v  = spd[1];
    RouteToSet.spd_inf.aclt   = spd[2];
    RouteToSet.spd_inf.decr   = spd[3];
    /*����·������*/
    RouteToSet.forward_param[0] = fwd1;
    RouteToSet.forward_param[1] = fwd2;
    RouteToSet.pid_param[0] = pid1;
    RouteToSet.pid_param[1] = pid2;
    
    /*Ѳ�߲���*/
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
    
    /*���µ�·��������ȥ*/
    UpdateRouteCtl(&RouteToSet);
}

void SetupTrace(uint8_t sensor1, uint8_t sensor2,
              int32_t set_value1, int32_t set_value2,
              int32_t * spd,
              uint8_t pid1, uint8_t pid2)
{
    route_inf_t RouteToSet;
    memset(&RouteToSet,0,sizeof(route_inf_t));
    /*����·������*/
    RouteToSet.route_type = ROUTE_TRACE;
    /*�����ٶȲ���*/
    RouteToSet.spd_inf.unif_v = spd[0];
    RouteToSet.spd_inf.end_v = spd[1];
    RouteToSet.spd_inf.aclt = spd[2];
    RouteToSet.spd_inf.decr = spd[3];
    /*����·������*/
    RouteToSet.pid_param[0] = pid1;
    RouteToSet.pid_param[1] = pid2;
    /*Ѳ�߲���*/
    RouteToSet.trace_inf.sensor[0] = sensor1;
    RouteToSet.trace_inf.sensor[1] = sensor2;
    RouteToSet.trace_inf.set_value[0] = set_value1;
    RouteToSet.trace_inf.set_value[1] = set_value2;
    
    /*���������Ѳ�߷�����˶�����*/
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
     /*���µ�·��������ȥ*/
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
    /*����·������*/
    RouteToSet.route_type = type;
    /*�������*/
    RouteToSet.start.x = x0;
    RouteToSet.start.y = y0;
    RouteToSet.start.ang = a0;
    /*�����յ�*/
    RouteToSet.end.x = x1;
    RouteToSet.end.y = y1;
    RouteToSet.end.ang = a1;
    /*����Բ��*/
    RouteToSet.center.x = cx;
    RouteToSet.center.y = cy;
    /*�����ٶȲ���*/
    RouteToSet.spd_inf.unif_v = spd[0];
    RouteToSet.spd_inf.end_v = spd[1];
    RouteToSet.spd_inf.aclt = spd[2];
    RouteToSet.spd_inf.decr = spd[3];
    /*����·������*/
    RouteToSet.forward_param[0] = fwd1;
    RouteToSet.forward_param[1] = fwd2;
    RouteToSet.pid_param[0] = pid1;
    RouteToSet.pid_param[1] = pid2;
    /*���µ�·��������ȥ*/
    UpdateRouteCtl(&RouteToSet);
}


void UpdateRouteCtl(route_inf_t * NewRoute)
{
    uint8_t ForwardParam1 = NewRoute->forward_param[0];
    uint8_t ForwardParam2 = NewRoute->forward_param[1];
    uint8_t PIDParam1 = NewRoute->pid_param[0];
    uint8_t PIDParam2 = NewRoute->pid_param[1];
    
    USART_printf("*·������:s(%d,%d,%.1f),e(%d,%d,%.1f) spd:%d,%d,%d,%d\r\n",
                 NewRoute->start.x,NewRoute->start.y,NewRoute->start.ang,
                 NewRoute->end.x,NewRoute->end.y,NewRoute->end.ang,
                    NewRoute->spd_inf.unif_v, NewRoute->spd_inf.end_v,
                    NewRoute->spd_inf.aclt, NewRoute->spd_inf.decr);
    USART_printf("*��ǰ����:(%d,%d,%.1f)\r\n",
                    G_Param.cur_pos.x,
                    G_Param.cur_pos.y,
                    G_Param.cur_pos.ang);
    
    memset(&RouteCtl, 0, sizeof(RouteCtl));
    
    /*����λ�ÿ���*/
    switch (NewRoute->route_type)
    {
        case ROUTE_LINE: 
            RouteCtl.pos_flg = 1;
            update_forward_param(&RouteCtl.pos.fwd_ctl, forwardParam[ForwardParam1]);  /*����ǰ������*/
            update_line(NewRoute);/*����ֱ��·��*/
            
            update_pid_param(&RouteCtl.pid_pos, 0, pidParam[PIDParam1]);/*����PID��������*/
            RouteCtl.end_time = RouteCtl.pos.fwd_ctl.end_time; /*��¼����ʱ��*/
            
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
            update_forward_param(&RouteCtl.pos.fwd_ctl, forwardParam[ForwardParam1]);  /*����ǰ������*/
            update_arc(NewRoute);/*���»���·��*/
            
            update_pid_param(&RouteCtl.pid_pos, RouteCtl.pos.arc_r, pidParam[PIDParam1]);/*����PID��������*/
            RouteCtl.end_time = RouteCtl.pos.fwd_ctl.end_time; /*��¼����ʱ��*/
            
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
            update_pid_param(&RouteCtl.pid_pos, 0, pidParam[PIDParam1]);/*����ֱ�Ӹ���PID��������*/
            
            USART_printf("LOCK:\r\n SetPoint(%d,%d,%.1f)\r\n", 
                          RouteCtl.set_point.x, 
                          RouteCtl.set_point.y,
                          RouteCtl.set_point.ang);
            break;
        case ROUTE_TRACE:
            RouteCtl.trace_flg = 1;
            update_forward_param(&RouteCtl.trace.SeesawCtl.fwd_ctl, forwardParam[ForwardParam1]);  /*����ǰ������*/ 
            update_trace(NewRoute);/*����Ѳ��·��*/
            RouteCtl.end_time = RouteCtl.trace.SeesawCtl.fwd_ctl.end_time;/*��¼����ʱ��*/
            
            USART_printf("TRACE set(%d,%d)\r\n", (int32_t)RouteCtl.trace.set_value[0], (int32_t)RouteCtl.trace.set_value[1]);
            break;
            
    }
    if (RouteCtl.trace_flg == 0)
    {
        RouteCtl.start_time = RouteCtl.pos.fwd_ctl.start_time; //��ȡ·����ʼ��ʱ��
    
        /*���½Ƕȿ���*/
        /*�Ƕȷ����仯���Ҳ�������·����ʱ��˵��������·��*/
        if (NewRoute->start.ang != NewRoute->end.ang && NewRoute->route_type != ROUTE_LOCK)
        {
            RouteCtl.rot_flg = 1;
            update_forward_param(&RouteCtl.rot.fwd_ctl, forwardParam[ForwardParam2]);  /*����ǰ������*/
            /*������������*/
            update_rotate(NewRoute);
        }
        else
        {
            RouteCtl.rot_flg = 0;
            /*�����Ƕ�Ҳ��ֱ�Ӹ���PID����*/
            update_pid_param(&RouteCtl.pid_ang, (int32_t)(NewRoute->start.ang * 10), pidParam[PIDParam2]);
        }  
    }
    
}

void RouteControl(void)
{
    
    if (RouteCtl.pos_flg == 1)/*�����ֱ��·��*/
    {
         control_line();   //ֱ�߿���
         lock_line_ctl();  //����ֱ��
    }
    else if (RouteCtl.pos_flg == 2)
    {
        control_arc();     //Բ������
        lock_arc_ctl();    //����Բ��
    }
    else if (RouteCtl.lock_flg == 1)
    {
        lock_pos_ctl();    //��������
    }
    else if (RouteCtl.trace_flg == 1)
    {
        trace_route_ctl(); //Ѳ�߿���
    }
    /*��Ѳ��·���нǶȿ���*/
    if (RouteCtl.trace_flg != 1)
    {
        if (RouteCtl.rot_flg == 1)
        {
            control_rotate();//��������
        }
        else if (RouteCtl.rot_flg == 0)
        {
            lock_angle_ctl();//�Ƕ�����
        }
    }
}

uint32_t GetRouteEndTime(void)
{
    return RouteCtl.end_time;
}

/*ֱ�߿���************************************************************************************/

//����ֱ��·��
void update_line(route_inf_t * route_inf)
{
    vect_t * spd_vect = GetCurSpdVectPointer();         /*��ǰ���ٶ�����*/
    point_t * p_pos   = GetCurPosPointer();             /*��ǰ�����ָ��*/
    point_t ft_point;        							/*��������*/
    uint32_t cur_spd;                                   /*��ǰ����������ֱ���ϵķ���*/ 
	float   spd_vect_ang;								/*��ǰ�ٶȷ���*/
	float   cur_line_ang;								/*������ֱ�������Ƕ�*/
	float   param[6];									/*���洫�ݸ�ǰ���Ĳ���*/

	spd_inf_t *p_spd   = &route_inf->spd_inf;		/*�ٶ���Ϣ�ṹ��*/
	point_t   *p_start = &route_inf->start;	        /*�������*/
	point_t   *p_end   = &route_inf->end;			/*�յ�����*/

	set_vect(&RouteCtl.pos.line, p_start, p_end);			/*�趨��ֱ������*/
	get_foot_point(&RouteCtl.pos.line, p_pos, &ft_point);   /*��ǰ���굽Ŀ��ֱ�ߵĴ���*/
	set_vect(&RouteCtl.pos.line, &ft_point, p_end);		    /*ʵ�ʵ�ֱ������*/

    /*���㵱ǰ����������ֱ���ϵķ���*/
  spd_vect_ang = get_v2x_ang(spd_vect);
	cur_line_ang = get_v2x_ang(&RouteCtl.pos.line);
	cur_spd = GetCurSpd() * my_cos(spd_vect_ang - cur_line_ang);
    
	param[0] = get_vect_len(&RouteCtl.pos.line);	    /*��·��*/
	param[1] = cur_spd;				                    /*���ٶ�*/
	param[2] = p_spd->unif_v;				            /*�����ٶ�*/
	param[3] = p_spd->end_v;	               			/*ĩ�ٶ�*/
	param[4] = p_spd->aclt;					            /*���ٶ�*/
	param[5] = p_spd->decr;					            /*���ٶ�*/
    
    /*��ʼ��ǰ������*/
//	update_forward_ctl(&RouteCtl.pos.fwd_ctl, GetCurTimeMS(), param);	
	update_forward_ctl(&RouteCtl.pos.fwd_ctl, GetSysTime_us() / 1000.0f);	
}


//ֱ�߿���
void control_line(void)
{
	point_t * p_pos = GetCurPosPointer();
	vect_t        *p_exp_vect = &RouteCtl.pos.line;         /*�趨��ֱ������*/
	forward_ctl_t *p_fwd      = &RouteCtl.pos.fwd_ctl;      /*ǰ�����ƽṹ��ָ��*/
	int32_t        real_pos;							    /*��ǰ��ʵ��λ��*/
	int32_t          ctl_v;								    /*ǰ���ٶȿ�����*/
			
	real_pos  = my_abs(p2v_parallel_dist(p_exp_vect,p_pos)); 
    
	ctl_v     = forward_ctl(p_fwd, real_pos, GetCurTimeMS());
	
    
    
	if (ctl_v > p_fwd->max_out) ctl_v = p_fwd->max_out;
	else if (ctl_v < p_fwd->min_out) ctl_v = p_fwd->min_out;
	
	linear_speed(VECT(*p_exp_vect),p_pos->ang,ctl_v);      /*������ֱ�ߺ������������ٶ�*/
	
	//USART_printf("ctl_line real:%d ctl_v:%d\r\n",real_pos, ctl_v);
}

/*����ֱ�߹켣*/
void lock_line_ctl(void)
{
	point_t * p_pos = GetCurPosPointer();   /*��ǰ�����ָ��*/
	point_t ft_point;			            /*��ǰλ����Ԥ�ڵ�ֱ�ߵĴ��߽���*/
	vect_t  vertical;			         	/*��������*/
	int32_t dist;				        	/*��ǰ������ֱ�ߵĴ�ֱ����*/
	int32_t ctl_v;				        	/*�����ٶ����*/
	
	get_foot_point(&RouteCtl.pos.line, p_pos, &ft_point);       /*��ȡ����*/
	set_vect(&vertical, p_pos, &ft_point);			        	/*���ô�������*/
	dist = get_vect_len(&vertical);			                    /*λ��ƫ��*/
	ctl_v = pid_calc(&RouteCtl.pid_pos, dist);                  /*������*/
	linear_speed(VECT(vertical),p_pos->ang,my_abs(ctl_v)); 	
    //USART_printf("lock_line dist:%d ctl_v:%d\r\n",dist, ctl_v);
}
/********************************************************************************************/


/*�������************************************************************************************/
/*���»���·��*/
void update_arc(route_inf_t * route_inf)
{
	point_t * p_pos = GetCurPosPointer();  /*��ǰλ�����������*/
	vect_t  rad_vect;		/*�뾶����*/
	float   end_vect_ang;	/*�յ������Ƕ�*/
	float   param[6];		/*���洫�ݸ�ǰ���Ĳ���*/
	float   spd_vect_ang;	/*��ǰ�ٶȷ���*/
	uint32_t cur_speed;     /*��ǰ�������������ߵ����߷����ϵķ���*/
	pos_ctl_t * p_pos_ctl = &RouteCtl.pos;     /*λ�ÿ��ƽṹ��*/
	spd_inf_t * p_spd    = &route_inf->spd_inf;		/*�ٶȿ�����Ϣ�ṹ��*/
	point_t   * p_center = &route_inf->center;		/*Բ������ָ��*/
	point_t   * p_end    = &route_inf->end;			/*�յ�����ָ��*/
	int8_t     dir      =  route_inf->route_type;	/*Բ���䷽��*/

	memcpy(&p_pos_ctl->center, p_center, sizeof(point_t));	/*����Բ������*/
	p_pos_ctl->dir = dir;									/*����Բ������ת����*/
	
	set_vect(&rad_vect, p_center, p_end);				/*���ð뾶����*/
	p_pos_ctl->arc_r = get_vect_len(&rad_vect);				/*�����İ뾶*/
	end_vect_ang = get_v2x_ang (&rad_vect);	   			/*�յ������Ƕ�*/

	set_vect(&rad_vect, p_center, p_pos);			        /*���뾶����*/
	p_pos_ctl->start_vect_ang = get_v2x_ang(&rad_vect);		/*��������Ƕ�*/

	if (dir == ROUTE_CLK)								/*˳ʱ��Բ�������������Ƕ�Ӧ�����յ������Ƕ�*/
	{
		while (p_pos_ctl->start_vect_ang < end_vect_ang)
			end_vect_ang -= 360;				
	}
	else												/*��ʱ��Բ�������������Ƕ�ӦС���յ������Ƕ�*/
	{
		while (p_pos_ctl->start_vect_ang > end_vect_ang)
			end_vect_ang += 360;
	}
    /*���㳵���ڳ�ʼ���߷����ϵķ���*/
    spd_vect_ang = get_v2x_ang(GetCurSpdVectPointer());
    cur_speed = GetCurSpd() * my_cos(spd_vect_ang - (p_pos_ctl->start_vect_ang + dir * 90));	
	/*��·�� �Ƕ�*/
	param[0] = fabsf(p_pos_ctl->start_vect_ang - end_vect_ang);	
    /*Բ����Ļ��Ȳ��ᳬ��270�㡤�� ����270��ò��Ҳûɶ����=��=*/
	if (param[0] > 270) 										
	{
		param[0] -= 360;
		param[0]  = fabsf(param[0]);
	}
	param[0] = ang2rad(param[0]);						            /*ת���ɻ���*/
	param[1] = (float)cur_speed  /p_pos_ctl->arc_r;		          	/*���ٶ�*/
	param[2] = (float)p_spd->unif_v / p_pos_ctl->arc_r;			    /*�����ٶ�*/
	param[3] = (float)p_spd->end_v  / p_pos_ctl->arc_r;		     	/*ĩ�ٶ�*/
	param[4] = (float)p_spd->aclt   / p_pos_ctl->arc_r;  		    /*���ٶ�*/
	param[5] = (float)p_spd->decr   / p_pos_ctl->arc_r;		  	    /*���ٶ�*/

//    update_forward_ctl(&p_pos_ctl->fwd_ctl, GetCurTimeMS(), param);	/*��ʼ��ǰ������*/
	    update_forward_ctl(&p_pos_ctl->fwd_ctl, GetCurTimeMS());	/*��ʼ��ǰ������*/
}


void control_arc(void)
{
    point_t cur_pos;
    point_t * p_pos = &cur_pos;             /*��ǰ�����ָ��*/
    vect_t   rad_vect;		                /*��ǰ�뾶����*/
	point_t *p_center_pos;	                /*Բ������*/
	int32_t	 cur_radius;	                /*��ǰ��ת��뾶*/
	int8_t   dir;			                /*Բ�������ת����*/
    
	float    cur_ang;		/*��ǰ�뾶�����Ƕ�*/
	float    real_pos;		/*��ǰλ��*/
	float    cur_cmdpos;	/*�����ڵ�ָ��λ��*/
	float    next_ang;		/*��һʱ�̵İ뾶�����Ƕ�*/
	float    ctl_v;			/*�ٶȿ�����*/
	float    linear_v;		/*���ٶ�*/

    pos_ctl_t * p_pos_ctl = &RouteCtl.pos;              /*λ�ÿ��ƽṹ��*/
    
    memcpy(&cur_pos, &G_Param.cur_pos, sizeof(point_t));
	dir          =  p_pos_ctl->dir;						/*��ȡ��ת����*/
	p_center_pos = &p_pos_ctl->center;					/*��ȡԲ������*/
	set_vect(&rad_vect, p_center_pos, p_pos);	        /*���õ�ǰ�뾶����*/
	cur_radius = get_vect_len(&rad_vect);
	cur_ang    = get_v2x_ang (&rad_vect);			    /*��ǰ�뾶���ȺͽǶ�*/

	if (dir == ROUTE_CLK)							    /*˳ʱ��Բ�������������Ƕ�Ӧ�����յ������Ƕ�*/
	{
		while (p_pos_ctl->start_vect_ang < cur_ang)
			cur_ang -= 360;				
	}
	else											    /*��ʱ��Բ�������������Ƕ�ӦС���յ������Ƕ�*/
	{
		while (p_pos_ctl->start_vect_ang > cur_ang)
			cur_ang += 360;
	}
	
	/*���뾶�����ĽǶ�ת��Ϊλ����*/
	real_pos   = fabsf(p_pos_ctl->start_vect_ang - cur_ang);
   
	if (real_pos > 270)						            /*�˶���Χ�����ܳ���270��*/
	{
		real_pos -= 360;
	}
    
	real_pos   = ang2rad(real_pos);
	ctl_v      = forward_ctl(&p_pos_ctl->fwd_ctl, real_pos, GetCurTimeMS());	/*����ǰ�����Ƽ����ٶȿ�����*/
    linear_v   = ctl_v * cur_radius;
  
    /*�޷����*/
	if (linear_v < p_pos_ctl->fwd_ctl.min_out)
		linear_v = p_pos_ctl->fwd_ctl.min_out;
	else if (linear_v > p_pos_ctl->fwd_ctl.max_out)
		linear_v = p_pos_ctl->fwd_ctl.max_out;	
        
    /*�����ٶȷ���*/
	cur_cmdpos = rad2ang(dir * p_pos_ctl->fwd_ctl.cmd_pos[2]);		           /*�����ڵ�ָ��λ��*/
	next_ang   = p_pos_ctl->start_vect_ang + cur_cmdpos;	   				   /*����ʵ�ʵĽǶ�*/
//    USART_printf("ArcCtl:cmd2%.3f,cmd1%.3f %.3f ",
//                p_pos_ctl->fwd_ctl.cmd_pos[2],
//                p_pos_ctl->fwd_ctl.cmd_pos[1],
//                linear_v);
    {
        vect_t spd_vect;                /*�ٶȷ�������*/
        point_t pos_next;               /*��һʱ�̵�Ŀ���*/
        pos_next.x = p_center_pos->x + cur_radius * my_cos(next_ang);
        pos_next.y = p_center_pos->y + cur_radius * my_sin(next_ang);
        set_vect(&spd_vect, p_pos, &pos_next);
        	
        linear_speed(VECT(spd_vect),p_pos->ang,linear_v);
//        USART_printf("��ǰ:%d,%d ����:%d,%d\r\n",
//                    p_pos->x, p_pos->y,
//                    pos_next.x, pos_next.y);
    }
    
    
}
/*��������켣*/
void lock_arc_ctl(void)
{
    point_t * p_pos = GetCurPosPointer();       /*��ǰ�����ָ��*/
    vect_t  rad_vect;		/*��ǰ�뾶����*/
	int32_t cur_radius;		/*��ǰ�뾶*/
	int32_t ctl_v;			/*�ٶȿ�����*/
	pos_ctl_t * p_pos_ctl = &RouteCtl.pos;     /*λ�ÿ��ƽṹ��*/
    
    
	set_vect(&rad_vect, p_pos,&p_pos_ctl->center);	/*���õ�ǰ��ָ��Բ�ĵ�����*/
	cur_radius = get_vect_len(&rad_vect);		/*���㵱ǰ�뾶����*/
    
    ctl_v = pid_calc(&RouteCtl.pid_pos, cur_radius);
    
	if (cur_radius < p_pos_ctl->arc_r)					/*��ƫ��Բ�ķ����˶�*/
	{
        linear_speed(VECT(rad_vect),p_pos->ang,-my_abs(ctl_v)); 	
	}
	else										/*������Բ�ķ����˶�*/
	{
        linear_speed(VECT(rad_vect),p_pos->ang,my_abs(ctl_v)); 	
	}
    //USART_printf("lock_arc dist:%d ctl_v:%d\r\n",(int32_t)RouteCtl.pid_pos.err[1], ctl_v);
}
/********************************************************************************************/

/*��ת������غ���*///*******************************************************************************
/*������ת����*/
void update_rotate(route_inf_t * route_inf)
{
    point_t   * p_pos   = GetCurPosPointer();
    rot_ctl_t * rot_ctl = &RouteCtl.rot;
	float end_ang;				   /*����ʱ�ĳ�����̬*/
    uint32_t start_time = RouteCtl.start_time;
    uint32_t end_time   = RouteCtl.end_time;
	rot_ctl->start_ang  = p_pos->ang; //��ʼ��̬
	end_ang             = route_inf->end.ang; /*�����յ�ĳ�����̬*/
    
	rot_ctl->dir = (rot_ctl->start_ang < end_ang)? ROTATE_ANTI_CLK: ROTATE_CLK;	/*��ת����*/		
	rot_ctl->fwd_ctl.total_len  = my_abs(end_ang - rot_ctl->start_ang); 		/*�ܳ���*/
	rot_ctl->fwd_ctl.start_time = start_time;		   		/*��ʼʱ��*/
	rot_ctl->fwd_ctl.end_time   = end_time;			   		/*����ʱ��*/
	rot_ctl->fwd_ctl.aclt_time  = (start_time + end_time) / 2;/*���ٶεĽ���ʱ���*/
	rot_ctl->fwd_ctl.decr_time  = (end_time - start_time) / 2;/*���ٶε�ʱ�䳤��*/
	rot_ctl->fwd_ctl.unif_time  = rot_ctl->fwd_ctl.aclt_time;				/*���ٶεĽ���ʱ���*/          
	rot_ctl->fwd_ctl.aclt       = 4 * rot_ctl->fwd_ctl.total_len / square((float)(end_time - start_time)/1000);
	rot_ctl->fwd_ctl.decr       = -rot_ctl->fwd_ctl.aclt;				   	/*�Ӽ��ٶ�*/
	rot_ctl->fwd_ctl.start_v   = 0;								   		/*���ٶ�*/
	rot_ctl->fwd_ctl.end_v      = 0;								   		/*ĩ�ٶ�*/
	rot_ctl->fwd_ctl.unif_v     = rot_ctl->fwd_ctl.decr_time * rot_ctl->fwd_ctl.aclt / 1000;
																		/*�����ٶ�*/
	rot_ctl->fwd_ctl.cmd_pos[0] = rot_ctl->fwd_ctl.aclt * square((float)FWD_CTL_PERIOD/1000) / 2;
	rot_ctl->fwd_ctl.cmd_pos[1] = 0;
	rot_ctl->fwd_ctl.cmd_pos[2] = 0;
    
}
/*��ת����*/
void control_rotate(void)
{
    point_t * p_pos = GetCurPosPointer();
    float real_pos;				/*��ǰλ��*/
	float ctl_v;				/*�ٶȿ�����*/
	rot_ctl_t * rot_ctl = &RouteCtl.rot;
    
	real_pos = fabsf(p_pos->ang - rot_ctl->start_ang);	//��ȡ��ǰ�Ƕ�
	ctl_v    = forward_ctl(&rot_ctl->fwd_ctl, real_pos, GetCurTimeMS());

	if (ctl_v < rot_ctl->fwd_ctl.min_out)
		ctl_v = rot_ctl->fwd_ctl.min_out;
	else if (ctl_v > rot_ctl->fwd_ctl.max_out)
		ctl_v = rot_ctl->fwd_ctl.max_out;		   /*�޷����*/

	ctl_v *= rot_ctl->dir;
	rotate_speed(ctl_v);
}
/***************************************************************************************************/



/*Ѳ�߿���*******************************************************************************************/

void UpdateSeesawTraceControl(route_inf_t * route_inf);
/*����·����Ϣ����Ѳ����Ϣ*/
void update_trace(route_inf_t * route_inf)
{
    uint8_t locus_param = route_inf->pid_param[0];
    uint8_t ang_param   = route_inf->pid_param[1];
    trace_ctl_t * trace_ctl = &RouteCtl.trace;
    
    /*һ���Ѳ�߶��������ٶ���ΪѲ���ٶ�*/
    trace_ctl->speed = route_inf->spd_inf.unif_v;
    
    /*�������Ѳ�ߵ���Ϣ*/
    memcpy(&trace_ctl->move_vect,  &route_inf->trace_inf.move_vect, sizeof(vect_t));
    memcpy(&trace_ctl->trace_vect, &route_inf->trace_inf.trace_vect,sizeof(vect_t));
    //memcpy(&trace_ctl->sensor, &route_inf->trace_inf.sensor, sizeof(trace_ctl->sensor));
    //memcpy(&trace_ctl->set_value, &route_inf->trace_inf.set_value, sizeof(trace_ctl->set_value));
    trace_ctl->sensor[0] = route_inf->trace_inf.sensor[0];
    trace_ctl->sensor[1] = route_inf->trace_inf.sensor[1];
    trace_ctl->set_value[0] = route_inf->trace_inf.set_value[0];
    trace_ctl->set_value[1] = route_inf->trace_inf.set_value[1];
    /*PID����*/
//    if (trace_ctl->sensor[0] == SEESAW_CAMERA_LEFT && trace_ctl->sensor[1] == SEESAW_CAMERA_RIGHT)
//    {
//        
//        update_pid_param(&RouteCtl.pid_pos, (trace_ctl->set_value[0] + (-trace_ctl->set_value[1])), pidParam[locus_param]);
//        update_pid_param(&RouteCtl.pid_ang, (trace_ctl->set_value[0] - (-trace_ctl->set_value[1])), pidParam[ang_param]);
//        
//        update_forward_param(&trace_ctl->SeesawCtl.fwd_ctl, forwardParam[route_inf->forward_param[0]]);/*����ǰ������*/
//        UpdateSeesawTraceControl(route_inf);//�������ΰ�Ѳ�ߵ�ǰ������
//        RouteCtl.trace.SeesawCtl.end_v = route_inf->spd_inf.end_v;
//        USART_printf("TRACE SEESAW :(%d,%d)\r\n",  
//                    (int32_t)RouteCtl.pid_pos.set_value, (int32_t)RouteCtl.pid_ang.set_value);
//    }
//    else
//    {
//        update_pid_param(&RouteCtl.pid_pos, trace_ctl->set_value[0], pidParam[locus_param]);
//        update_pid_param(&RouteCtl.pid_ang, (int32_t)(trace_ctl->set_value[1] * 10), pidParam[ang_param]); //�ǶȷŴ�10�����п���
//        
//        USART_printf("TRACE OTHER :(%d,%d)\r\n",  
//                    (int32_t)RouteCtl.pid_pos.set_value, (int32_t)RouteCtl.pid_ang.set_value);
//    }
    
    
}
//���ΰ�Ѳ��ר���ٶȿ���
//void UpdateSeesawTraceControl(route_inf_t * route_inf)
//{
//    point_t * p_pos = GetCurPosPointer();  /*��ǰλ�����������*/
//    vect_t  rad_vect;		/*�뾶����*/
//	float   end_vect_ang;	/*�յ������Ƕ�*/
//	float   param[6];		/*���洫�ݸ�ǰ���Ĳ���*/
//    float   spd_vect_ang;	/*��ǰ�ٶȷ���*/
//    uint32_t cur_speed;     /*��ǰ�������������ߵ����߷����ϵķ���*/
//	SeesawCtlTypeDef * p_pos_ctl = &RouteCtl.trace.SeesawCtl; /*λ�ÿ��ƽṹ��*/
//	spd_inf_t * p_spd    = &route_inf->spd_inf;		/*�ٶȿ�����Ϣ�ṹ��*/
//	point_t   * p_center = &route_inf->center;		/*Բ������ָ��*/
//	point_t   * p_end    = &route_inf->end;			/*�յ�����ָ��*/
//	int8_t     dir;	                                /*Բ���䷽��*/
//    
//    /*���ݺ�����ȷ����ת����*/
//    if (GetGroundType() == GROUND_BLUE) dir = ROUTE_ANTI_CLK;
//    else dir = ROUTE_CLK;

//	memcpy(&p_pos_ctl->center, p_center, sizeof(point_t));	/*����Բ������*/
//	p_pos_ctl->dir = dir;									/*����Բ������ת����*/
//	
//	set_vect(&rad_vect, p_center, p_end);				/*���ð뾶����*/
//	p_pos_ctl->arc_r = get_vect_len(&rad_vect);				/*�����İ뾶*/
//	end_vect_ang = get_v2x_ang (&rad_vect);	   			/*�յ������Ƕ�*/

//	set_vect(&rad_vect, p_center, p_pos);			/*���뾶����*/
//	p_pos_ctl->start_vect_ang = get_v2x_ang(&rad_vect);		/*��������Ƕ�*/

//	if (dir == ROUTE_CLK)								/*˳ʱ��Բ�������������Ƕ�Ӧ�����յ������Ƕ�*/
//	{
//		while (p_pos_ctl->start_vect_ang < end_vect_ang)
//			end_vect_ang -= 360;				
//	}
//	else												/*��ʱ��Բ�������������Ƕ�ӦС���յ������Ƕ�*/
//	{
//		while (p_pos_ctl->start_vect_ang > end_vect_ang)
//			end_vect_ang += 360;
//	}
//    /*���㳵���ڳ�ʼ���߷����ϵķ���*/
//    spd_vect_ang = get_v2x_ang(GetCurSpdVectPointer());
//    cur_speed = GetCurSpd() * my_cos(spd_vect_ang - (p_pos_ctl->start_vect_ang + dir * 90));	
//	/*��·�� �Ƕ�*/
//	param[0] = fabsf(p_pos_ctl->start_vect_ang - end_vect_ang);	
//    /*Բ����Ļ��Ȳ��ᳬ��270�㡤�� ����270��ò��Ҳûɶ����=��=*/
//	if (param[0] > 270) 										
//	{
//		param[0] -= 360;
//		param[0]  = fabsf(param[0]);
//	}
//	param[0] = ang2rad(param[0]);						            /*ת���ɻ���*/
//	param[1] = (float)cur_speed  /p_pos_ctl->arc_r;		          	/*���ٶ�*/
//	param[2] = (float)p_spd->unif_v / p_pos_ctl->arc_r;			    /*�����ٶ�*/
//	param[3] = (float)p_spd->end_v  / p_pos_ctl->arc_r;		     	/*ĩ�ٶ�*/
//	param[4] = (float)p_spd->aclt   / p_pos_ctl->arc_r;  		    /*���ٶ�*/
//	param[5] = (float)p_spd->decr   / p_pos_ctl->arc_r;		  	    /*���ٶ�*/

//    update_forward_ctl(&p_pos_ctl->fwd_ctl, GetCurTimeMS(), param);	/*��ʼ��ǰ������*/
//}


//void SeesawCtl(void)
//{
//    static float last_linear_v = 0;
//    point_t * p_pos = GetCurPosPointer();   /*��ǰ�����ָ��*/
//    vect_t   rad_vect;		                /*��ǰ�뾶����*/
//	point_t *p_center_pos;	                /*Բ������*/
//	int32_t	 cur_radius;	                /*��ǰ��ת��뾶*/
//	int8_t   dir;			                /*Բ�������ת����*/
//    
//	float    cur_ang;		/*��ǰ�뾶�����Ƕ�*/
//	float    real_pos;		/*��ǰλ��*/
//	float    ctl_v;			/*�ٶȿ�����*/
//	float    linear_v;		/*���ٶ�*/
//    
//    SeesawCtlTypeDef * p_pos_ctl = &RouteCtl.trace.SeesawCtl;     /*λ�ÿ��ƽṹ��*/
//    
//	dir          =  p_pos_ctl->dir;						/*��ȡ��ת����*/
//	p_center_pos = &p_pos_ctl->center;					/*��ȡԲ������*/
//	set_vect(&rad_vect, p_center_pos, p_pos);	      /*���õ�ǰ�뾶����*/
//	cur_radius = get_vect_len(&rad_vect);
//	cur_ang    = get_v2x_ang (&rad_vect);			/*��ǰ�뾶���ȺͽǶ�*/

//	if (dir == ROUTE_CLK)							/*˳ʱ��Բ�������������Ƕ�Ӧ�����յ������Ƕ�*/
//	{
//		while (p_pos_ctl->start_vect_ang < cur_ang)
//			cur_ang -= 360;				
//	}
//	else											/*��ʱ��Բ�������������Ƕ�ӦС���յ������Ƕ�*/
//	{
//		while (p_pos_ctl->start_vect_ang > cur_ang)
//			cur_ang += 360;
//	}
//	
//	/*���뾶�����ĽǶ�ת��Ϊλ����*/
//	real_pos   = fabsf(p_pos_ctl->start_vect_ang - cur_ang);
//   
//	if (real_pos > 270)						/*�˶���Χ�����ܳ���270��*/
//	{
//		real_pos -= 360;
//	}
//    
//	real_pos   = ang2rad(real_pos);
//	ctl_v      = forward_ctl(&p_pos_ctl->fwd_ctl, real_pos, GetCurTimeMS());	  		/*����ǰ�����Ƽ����ٶȿ�����*/
//    linear_v   = ctl_v * cur_radius;
//  
//    /*�޷����*/
//	if (linear_v < p_pos_ctl->fwd_ctl.min_out)
//		linear_v = p_pos_ctl->fwd_ctl.min_out;
//	else if (linear_v > p_pos_ctl->fwd_ctl.max_out)
//		linear_v = p_pos_ctl->fwd_ctl.max_out;	
//    if (GetCurTimeMS() >= RouteCtl.end_time)  //����ʱ��֮ά�����һ�ε����
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

///*Ѳ�߿����㷨*/
//void trace_route_ctl(void)
//{
//    static uint8_t err_cnt = 0;
//    point_t * p_pos = GetCurPosPointer();
//    int32_t ctl_v = 0;
//    int32_t sensor1;
//    int32_t sensor2;
//    vect_t rad_vect;
//    trace_ctl_t * trace = &RouteCtl.trace;   //Ѳ�߿��ƽṹ��
//    
//    /*��ô��������*/
//    sensor1 = trace->sensor[0];
//    sensor2 = trace->sensor[1];
//    /*�ִ��������ͽ��п���*/
//    if (sensor1 == SEESAW_CAMERA_LEFT && sensor2 == SEESAW_CAMERA_RIGHT)
//    {
////        USART_printf("����ͷ:(%d,%d)\r\n",(int32_t) GetPosSensorValue(sensor1),(int32_t) GetPosSensorValue(sensor2) );
//        if (GetPosSensorValue(sensor1) <= 320 && GetPosSensorValue(sensor1) >= 0 && GetPosSensorValue(sensor2) <= 320 && GetPosSensorValue(sensor2) >= 0)
//        {
//            err_cnt = 0;
//            trace->real_value[0] = GetPosSensorValue(sensor1);
//            trace->real_value[1] = GetPosSensorValue(sensor2);
//        }
//        else
//        {
//            err_cnt++;
//            if (err_cnt > 50) /*��������*/
//            {
//                UnderpanBrake(); 
//                USART_printf("����ͷ��ʧ\r\n");
//                while(1);
//            }
//            
//        }
//        /*�����켣��PID����*/
//        ctl_v = -pid_calc(&RouteCtl.pid_pos, trace->real_value[0] + (-trace->real_value[1])) 
//                - GetCurSpd() * SEESAW_FORWARD_POS_KF;
//        set_vect(&rad_vect, p_pos, &RouteCtl.trace.SeesawCtl.center); //��ȡָ��Բ�ĵ�����
//        linear_speed(VECT(rad_vect),p_pos->ang,ctl_v);
//        /*�����Ƕȵ�PID����*/
//        ctl_v = pid_calc(&RouteCtl.pid_ang, trace->real_value[0] - (-trace->real_value[1])) 
//                + GetCurSpd() * SEESAW_FORWARD_ANG_KF;
//        rotate_speed(ctl_v);

//        //�ٶȿ���
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
//                if (err_cnt > 50) /*��������*/
//                {
//                    UnderpanBrake(); 
//                    USART_printf("����ͷ��ʧ\r\n");
//                    while(1);
//                }
//                
//            }
//        }
//        else //����DT50�ı����������趨ֵ����800���ڱ�����
//        {
//            if  (GetPosSensorValue(sensor1) >  trace->set_value[0] - 800 && GetPosSensorValue(sensor1) < trace->set_value[0] + 800)
//            {
//                err_cnt = 0;
//                trace->real_value[0] = (int32_t)GetPosSensorValue(sensor1);
//            }
//            else
//            {
//                 err_cnt++;
//                if (err_cnt > 50) /*��������*/
//                {
//                    UnderpanBrake(); 
//                    LCD12864_Clear();
//                    LCD12864_Printf(3,0,"DT50 ERR");
//                    USART_printf("DT50����\r\n");
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
//        /*�Ƕȱ���*/
//        if (my_abs(trace->real_value[1] - (int32_t)(trace->set_value[1] * 10))> 400) 
//        {
//            UnderpanBrake();
//            LCD12864_Clear();
//            LCD12864_Printf(0,0,"ANGLE!");
//            LCD12864_Printf(1,0,"rea:%.1f", (float)(trace->real_value[1]) / 10.0f);
//            LCD12864_Printf(2,0,"set:%.1f", (float)(trace->set_value[1]) / 10.0f);
//            USART_printf("�Ƕȱ�����real:%.1f,set:%.1f",
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



/*�Ƕȿ���*/
void lock_angle_ctl(void)
{
    point_t * p_pos = GetCurPosPointer();               /*��ǰ�����ָ��*/
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
/*�������*/
void lock_pos_ctl(void)
{
    point_t * p_pos = GetCurPosPointer();               /*��ǰ�����ָ��*/
    vect_t target_vect; /*��ǰ��ָ��Ŀ��������*/
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






