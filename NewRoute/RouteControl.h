#ifndef _ROUTECONTROL_H_
#define _ROUTECONTROL_H_ 

#include "stm32f4xx.h"
#include "NewVect.h"
//#include "Global.h"
#define ROUTE_CONTROL_PERIOD_MS  (CONTROL_PERIOD_MS) /*·����������ms*/
#define ROUTE_CONTROL_PERIOD_S   ((float)ROUTE_CONTROL_PERIOD_MS / 1000.0f)

#define SEESAW_FORWARD_POS_KF (-22) 
#define SEESAW_FORWARD_ANG_KF (G_Param.ground_type == GROUND_BLUE? (25) : (-25))  

/*****************************************************************************/

/*λ��ʽPID*/
typedef struct
{
    double set_value;       /*����ֵ*/
    double real_value[2];   /*���ֵ(ʵ��ֵ)*/
    double err[2];          /*ƫ��*/
    double pid_out;         /*PID������*/
    double err_iteg;        /*������*/
    /*Param*/
    double Kp;              /*KP*/
    double Ki;              /*Ki*/
    double Kd;              /*Kd*/
    double out_max;         /*PID�����������ֵ*/
    double out_min;         /*PID����������Сֵ*/
    double iteg_max;        /*�����޷�*/
    double dead_zone;       /*����*/
}
pid_t;




/*ǰ�������б�*/
enum forward_index
{
    NORMAL_LINE_FORWARD_PARAM = 0,
    NORMAL_ROT_FORWARD_PARAM,
    NORMAL_ARC_FORWARD_PARAM,
    POLE_ROT_FORWARD_PARAM,
    FORWARD_NUM,
};

/*PID�����б�*/
enum pid_index
{
    PID_LOCK_LINE_PARAM = 0, //����ֱ�߹켣PID
    PID_LOCK_ARC_PARAM,      //�������߹켣PID
    PID_LOCK_ANG_PARAM,      //�����Ƕ�PID
    PID_LOCK_TRACE_LOCUS,    //Ѳ�������켣
    PID_LOCK_TRACE_ANGLE,    //Ѳ�������Ƕ�
    PID_LOCK_POINT,          //����PID
    PID_SWING_TRACE_LOCUS,   //��ǧ����PID
    PID_POLE_TRACE_LOCUS,    //÷��׮DT50Ѳ��
    PID_LIDAR_LOCK,          //�����״�����
    
    PID_LOCK_TRACE_LOCUS_0SPEED,//���ΰ�Ѳ��ֹͣ
    PID_LOCK_TRACE_ANGLE_0SPEED, //���ΰ�Ѳ��ֹͣ
    
    PID_LOCK_TRACE_LOCUS_HANDOVER, //���ΰ彻��
    PID_LOCK_TRACE_ANGLE_HANDOVER, //���ΰ彻��
    PID_NUM,
};

/*****************************************************************************/

/*Ѳ����Ϣ*/
typedef struct
{
    uint8_t sensor[2];    /*Ѳ���õĴ�����*/
    int32_t set_value[2]; /*Ŀ��ֵ*/
    vect_t  trace_vect;   /*�������ķ���*/
    vect_t  move_vect;    /*��Գ�����˶�����*/
}trace_inf_t;

/*�ٶȲ���*/
typedef struct
{
	int32_t unif_v;		   	/*�����ٶ�*/
	int32_t end_v;			/*ĩ�ٶ�*/
	int32_t aclt;			/*���ٶ�*/
	int32_t decr;			/*���ٶ�*/
}							/*���ٶ�Ϊ����ʵ���ٶ�*/
spd_inf_t;

/*·����Ϣ����*/
typedef struct 
{
	point_t     start;			  /*�������*/
	point_t     end;		      /*�յ�����*/
	point_t     center;			  /*Բ������*/
	int8_t      route_type;		  /*·�����ͣ�ֱ��or����(����),Ѳ��(����+����),����*/
	spd_inf_t   spd_inf;		  /*·�ε��ٶȿ�����Ϣ*/
	uint8_t     forward_param[2]; /*ǰ�����Ʋ���*/
	uint8_t     pid_param[2];     /*PID���Ʋ���*/
	trace_inf_t trace_inf;        /*Ѳ����Ϣ*/
}
route_inf_t;
/*****************************************************************************/
//·�����ͣ�ֱ�ߣ�˳ʱ�뻡�䣬��ʱ�뻡��
enum _route_type
{
    ROUTE_LINE = 0,
    ROUTE_CLK = -1,
    ROUTE_ANTI_CLK = 1,
    ROUTE_LOCK = 2,
    ROUTE_TRACE = 3, //Ѳ������
};

/*·������*/
void UpdateRouteCtl(route_inf_t * NewRoute);
/*·������*/
void RouteControl(void);
/*���·������ʱ��*/
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


/*�߳�����*/
/*����*/
extern const point_t StartPoint_BLUE;
extern point_t SeesawPoint_BLUE; //*��Բ�ĵ� 0x00
extern const uint8_t SeesawPoint_BLUE_addr;
extern point_t SwingPoint_BLUE;//*������ǧѲ�ߵĵ� 0x00 + 12
extern const uint8_t SwingPoint_BLUE_addr;
extern point_t PolePutPoint_BLUE; //*÷��׮�Ž���Ѳ��֮ǰ 0x00 + 12 * 2
extern const uint8_t PolePutPoint_BLUE_addr;

extern int32_t SeesawCam_BLUE[2]; // 0x00 + 12 * 2 + 4, 0x00 + 12* 2 + 8
extern const uint8_t SeesawCam_BLUE_addr[2];
extern int32_t SwingCam_BLUE;//0x00 + 12*2 + 8 + 4
extern const uint8_t SwingCam_BLUE_addr;
extern int32_t DT50_PUT_BLUE;//0x00 + 12*2 + 8 + 4*2
extern const uint8_t DT50_PUT_BLUE_addr;
extern int32_t DT50_GET_BLUE;//0x00 + 12*2 + 8 + 4*3
extern const uint8_t DT50_GET_BLUE_addr;

/*�쳡*/
extern const point_t StartPoint_RED;
extern point_t SeesawPoint_RED; //��Բ�ĵ� 0x32
extern const uint8_t SeesawPoint_RED_addr;
extern point_t SwingPoint_RED;//������ǧѲ�ߵĵ� 0x32 + 12
extern const uint8_t SwingPoint_RED_addr;
extern point_t PolePutPoint_RED; //÷��׮�Ž���Ѳ��֮ǰ 0x32+12*2
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
