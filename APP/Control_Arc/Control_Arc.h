#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "My_Math.h"
#include <stdint.h>
#include "PID_Parameter.h"
/*ÿ1/4Բ���ֶ���*/
#define N10 10
#define N20 20
#define N40 40
#define N50 50
#define N60 60
#define N80 80
#define N100 100
#define N120 120
#define N150 150
#define N200 200
#define N250 250
#define N300 300
#define PI 3.1415926
/*������*/
typedef struct 
{
    int32_t x;
    int32_t y;
}
point;

/*·�����ýṹ��*/
typedef struct 
{
	point     start;			  /*�������*/
	point     end;		      /*�յ�����*/
	point     center;			  /*Բ������*/
	int8_t      dir;		  /*���߷���    1:˳ʱ�룻 0:��ʱ��*/
	int32_t    count;      /*������´���*/
}
route_arc;

/*Բ��·������*/
void UpdateRoute_Arc(int32_t count);

/*�趨Բ��·��*/							
void SetupArc(int32_t x0, int32_t y0, 
              int32_t x1, int32_t y1,
              int32_t cx, int32_t cy, 
              int8_t Dir, int32_t count);

/*����·������*/
void Calculate_Arc(route_arc * NewRoute);

/*����m,n����֮��ľ��룬��λ��mm*/
int32_t get_len(point x1,point x2);

/*PID�����趨*/
/*PID�����趨*/
void setup_PID(int32_t Angle_P,  int32_t Angle_D,  int32_t Angle_Max,  int32_t Angle_Min,
	             int32_t X_P,  int32_t X_D,  int32_t X_Max,  int32_t X_Min,
               int32_t Y_P,  int32_t Y_D,  int32_t Y_Max,  int32_t Y_Min);

