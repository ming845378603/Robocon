#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "My_Math.h"
#include <stdint.h>
#include "PID_Parameter.h"
/*每1/4圆弧分段数*/
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
/*点坐标*/
typedef struct 
{
    int32_t x;
    int32_t y;
}
point;

/*路径设置结构体*/
typedef struct 
{
	point     start;			  /*起点坐标*/
	point     end;		      /*终点坐标*/
	point     center;			  /*圆心坐标*/
	int8_t      dir;		  /*弧线方向    1:顺时针； 0:逆时针*/
	int32_t    count;      /*程序更新次数*/
}
route_arc;

/*圆弧路径更新*/
void UpdateRoute_Arc(int32_t count);

/*设定圆弧路径*/							
void SetupArc(int32_t x0, int32_t y0, 
              int32_t x1, int32_t y1,
              int32_t cx, int32_t cy, 
              int8_t Dir, int32_t count);

/*弧线路径计算*/
void Calculate_Arc(route_arc * NewRoute);

/*计算m,n两点之间的距离，单位：mm*/
int32_t get_len(point x1,point x2);

/*PID参数设定*/
/*PID参数设定*/
void setup_PID(int32_t Angle_P,  int32_t Angle_D,  int32_t Angle_Max,  int32_t Angle_Min,
	             int32_t X_P,  int32_t X_D,  int32_t X_Max,  int32_t X_Min,
               int32_t Y_P,  int32_t Y_D,  int32_t Y_Max,  int32_t Y_Min);

