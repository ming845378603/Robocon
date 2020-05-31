#include "Control_Arc.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "GYRO_Lib.h"
#include "Basal_Move.h"
#include "PID_Parameter.h"
#include "PID_forward.h"
#include "beep.h"
#include "can.h"
#include "PID_forward.h"
#include "Route_Control.h"
#include "usart3.h"
extern s16 targetX, targetY1, targetA;
extern int32_t Stop_flag;
extern u8 Step_flag;
extern u8 Start_flag;
u8 up_step = 0;
/*����·������*/
void UpdateRoute_Arc(int32_t count)
{
    SetupArc(0,0,-750,750,0,750,1,count);
}
/*���߲����趨*/
void SetupArc(int32_t x0, int32_t y0,
              int32_t x1, int32_t y1,
              int32_t cx, int32_t cy,
              int8_t Dir, int32_t count)
{
    route_arc  RouteToSet;
    memset(&RouteToSet,0,sizeof(route_arc));
    /*�������*/
    RouteToSet.start.x = x0;
    RouteToSet.start.y = y0;
    /*�����յ�*/
    RouteToSet.end.x = x1;
    RouteToSet.end.y = y1;
    /*����Բ��*/
    RouteToSet.center.x = cx;
    RouteToSet.center.y = cy;
    /*������ת����1:˳ʱ�룻 0:��ʱ��*/
    RouteToSet.dir = Dir;
    /*������´���*/
    RouteToSet.count = count;

    Calculate_Arc(&RouteToSet);/*����·������*/
}

/*����·������*/
void Calculate_Arc(route_arc * NewRoute)
{
    /*���߰뾶*/
//   	int32_t R = get_len(NewRoute->start,NewRoute->center);
    /*����΢�ֽǶ�*/
//	  float d_angle = PI/(2*N);
    float d_len10 = (float)NewRoute->count / (float)N10;
    float d_len20 = (float)NewRoute->count / (float)N20;
    float d_len40 = (float)NewRoute->count / (float)N40;
    float d_len50 = (float)NewRoute->count / (float)N50;
    float d_len60 = (float)NewRoute->count / (float)N60;
    float d_len80 = (float)NewRoute->count / (float)N80;
    float d_len100 = (float)NewRoute->count / (float)N100;
    float d_len120 = (float)NewRoute->count / (float)N120;
    float d_len150 = (float)NewRoute->count / (float)N150;
    float d_len200 = (float)NewRoute->count / (float)N200;
    float d_len250 = (float)NewRoute->count / (float)N250;
    float d_len300 = (float)NewRoute->count / (float)N300;
    /*������һ����Ŀ������*/
    if(Start_flag==1)
    {

        if (Step_flag == 1)    //o-->A
        {
            targetX = 0 - 1200*d_len100;
            targetY1 = 0;
            setup_PID(500,100,1000,1000,
                      500,100,1000,1000,
                      500,100,1000,1000);
        }
        if(Step_flag == 2)    //A-->B
        {
            targetX = -1200 + 0;
            targetY1 = 2050*d_len200;
            setup_PID(500,100,1000,1000,
                      500,100,1000,1000,
                      500,100,1000,1000);
        }
        if(Step_flag == 3)    //B-->C
        {
            targetX = -1200 + 1200*d_len100;
            targetY1 = 2050 + 0;
            setup_PID(500,100,1000,1000,
                      500,100,1000,1000,
                      500,100,1000,1000);
        }
        if(Step_flag == 4)   //C-->D
        {
            targetX = 0 + 0;
            targetY1 = 2050+1500*d_len150;
            setup_PID(500,100,1000,1000,
                      500,100,1000,1000,
                      500,100,1000,1000);
        }
        if(Step_flag == 5)   //D-->E
        {
            targetX = 0 - 1200*d_len100;
            targetY1 = 3550;
            setup_PID(500,100,1000,1000,
                      500,100,1000,1000,
                      500,100,1000,1000);
        }
        if(Step_flag == 6)   //E-->F
        {
            targetX = -1200 + 0;
            targetY1 = 3550 + 1000*d_len100;
            setup_PID(500,100,1000,1000,
                      500,100,1000,1000,
                      500,100,1000,1000);
        }
        if(Step_flag == 7)   //F-->M
        {
            targetX = -1200 + 630*d_len100;
            targetY1 = 4550 + 750*d_len100;
            setup_PID(500,100,1000,1000,
                      500,100,1000,1000,
                      500,100,1000,1000);
        }
        if(Step_flag == 8)   //M-->N
        { 
            targetX = -570 + 0;
            targetY1 = 5300 + 2300*d_len250;
            setup_PID(500,100,1000,1000,
                      500,100,1000,1000,
                      500,100,1200,1200);
        }
        if(Step_flag == 9)   //N-->Q
        {
            targetX = -570 + 0;
            targetY1 = 7600 + 200*d_len20;
            setup_PID(500,100,800,800,
                      500,100,800,800,
                      500,100,800,800);
        }

        if(Step_flag == 10)   //Q-->W  ȥ���ӣ�ͬʱ������ת
        {
            targetX = -570 - 3730*d_len300;
            targetY1 = 7800 + 0;
            targetA = -90;
            setup_PID(1000,200,1500,1500,
                      500,100,1600,1600,
                      500,100,500,500);
        }

        if(Step_flag == 11)   //W-->X ����ɲ����������ת
        {
            targetX = -4300 - 200*d_len200;
            targetY1 = 7800 + 0;
            targetA = 0;
            setup_PID(1000,200,1300,1300,
                      500,100,400,400,
                      500,100,400,400);						
        }

			 if(Step_flag == 12)   //�������λ��
        {
            targetX = -4500 + 0;
            targetY1 = 7800 + 0;
            targetA = 0+0;
            setup_PID(1000,200,1300,1300,
                      500,100,400,400,
                      500,100,400,400);	
        }
    }

    if(Start_flag == 3)			  //�ڶ�����ȥץ�޹�
    {
        if(Step_flag == 1)   //��ȥ���޹�
        {
           
        }
        if(Step_flag == 2)   //X-->Y��צ���ſ����ϲ����
        {

        }
				if(Step_flag == 3)   //X-->Y��צ���ſ����ϲ����
        {
            targetX = -4500 + 0;
            targetY1 = 7800 + 200;
            targetA = 0;
            setup_PID(1000,200,500,500,
                      500,100,500,500,
                      500,100,500,500);
        }


    }

    if(Start_flag == 5)			  //��������ץȡ�޹� �� ����ȴ�λ��
    {
        if(Step_flag == 1)   //ץȡ�޹�
        {
	
        }
        if(Step_flag == 2)   //�ϲ㷭ת
        {

        }
        if(Step_flag == 3)   //Y-->Z
        {
            targetX = -4500 + 1400*d_len200;
            targetY1 = 8000 - 200*d_len200;
            targetA = 0;
            setup_PID(1000,200,500,500,
                      500,100,500,500,
                      500,100,500,500);
        }

    }


    if(Start_flag == 7)        //���Ĳ�������Ͷ���� �� ����Ͷ��
    {
        if(Step_flag==1)          //Z-->R
        {
            targetX = -3150 + 0;
            targetY1 = 7800 - 3700*d_len300;
            targetA = 0;
            setup_PID(1000,200,1300,1300,
                      500,100,500,500,
                      500,100,1600,1600);
        }
        if(Step_flag==2)          //R-->T
        {
            targetX = -3150 + 0;
            targetY1 = 4100 - 100*d_len20;
            targetA = 0;
            setup_PID(1000,200,1300,1300,
                      500,100,300,300,
                      500,100,300,300);
        }

        if(Step_flag==3)          //������ת
        {
            targetX = -3150 + 0;
            targetY1 = 4000 + 0;
            targetA = 0 + 40*d_len20;
            setup_PID(1000,200,800,800,
                      500,100,300,300,
                      500,100,300,300);
        }
        if(Step_flag==4)          //װ��ǰ��
        {				

        }
        if(Step_flag==5)          //�ɿ�צ��
        {

        }
        if(Step_flag==6)          //���޹�
        {					

        }
    }
		
/*************************�ڶ������޹�************************/	
		if(Start_flag == 9)//���岽����ȥ���޹�
		{
				if(Step_flag==1)          //�ϲ��ջأ��Ƕ�ת�أ�T-->L
				{
            targetX = -3150;
            targetY1 = 4000 + 3600*d_len300;
            targetA = 0;
            setup_PID(1000,200,800,800,
                      500,100,1000,1000,
                      500,100,1600,1600);
        }
				if(Step_flag==2)          //L-->K��ɲ��
				{
            targetX = -3150 + 0;
            targetY1 = 7600 + 200*d_len20;
            targetA = 0;
            setup_PID(1000,200,1300,1300,
                      500,100,300,300,
                      500,100,300,300);
        }
				if(Step_flag==3)          //�ϲ���ת�£�צ���ɿ�
				{

        }
		}

		if(Start_flag == 11)//��������ץ�޹ǣ��ߣ����޹�
		{
				if(Step_flag==1)          //ץȡ�޹�
        {

        }
        if(Step_flag==2)          //�ϲ㷭ת
        {

        }		
				if(Step_flag==3)          //Z-->R
        {
            targetX = -3150 + 0;
            targetY1 = 7800 - 3600*d_len300;
            targetA = 0;
            setup_PID(1000,200,1300,1300,
                      500,100,500,500,
                      500,100,1600,1600);
        }
        if(Step_flag==4)          //R-->T
        {
            targetX = -3150 + 0;
            targetY1 = 4200 - 200*d_len20;
            targetA = 0;
            setup_PID(1000,200,1300,1300,
                      500,100,300,300,
                      500,100,300,300);
        }
        if(Step_flag==5)          //������ת
        {
            targetX = -3150 + 0;
            targetY1 = 4000 + 0;
            targetA = 0 + 40*d_len20;
            setup_PID(1000,200,800,800,
                      500,100,300,300,
                      500,100,300,300);
        }
        if(Step_flag==6)          //װ��ǰ��
        {
	
        }
        if(Step_flag==7)          //�ɿ�צ��
        {

        }
        if(Step_flag==8)          //���޹�
        {

        }				
		}
/*************************���������޹�***********************/
		if(Start_flag == 13)//���߲�����ȥ���޹�
		{
				if(Step_flag==1)          //�ϲ��ջأ��Ƕ�ת�أ�T-->L
				{
            targetX = -3150 + 550;
            targetY1 = 4000 + 3600*d_len300;
            targetA = 0;
            setup_PID(1000,200,800,800,
                      500,100,600,600,
                      500,100,1600,1600);
        }
				if(Step_flag==2)          //L-->K��ɲ��
				{
            targetX = -2600;
            targetY1 = 7600 + 200*d_len20;
            targetA = 0;
            setup_PID(1000,200,1300,1300,
                      500,100,300,300,
                      500,100,300,300);
        }
				if(Step_flag==3)          //�ϲ���ת�£�צ���ɿ�
				{

        }
		}

		if(Start_flag == 15)//�ڰ˲���ץ�޹ǣ��ߣ����޹�
		{
				if(Step_flag==1)          //ץȡ�޹�
        {

        }
        if(Step_flag==2)          //�ϲ㷭ת
        {

        }		
				if(Step_flag==3)          //Z-->R
        {
            targetX = -2600 - 550;
            targetY1 = 7800 - 3600*d_len300;
            targetA = 0;
            setup_PID(1000,200,1300,1300,
                      500,100,500,500,
                      500,100,1600,1600);
        }
        if(Step_flag==4)          //R-->T
        {
            targetX = -3150 + 0;
            targetY1 = 4200 - 200*d_len20;
            targetA = 0;
            setup_PID(1000,200,1300,1300,
                      500,100,300,300,
                      500,100,300,300);
        }
        if(Step_flag==5)          //������ת
        {
            targetX = -3150 + 0;
            targetY1 = 4000 + 0;
            targetA = 0 + 40*d_len20;
            setup_PID(1000,200,800,800,
                      500,100,300,300,
                      500,100,300,300);
        }
        if(Step_flag==6)          //װ��ǰ��
        {
	
        }
        if(Step_flag==7)          //�ɿ�צ��
        {

        }
        if(Step_flag==8)          //���޹�
        {

        }				
		}
		
/********************�س�**********************/		
    if(Start_flag == 20)        //�س�
    {
        if(Step_flag==1)          //T-->Z
        {
            targetX = -3150 + 0;
            targetY1 = 7800;
            targetA = 0;
            setup_PID(1000,200,500,500,
                      500,100,1000,1000,
                      500,100,1000,1000);
        }
        if(Step_flag==2)          //Z-->Q
        {
            targetX = 0;
            targetY1 = 7800;
            targetA = 0;
            setup_PID(1000,200,500,500,
                      500,100,1000,1000,
                      500,100,1000,1000);
        }
        if(Step_flag==3)          //Q-->O
        {
            targetX = 0;
            targetY1 = 0;
            targetA = 0;
            setup_PID(1000,200,500,500,
                      500,100,1000,1000,
                      500,100,1000,1000);
        }

    }
		targetX = -targetX;

}

/*����m,n����֮��ľ��룬��λ��mm*/
int32_t get_len(point m,point n)
{
    int32_t len;
    len = my_sqrt(square(m.x-n.x)+square(m.y-n.y));
    return len;
}

/*PID�����趨*/
void setup_PID(int32_t Angle_P,  int32_t Angle_D,  int32_t Angle_Max,  int32_t Angle_Min,
               int32_t X_P,  int32_t X_D,  int32_t X_Max,  int32_t X_Min,
               int32_t Y_P,  int32_t Y_D,  int32_t Y_Max,  int32_t Y_Min)
{
    G_PID_Parameter[ROUTE_LINE].PID_Angle.Parameter.P=Angle_P;
    G_PID_Parameter[ROUTE_LINE].PID_Angle.Parameter.I=0;
    G_PID_Parameter[ROUTE_LINE].PID_Angle.Parameter.D=Angle_D;
    G_PID_Parameter[ROUTE_LINE].PID_Angle.Parameter.Out_Max=Angle_Max;
    G_PID_Parameter[ROUTE_LINE].PID_Angle.Parameter.Out_Min=-Angle_Min;
    G_PID_Parameter[ROUTE_LINE].PID_Angle.Parameter.Dead_Zone=0;

    G_PID_Parameter[ROUTE_LINE].PID_LineX.Parameter.P=X_P;
    G_PID_Parameter[ROUTE_LINE].PID_LineX.Parameter.I=0;
    G_PID_Parameter[ROUTE_LINE].PID_LineX.Parameter.D=X_D;
    G_PID_Parameter[ROUTE_LINE].PID_LineX.Parameter.Out_Max=X_Max;
    G_PID_Parameter[ROUTE_LINE].PID_LineX.Parameter.Out_Min=-X_Min;
    G_PID_Parameter[ROUTE_LINE].PID_LineX.Parameter.Dead_Zone=0;

    G_PID_Parameter[ROUTE_LINE].PID_LineY.Parameter.P=Y_P;
    G_PID_Parameter[ROUTE_LINE].PID_LineY.Parameter.I=0;
    G_PID_Parameter[ROUTE_LINE].PID_LineY.Parameter.D=Y_D;
    G_PID_Parameter[ROUTE_LINE].PID_LineY.Parameter.Out_Max=Y_Max;
    G_PID_Parameter[ROUTE_LINE].PID_LineY.Parameter.Out_Min=-Y_Min;
    G_PID_Parameter[ROUTE_LINE].PID_LineY.Parameter.Dead_Zone=0;


}

