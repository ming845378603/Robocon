#include "includes.h"
#include "Basal_Move.h"
#include "RouteControl.h"

//OSTaskCreate(ManualTask,(void *) 0, &ManualTaskSTK[MANUAL_TASK_SIZE - 1], MANUAL_TASK_PRIO);
point_t PolePoint = {0,0,0.0f};
point_t StopPoint = {0,0,0.0f};
int32_t temp_speed[4] = {1000,0,2000,2000};
/*�ֶ�������������´���·������*/
void ManualTask(void * p_arg)
{
    (void)p_arg;
    
    while(1)
    {
        if (KEY_w == 0 && KEY_s == 0 && KEY_a == 0 && KEY_d == 0 
            && KEY_BL ==0 && KEY_BR == 0)
        {
            KEY_CLR();
            UnderpanBrake();
        }
        else
        {
            if (KEY_w == 1)
            {
                KEY_CLR();
                linear_speed(0,10,0,SPEED(200));
                SetMotorSpeed();
            }
            if (KEY_s == 1)
            {
                KEY_CLR();
                linear_speed(0,-10,0,SPEED(200));
                SetMotorSpeed();
            }
            if (KEY_a == 1)
            {
                KEY_CLR();
                linear_speed(-10,0,0,SPEED(200));
                SetMotorSpeed();
            }
            if (KEY_d == 1)
            {
                KEY_CLR();
                linear_speed(10,0,0,SPEED(200));
                SetMotorSpeed();
            }
            if (KEY_BL == 1)
            {
                KEY_CLR();
                rotate_speed(SPEED(200));
                SetMotorSpeed();
            }
            if (KEY_BR == 1)
            {
                KEY_CLR();
                rotate_speed(SPEED(-200));
                SetMotorSpeed();
            }
        }
         /*�����ƶ�*/
        if (KEY_UR == 1)
        {
            KEY_CLR();
            StopPoint.x = G_Param.cur_pos.x;
            StopPoint.y = G_Param.cur_pos.y;
            StopPoint.ang = G_Param.cur_pos.ang;
            if (GetRoute() == POLE_ROUTE4) //û��⵽÷��׮�Ļ�
            {
                if (GetGroundType() == GROUND_BLUE)
                {
                    SetupLine(StopPoint.x,StopPoint.y, StopPoint.ang,
                      StopPoint.x - 300,PolePoint.y, PolePoint.ang,
                      temp_speed,
                      NORMAL_LINE_FORWARD_PARAM,NORMAL_ROT_FORWARD_PARAM,
                      PID_LOCK_LINE_PARAM,PID_LOCK_ANG_PARAM);
                    while(1)
                    {
                        RouteControl();//·������
                        SetMotorSpeed();//ת����������
                        OSTimeDly(10);
                        //�ж��Ƿ�����·���л�����
                        if (GetCurTimeMS() >= GetRouteEndTime())
                        {
                            break;
                        }
                        
                    }
                    SetupLine(StopPoint.x - 300,PolePoint.y, PolePoint.ang,
                              PolePoint.x,PolePoint.y, PolePoint.ang,
                              temp_speed,
                              NORMAL_LINE_FORWARD_PARAM,NORMAL_ROT_FORWARD_PARAM,
                              PID_LOCK_LINE_PARAM,PID_LOCK_ANG_PARAM);
                    while(1)
                    {
                        RouteControl();//·������
                        SetMotorSpeed();//ת����������
                        OSTimeDly(10);
                        //�ж��Ƿ�����·���л�����
                        if (GetCurTimeMS() >= GetRouteEndTime())
                        {
                            break;
                        }
                        
                    }
                    Buzzer(2,100,100);
                }
                 else
                {
                    SetupLine(StopPoint.x,StopPoint.y, StopPoint.ang,
                      StopPoint.x + 300,PolePoint.y, PolePoint.ang,
                      temp_speed,
                      NORMAL_LINE_FORWARD_PARAM,NORMAL_ROT_FORWARD_PARAM,
                      PID_LOCK_LINE_PARAM,PID_LOCK_ANG_PARAM);
                    while(1)
                    {
                        RouteControl();//·������
                        SetMotorSpeed();//ת����������
                        OSTimeDly(10);
                        //�ж��Ƿ�����·���л�����
                        if (GetCurTimeMS() >= GetRouteEndTime())
                        {
                            break;
                        }
                        
                    }
                    SetupLine(StopPoint.x + 300,PolePoint.y, PolePoint.ang,
                              PolePoint.x,PolePoint.y, PolePoint.ang,
                              temp_speed,
                              NORMAL_LINE_FORWARD_PARAM,NORMAL_ROT_FORWARD_PARAM,
                              PID_LOCK_LINE_PARAM,PID_LOCK_ANG_PARAM);
                    while(1)
                    {
                        RouteControl();//·������
                        SetMotorSpeed();//ת����������
                        OSTimeDly(10);
                        //�ж��Ƿ�����·���л�����
                        if (GetCurTimeMS() >= GetRouteEndTime())
                        {
                            break;
                        }
                        
                    }
                    Buzzer(2,100,100);
                }
            }
        }
        OSTimeDly(50);
    }
}














