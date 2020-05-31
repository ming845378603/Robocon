#include "task.h"
#include "led.h"
#include "Control_Arc.h"
#include "PID_Control.h"
#include "Basal_Move.h"
#include "Robot.h"
/*******************************************************************************
* @FunctionName   : Loop_check
* @Description    : Loop_check
* @Param          : None
* @Retval         : None
*******************************************************************************/
loop_t loop;
void Loop_check()  //TIME INTTERRUPT
{
    loop.time++; //u16
    loop.cnt_10ms++;
    loop.cnt_20ms++;
    loop.cnt_50ms++;
    loop.cnt_100ms++;
    loop.cnt_200ms++;
    loop.cnt_500ms++;

    if( loop.check_flag >= 2)
    {
        loop.err_flag ++;
    }
    else
    {
        loop.check_flag += 1;
    }
}

/*******************************************************************************
* @FunctionName   : main_loop
* @Description    : main_loop
* @Param          : None
* @Retval         : None
*******************************************************************************/
void main_loop()
{
    if( loop.check_flag >= 1 )
    {
        if( loop.cnt_10ms >= 10 )
        {
            loop.cnt_10ms = 0;
            Duty_10ms();
        }
        if( loop.cnt_20ms >= 20 )
        {
            loop.cnt_20ms = 0;
            Duty_20ms();
        }
        if( loop.cnt_50ms >= 50 )
        {
            loop.cnt_50ms = 0;
            Duty_50ms();
        }
        if( loop.cnt_100ms >= 100 )
        {
            loop.cnt_100ms = 0;
            Duty_100ms();
        }
        if( loop.cnt_200ms >= 200 )
        {
            loop.cnt_200ms = 0;
            Duty_200ms();
        }
        if( loop.cnt_500ms >= 500 )
        {
            loop.cnt_500ms = 0;
            Duty_500ms();
        }
        loop.check_flag = 0;
    }
}

extern u8 Start_flag;
u8 Step_flag = 1;
int32_t Count_Flag = 0;
extern s16 targetX, targetY1, targetA;

void Duty_10ms(void)
{
    if(Start_flag==1)           //��һ��
    {
        if(Step_flag==1)          //o-->A
        {
            Count_Flag++;
            UpdateRoute_Arc(Count_Flag);
            if(Count_Flag == N100)
            {
                Step_flag = 2;
                Count_Flag = 0;
            }
        }
        if(Step_flag==2)     //A-->B
        {
            Count_Flag++;
            UpdateRoute_Arc(Count_Flag);
            if(Count_Flag == N200)
            {
                Step_flag = 3;
                Count_Flag = 0;
            }
        }
        if(Step_flag==3)       //B-->C
        {
            Count_Flag++;
            UpdateRoute_Arc(Count_Flag);
            if(Count_Flag == N100)
            {
                Step_flag = 4;
                Count_Flag = 0;
            }
        }
        if(Step_flag==4)         //C-->D
        {
            Count_Flag++;
            UpdateRoute_Arc(Count_Flag);
            if(Count_Flag == N150)
            {
                Step_flag = 5;
                Count_Flag = 0;
            }
        }
        if(Step_flag==5)       //D-->E
        {
            Count_Flag++;
            UpdateRoute_Arc(Count_Flag);
            if(Count_Flag == N100)
            {
                Step_flag = 6;
                Count_Flag = 0;
            }
        }
        if(Step_flag==6)   //E-->F
        {
            Count_Flag++;
            UpdateRoute_Arc(Count_Flag);
            if(Count_Flag == N100)
            {
                Step_flag = 7;
                Count_Flag = 0;
            }
        }
        if(Step_flag==7)      //F-->M
        {
            Count_Flag++;
            UpdateRoute_Arc(Count_Flag);
            if(Count_Flag == N100)
            {
                Step_flag = 8;
                Count_Flag = 0;
            }
        }
        if(Step_flag==8)         //M-->N
        {
            Count_Flag++;
            UpdateRoute_Arc(Count_Flag);
            if(Count_Flag == N250)
            {
                Step_flag = 9;
                Count_Flag = 0;
            }
        }

        if(Step_flag==9)         //N-->Q
        {
            Count_Flag++;
            UpdateRoute_Arc(Count_Flag);
            if(Count_Flag == N20)
            {
                Step_flag = 10;
                Count_Flag = 0;
            }
        }

        if(Step_flag==10)         //Q-->W ,ȥ����
        {
            Count_Flag++;
            UpdateRoute_Arc(Count_Flag);
            if(Count_Flag == N300)
            {
                Step_flag = 11;
                Count_Flag = 0;
            }
        }

        if(Step_flag==11)         //W-->X������ɲ��
        {
            Count_Flag++;
            UpdateRoute_Arc(Count_Flag);
            if(Count_Flag == N200) 
            {
                Step_flag = 12;
                Count_Flag = 0;
            }
        }
				
				if(Step_flag==12)         //�����������λ��
        {
            Count_Flag++;
            UpdateRoute_Arc(Count_Flag);
            if(Count_Flag == N100) 
            {
                Step_flag = 1;
                Count_Flag = 0;
                Start_flag = 2;
            }
        }			
				
    }

    if(Start_flag == 3)			  //�ڶ���
    {
        if(Step_flag==1)          //
        {
            Count_Flag++;
            UpdateRoute_Arc(Count_Flag);
            if(Count_Flag == N10)
            {
                Step_flag = 2;
                Count_Flag = 0;
            }
        }

        if(Step_flag==2)          //������
        {
            Count_Flag++;
            UpdateRoute_Arc(Count_Flag);
            if(Count_Flag == N10)
            {
                Step_flag = 3;
                Count_Flag = 0;
            }
        }
				
		  	if(Step_flag==3)          //x--->y,��צ�ӣ����������
        {
            Count_Flag++;
            UpdateRoute_Arc(Count_Flag);
            if(Count_Flag == N300)
            {
                Step_flag = 1;
                Count_Flag = 0;
                Start_flag = 4;
            }
        }
    }

    if(Start_flag == 5)	     //��������ץȡ�޹� �� ����ȴ�λ��
    {
        if(Step_flag==1)          //ץȡ�޹�
        {
            Count_Flag++;
            UpdateRoute_Arc(Count_Flag);
            if(Count_Flag == N10)
            {
                Step_flag = 2;
                Count_Flag = 0;
            }
        }
        if(Step_flag==2)          //�ϲ㷭ת
        {
            Count_Flag++;
            UpdateRoute_Arc(Count_Flag);
            if(Count_Flag == N10)
            {
                Step_flag = 3;
                Count_Flag = 0;
            }
        }
        if(Step_flag==3)          //Y-->Z
        {
            Count_Flag++;
            UpdateRoute_Arc(Count_Flag);
            if(Count_Flag == N200)
            {
                Step_flag = 1;
                Count_Flag = 0;
                Start_flag = 6;
            }
        }

    }

    if(Start_flag == 7)        //���Ĳ�������Ͷ���� �� ����Ͷ��
    {
        if(Step_flag==1)          //Z-->R
        {
            Count_Flag++;
            UpdateRoute_Arc(Count_Flag);
            if(Count_Flag == N300)
            {
                Step_flag = 2;
                Count_Flag = 0;
            }
        }
        if(Step_flag==2)          //R-->T
        {
            Count_Flag++;
            UpdateRoute_Arc(Count_Flag);
            if(Count_Flag == N20)
            {
                Step_flag = 3;
                Count_Flag = 0;
            }
        }
        if(Step_flag==3)          //������ת
        {
            Count_Flag++;
            UpdateRoute_Arc(Count_Flag);
            if(Count_Flag == N20)
            {
                Step_flag = 4;
                Count_Flag = 0;
            }
        }
        if(Step_flag==4)          //װ��ǰ��
        {
            Count_Flag++;
            UpdateRoute_Arc(Count_Flag);
            if(Count_Flag == N10)
            {
                Step_flag = 5;
                Count_Flag = 0;
            }
        }
        if(Step_flag==5)          //�ɿ�צ��
        {
            Count_Flag++;
            UpdateRoute_Arc(Count_Flag);
            if(Count_Flag == N10)
            {
                Step_flag = 6;
                Count_Flag = 0;
            }
        }
        if(Step_flag==6)          //���޹�
        {
            Count_Flag++;
            UpdateRoute_Arc(Count_Flag);
            if(Count_Flag == N10)
            {
                Step_flag = 1;
                Count_Flag = 0;
                Start_flag = 8;
            }
        }
    }
		
/*************************�ڶ������޹�************************/		
		if(Start_flag == 9)//��ȥ���޹�
		{
				if(Step_flag==1)          //�ϲ��ջأ��Ƕ�ת�أ�T-->L
				{
            Count_Flag++;
            UpdateRoute_Arc(Count_Flag);
            if(Count_Flag == N300)
            {
                Step_flag = 2;
                Count_Flag = 0;
            }
        }
				if(Step_flag==2)          //L-->K��ɲ��
				{
            Count_Flag++;
            UpdateRoute_Arc(Count_Flag);
            if(Count_Flag == N20)
            {
                Step_flag = 3;
                Count_Flag = 0;
            }
        }
				if(Step_flag==3)          //�ϲ���ת��
				{
            Count_Flag++;
            UpdateRoute_Arc(Count_Flag);
            if(Count_Flag == N10)
            {
                Step_flag = 1;
                Count_Flag = 0;
							  Start_flag = 10;
            }
        }
		}
		
		if(Start_flag == 11)//ץ�޹ǣ��ߣ����޹�
		{
				if(Step_flag==1)          //ץȡ�޹�
        {
            Count_Flag++;
            UpdateRoute_Arc(Count_Flag);
            if(Count_Flag == N10)
            {
                Step_flag = 2;
                Count_Flag = 0;
            }
        }
        if(Step_flag==2)          //�ϲ㷭ת
        {
            Count_Flag++;
            UpdateRoute_Arc(Count_Flag);
            if(Count_Flag == N10)
            {
                Step_flag = 3;
                Count_Flag = 0;
            }
        }		
				if(Step_flag==3)          //Z-->R
        {
            Count_Flag++;
            UpdateRoute_Arc(Count_Flag);
            if(Count_Flag == N300)
            {
                Step_flag = 4;
                Count_Flag = 0;
            }
        }
        if(Step_flag==4)          //R-->T
        {
            Count_Flag++;
            UpdateRoute_Arc(Count_Flag);
            if(Count_Flag == N20)
            {
                Step_flag = 5;
                Count_Flag = 0;
            }
        }
        if(Step_flag==5)          //������ת
        {
            Count_Flag++;
            UpdateRoute_Arc(Count_Flag);
            if(Count_Flag == N20)
            {
                Step_flag = 6;
                Count_Flag = 0;
            }
        }
        if(Step_flag==6)          //װ��ǰ��
        {
            Count_Flag++;
            UpdateRoute_Arc(Count_Flag);
            if(Count_Flag == N10)
            {
                Step_flag = 7;
                Count_Flag = 0;
            }
        }
        if(Step_flag==7)          //�ɿ�צ��
        {
            Count_Flag++;
            UpdateRoute_Arc(Count_Flag);
            if(Count_Flag == N10)
            {
                Step_flag = 8;
                Count_Flag = 0;
            }
        }
        if(Step_flag==8)          //���޹�
        {
            Count_Flag++;
            UpdateRoute_Arc(Count_Flag);
            if(Count_Flag == N10)
            {
                Step_flag = 1;
                Count_Flag = 0;
                Start_flag = 12;
            }
        }				
		}
		
		
		
/***************���������޹�*************/
		if(Start_flag == 13)//��ȥ���޹�
		{
				if(Step_flag==1)          //�ϲ��ջأ��Ƕ�ת�أ�T-->L
				{
            Count_Flag++;
            UpdateRoute_Arc(Count_Flag);
            if(Count_Flag == N300)
            {
                Step_flag = 2;
                Count_Flag = 0;
            }
        }
				if(Step_flag==2)          //L-->K��ɲ��
				{
            Count_Flag++;
            UpdateRoute_Arc(Count_Flag);
            if(Count_Flag == N20)
            {
                Step_flag = 3;
                Count_Flag = 0;
            }
        }
				if(Step_flag==3)          //�ϲ���ת��
				{
            Count_Flag++;
            UpdateRoute_Arc(Count_Flag);
            if(Count_Flag == N10)
            {
                Step_flag = 1;
                Count_Flag = 0;
							  Start_flag = 14;
            }
        }
		}
		
		if(Start_flag == 15)//ץ�޹ǣ��ߣ����޹�
		{
				if(Step_flag==1)          //ץȡ�޹�
        {
            Count_Flag++;
            UpdateRoute_Arc(Count_Flag);
            if(Count_Flag == N10)
            {
                Step_flag = 2;
                Count_Flag = 0;
            }
        }
        if(Step_flag==2)          //�ϲ㷭ת
        {
            Count_Flag++;
            UpdateRoute_Arc(Count_Flag);
            if(Count_Flag == N10)
            {
                Step_flag = 3;
                Count_Flag = 0;
            }
        }		
				if(Step_flag==3)          //Z-->R
        {
            Count_Flag++;
            UpdateRoute_Arc(Count_Flag);
            if(Count_Flag == N300)
            {
                Step_flag = 4;
                Count_Flag = 0;
            }
        }
        if(Step_flag==4)          //R-->T
        {
            Count_Flag++;
            UpdateRoute_Arc(Count_Flag);
            if(Count_Flag == N20)
            {
                Step_flag = 5;
                Count_Flag = 0;
            }
        }
        if(Step_flag==5)          //������ת
        {
            Count_Flag++;
            UpdateRoute_Arc(Count_Flag);
            if(Count_Flag == N20)
            {
                Step_flag = 6;
                Count_Flag = 0;
            }
        }
        if(Step_flag==6)          //װ��ǰ��
        {
            Count_Flag++;
            UpdateRoute_Arc(Count_Flag);
            if(Count_Flag == N10)
            {
                Step_flag = 7;
                Count_Flag = 0;
            }
        }
        if(Step_flag==7)          //�ɿ�צ��
        {
            Count_Flag++;
            UpdateRoute_Arc(Count_Flag);
            if(Count_Flag == N10)
            {
                Step_flag = 8;
                Count_Flag = 0;
            }
        }
        if(Step_flag==8)          //���޹�
        {
            Count_Flag++;
            UpdateRoute_Arc(Count_Flag);
            if(Count_Flag == N10)
            {
                Step_flag = 1;
                Count_Flag = 0;
                Start_flag = 16;
            }
        }				
		}
		
    if(Start_flag == 20)        //�س�
    {
        if(Step_flag==1)          //T-->Z
        {
            Count_Flag++;
            UpdateRoute_Arc(Count_Flag);
            if(Count_Flag == N300)
            {
                Step_flag = 2;
                Count_Flag = 0;
            }
        }
        if(Step_flag==2)          //Z-->Q
        {
            Count_Flag++;
            UpdateRoute_Arc(Count_Flag);
            if(Count_Flag == N300)
            {
                Step_flag = 3;
                Count_Flag = 0;
            }
        }
        if(Step_flag==3)          //Q-->O
        {
            Count_Flag++;
            UpdateRoute_Arc(Count_Flag);
            if(Count_Flag == N300)
            {
                Step_flag = 1;
                Count_Flag = 0;
                Start_flag =21;
            }
        }
    }

    PID_Angle_Loop_Test();    //�Ƕ�--PID����
    PID_LineX_Loop_Test();		//X��--PID����
    PID_LineY_Loop_Test();		//Y��--PID����
    SetMotorSpeed();
}
void Duty_20ms(void)
{
}
void Duty_50ms(void)
{

}
void Duty_100ms(void)
{

}
void Duty_200ms(void)
{
    LED1=!LED1;
}
void Duty_500ms(void)
{

}



