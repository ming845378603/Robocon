#include "sys.h"
#include "Timer.h"
#include "Time.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "beep.h"
#include "key.h"
#include "My_BSP.h"
#include "Basal_Move.h"
#include "GYRO_Lib.h"
#include "My_Flag.h"
#include "PID_Control.h"
#include "Robot.h"
#include "ANO_DT.h"
#include "Time.h"
#include "Route_Control.h"
/******************************************************************************\
* Function Name  : TIM7_Init
* Input          : None
* Output         : None
* Return         : None
* Description    : TIM7初始化，用于GYRO的定时计算
\******************************************************************************/
void TIM7_Init(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

    //定时周期T=(Period+1)*(Prescaler+1)/TCLK=200us
    TIM_TimeBaseStructure.TIM_Period = 200-1;
    TIM_TimeBaseStructure.TIM_Prescaler = 8400 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);

    NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_ClearFlag(TIM7, TIM_FLAG_Update);
    TIM_ITConfig(TIM7, TIM_IT_Update | TIM_IT_Trigger, ENABLE);
    TIM_Cmd(TIM7, ENABLE);
}

