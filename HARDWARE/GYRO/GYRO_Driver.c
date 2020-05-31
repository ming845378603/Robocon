#include "GYRO_Lib.h"
#include "can.h"
#include "DataConverTypeDef.h"
#include "My_Flag.h"
#include "vect.h"
#include "My_Math.h"
#include "usart.h"
#include "Basal_Move.h"
#include "Time.h"
#include "PID_Control.h"
#include "Global.h"
#include "robot.h"
#include "usart3.h"
#include "Control_Arc.h"

__IO float GYRO_Angle_Last = 0;
__IO float GYRO_Angle_Now = 0;
__IO float GYRO_Angle = 0;
__IO float enc_x = 132/0.0392699081698724;
__IO float enc_y = -13/0.0392699081698724;
__IO int32_t GYRO_pulse_x = 0;
__IO int32_t GYRO_pulse_y = 0;
__IO int32_t enc_dy = 0;
__IO int32_t enc_dx = 0;
__IO EncodePointTypeDef GYRO_Location;

int DT35_DataA,DT35_DataB,DT35_DataC;//数据接收
int32_t	Makeup_Br = 0;int32_t Makeup_HandOver = 0;
extern s16 targetX, targetY1, targetA;//目标值
extern u8 Start_flag;
extern u8 Step_flag;
u8 res=0;
extern int32_t Stop_flag;
// 模块复位
static void GYRO_Reset_Module(void);
// GYRO初始化
//-----------------------------------------------------------------------------
void GYRO_Init(void)
{

    GYRO_Reset_Module();    //复位

    GYRO_Set_Angle(0);      //写入初始值
    GYRO_Set_Position(0, 0);
    delay_ms(20);  //每10ms更新一次
    GYRO_Set_Angle(0);      //写入初始值
    GYRO_Set_Position(0, 0);
    delay_ms(20);  //每10ms更新一次
    GYRO_Set_Angle(0);      //写入初始值
    GYRO_Set_Position(0, 0);
}


// GYRO_Set_Angle
// Inputs : float angle
// Outputs: void
// 设置角度
//-----------------------------------------------------------------------------
void GYRO_Set_Angle(float angle)
{
    uDataConvert32TypeDef temp;
    CanTxMsg TxMessage;
    temp.float_form = angle;
    TxMessage.StdId = POS_CID;				//控制器 ID
    TxMessage.DLC = 4;						//必须为 4
    TxMessage.IDE = CAN_Id_Standard;
    TxMessage.RTR = CAN_RTR_Data;
    TxMessage.Data[0] = temp.u8_form[0];
    TxMessage.Data[1] = temp.u8_form[1];
    TxMessage.Data[2] = temp.u8_form[2];
    TxMessage.Data[3] = temp.u8_form[3];
    CAN_Transmit(CAN_BUS, &TxMessage);
}

// GYRO_Set_tPosition
// Inputs : int32_t pos_x
//					int32_t pos_y
// Outputs: void
// 设置 x,y 坐标
//-----------------------------------------------------------------------------
void GYRO_Set_Position(int32_t pos_x, int32_t pos_y)
{
    CanTxMsg TxMessage;
    int32_t xx = 0, yy = 0;
    xx = -(int32_t)(pos_x* 12.8318);								//car_x 即为要发送的 x 坐标
    yy = -(int32_t)(pos_y* 12.5318);								//car_y 即为要发送的 y 坐标
    TxMessage.StdId = POS_CID;				//控制器 ID
    TxMessage.DLC = 8;//必须为 8
    TxMessage.IDE = CAN_Id_Standard;
    TxMessage.RTR = CAN_RTR_Data;
    TxMessage.Data[0] = (u8)(xx >> 0);
    TxMessage.Data[1] = (u8)(xx >> 8);
    TxMessage.Data[2] = (u8)(xx >> 16);
    TxMessage.Data[3] = (u8)(xx >> 24);
    TxMessage.Data[4] = (u8)(yy >> 0);
    TxMessage.Data[5] = (u8)(yy >> 8);
    TxMessage.Data[6] = (u8)(yy >> 16);
    TxMessage.Data[7] = (u8)(yy >> 24);
    CAN_Transmit(CAN_BUS, &TxMessage);
}

// GYRO_Reset_Module
// Inputs : void
// Outputs: void
// 模块软件复位
//-----------------------------------------------------------------------------
static void GYRO_Reset_Module(void)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = POS_CID;			//控制器 ID
    TxMessage.DLC = 2;					//必须为 2
    TxMessage.IDE = CAN_Id_Standard;
    TxMessage.RTR = CAN_RTR_Data;
    TxMessage.Data[0] = 0x55;
    TxMessage.Data[1] = 0xff;
    CAN_Transmit(CAN_BUS, &TxMessage);
}


/***************获得校正后的坐标值*******/
PointTypeDef GYRO_Get_Location(void)
{
	  PointTypeDef now_location; 	
	  now_location.Angle = GYRO_Location.Angle;
		now_location.Coords.x = (s32)(GYRO_Location.x); 
    now_location.Coords.y = (s32)(GYRO_Location.y);	
	
	  if(Start_flag==1)           //第一步
		{	 
  			 if(Step_flag==7)          //F-->M
				 {	
           Makeup_Br = - DT35_DataB + 1440 - (s32)(GYRO_Location.x);					 
				 }
				 if((Step_flag==12)||(Step_flag==13))        //W-->X 交接刹车
				 {
				   Makeup_Br = - DT35_DataB + 833 - (s32)(GYRO_Location.x) + 4500;
					 Makeup_HandOver = DT35_DataC - 275 -(s32)(GYRO_Location.y) + 7800;				 
				 }
					now_location.Angle = GYRO_Location.Angle;
					now_location.Coords.x = (s32)(GYRO_Location.x) + Makeup_Br; 
					now_location.Coords.y = (s32)(GYRO_Location.y) + Makeup_HandOver;			 
		 }
    else if(Start_flag==2)
		{
				   Makeup_Br = - DT35_DataB + 833 - (s32)(GYRO_Location.x) + 4500;
					 Makeup_HandOver = DT35_DataC - 275 -(s32)(GYRO_Location.y) + 7800;				 
					now_location.Angle = GYRO_Location.Angle;
					now_location.Coords.x = (s32)(GYRO_Location.x) + Makeup_Br; 
					now_location.Coords.y = (s32)(GYRO_Location.y) + Makeup_HandOver;		
		}	 
		else if(Start_flag==3)
		{
				 if(Step_flag==3)      
				 {
				   Makeup_Br = -DT35_DataB + 833 - (s32)(GYRO_Location.x) + 4500;
					 Makeup_HandOver = DT35_DataC - 475 -(s32)(GYRO_Location.y) + 8000;				 
				 }
					now_location.Angle = GYRO_Location.Angle;
					now_location.Coords.x = (s32)(GYRO_Location.x) + Makeup_Br; 
					now_location.Coords.y = (s32)(GYRO_Location.y) + Makeup_HandOver;					
		}
		else if(Start_flag==4)
		{
				  Makeup_Br = -DT35_DataB + 833 - (s32)(GYRO_Location.x) + 4500;
					Makeup_HandOver = DT35_DataC - 475 -(s32)(GYRO_Location.y) + 8000;				 
					now_location.Angle = GYRO_Location.Angle;
					now_location.Coords.x = (s32)(GYRO_Location.x) + Makeup_Br; 
					now_location.Coords.y = (s32)(GYRO_Location.y) + Makeup_HandOver;
		}
								
    return now_location;
}


#if defined(GYRO_INTERRUPT_INNER_ENABLE)

// Interrupt Function
// CAN1接收中断（暂没用到）
// CAN2（陀螺仪）接收中断
//-----------------------------------------------------------------------------
#if defined (GYRO_USE_CAN1)
    void CAN1_RX0_IRQHandler(void)       //使用CAN1 中断
#elif defined (GYRO_USE_CAN2)
    void CAN2_RX0_IRQHandler(void)       //使用CAN2 中断
#endif
    {
        uDataConvert32TypeDef temp;
        CanRxMsg RxMessage;
        if (CAN_GetITStatus(CAN_BUS, CAN_IT_FMP0) != RESET)
        {
            CAN_Receive(CAN_BUS, CAN_FIFO0, &RxMessage);
            switch (RxMessage.StdId)
            {
            case GYRO_ID:									//模块 ID 默认为 0x011
                if (RxMessage.DLC == 4)		//接收角度
                {
                    temp.u8_form[0] = RxMessage.Data[0];
                    temp.u8_form[1] = RxMessage.Data[1];
                    temp.u8_form[2] = RxMessage.Data[2];
                    temp.u8_form[3] = RxMessage.Data[3];
                    memcpy((void *)&GYRO_Angle_Now, &temp.float_form, 4);												
                }
            else if (RxMessage.DLC == 8)	//接收位置信息
            {
//	              temp.u8_form[0] = RxMessage.Data[0];
//                temp.u8_form[1] = RxMessage.Data[1];
//                temp.u8_form[2] = RxMessage.Data[2];
//                temp.u8_form[3] = RxMessage.Data[3];
//                memcpy((void *)&GYRO_pulse_x, &temp.s32_form, 4);

//                temp.u8_form[0] = RxMessage.Data[4];
//                temp.u8_form[1] = RxMessage.Data[5];
//                temp.u8_form[2] = RxMessage.Data[6];
//                temp.u8_form[3] = RxMessage.Data[7];
//                memcpy((void *)&GYRO_pulse_y, &temp.s32_form, 4);								
								
								//自己解算：无论你怎么解算你的坐标，首先让车原地转，xy都基本为零才对
   							 GYRO_pulse_y = TIM2->CNT;
                 enc_dy = GYRO_pulse_y - 30000;
                 GYRO_pulse_x = TIM5->CNT;
                 enc_dx = -(GYRO_pulse_x - 30000);
							
                TIM2->CNT = 30000;
                TIM5->CNT = 30000;
								
                GYRO_Angle = GYRO_Angle_Now / 2 + GYRO_Angle_Last / 2;
                enc_x = enc_x + (float)enc_dx * my_cos(GYRO_Angle) - (float)enc_dy * my_sin(GYRO_Angle);//- 94.46;
                enc_y = enc_y + (float)enc_dx * my_sin(GYRO_Angle) + (float)enc_dy * my_cos(GYRO_Angle);//- 224.62;

                GYRO_Location.x = (float)enc_x*Coder_Param - 132.6386f* my_cos(-GYRO_Angle + 5.6246f);
                GYRO_Location.y = (float)enc_y*Coder_Param + 132.6386f* my_sin(-GYRO_Angle + 5.6246f);
                GYRO_Angle_Last = GYRO_Angle_Now;
							
            }
                break;			
						case 0x13:		
								temp.u8_form[0] = RxMessage.Data[0];
								temp.u8_form[1] = RxMessage.Data[1];
								temp.u8_form[2] = RxMessage.Data[2];
								temp.u8_form[3] = RxMessage.Data[3];
								memcpy((void *)&DT35_DataA, &temp.s32_form, 4);
								break;								
						case 0x15:		
								temp.u8_form[0] = RxMessage.Data[0];
								temp.u8_form[1] = RxMessage.Data[1];
								temp.u8_form[2] = RxMessage.Data[2];
								temp.u8_form[3] = RxMessage.Data[3];
								memcpy((void *)&DT35_DataB, &temp.s32_form, 4);
								break;								
							case 0x16:		
								temp.u8_form[0] = RxMessage.Data[0];
								temp.u8_form[1] = RxMessage.Data[1];
								temp.u8_form[2] = RxMessage.Data[2];
								temp.u8_form[3] = RxMessage.Data[3];
								memcpy((void *)&DT35_DataC, &temp.s32_form, 4);
						    break;
							
//							case 0x20:
//                res=RxMessage.Data[0];
//							  if(res==0x07)                //电机急停
//								{
//										Stop_flag=1;
//										USART_SendData(USART3,0x00);	//上层电机急停
//								}
//								if((res==0x05)&&(Start_flag==0))             //开始任务
//								{
//										Start_flag = 1;
//								}
//								if((res==0x05)&&(Start_flag==2))
//								{
//										Start_flag = 3;
//								}
//								if((res==0x05)&&(Start_flag==4))
//								{
//										Start_flag = 5;
//								}
//								if((res==0x05)&&(Start_flag==6))
//								{
//										Start_flag = 7;
//								}
//								if((res==0x05)&&(Start_flag==8))
//								{
//										Start_flag = 9;
//								}
//								if((res==0x05)&&(Start_flag==10))
//								{
//										Start_flag = 11;
//								}
//								if((res==0x05)&&(Start_flag==12))
//								{
//										Start_flag = 13;
//								}
//								if((res==0x05)&&(Start_flag==14))
//								{
//										Start_flag = 15;
//								}
//								/***********微调***********/
//								if(res==0x03)
//								{
//										targetY1 = targetY1 + 10;
//										setup_PID(250,50,400,400,
//															250,50,400,400,
//															250,50,400,400);
//								}
//								if(res==0x06)
//								{
//										targetY1 = targetY1 - 10;
//										setup_PID(250,50,400,400,
//															250,50,400,400,
//															250,50,400,400);
//								}
//								if(res==0x04)
//								{
//										targetX = targetX - 10;
//										setup_PID(250,50,400,400,
//															250,50,400,400,
//															250,50,400,400);
//								}
//								if(res==0x10)
//								{
//										targetX = targetX + 10;
//										setup_PID(250,50,400,400,
//															250,50,400,400,
//															250,50,400,400);
//								}
//								/************回原点**************/
//								if(res==0x08)            //回原点
//								{
//										Start_flag = 20;
//										Stop_flag=0;
//								}				
//								break;
            default:
                break;
            }
						
//						GYRO_Location.x = (float)GYRO_pulse_x*Coder_Param;//看好这个xy的方向和符号
//            GYRO_Location.y = (float)GYRO_pulse_y*Coder_Param;
            GYRO_Location.Angle = GYRO_Angle_Now;
						
            nloc = GYRO_Get_Location();//通过外界传感器进行坐标矫正
						
            CAN_ClearITPendingBit(CAN_BUS, CAN_IT_FMP0);
        } 
    }
#endif
