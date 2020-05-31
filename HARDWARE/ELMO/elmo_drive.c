#include "elmo_drive.h" 
#include "can.h"
#include "delay.h"


#define elmo_delay 200


//unsigned int CAN_Time_Out = 0;
//unsigned char can_tx_success_flag = 0;

//static void CAN_Delay_Us(unsigned int t)
//{
//	int i;
//	for(i=0;i<t;i++)
//	{
//		int a=40;
//		while(a--);
//	}
//}


//void ELMO_Delay(int8_t time)
//{
//	int16_t i;
//	for (; time>0; time--)
//	{
//		for (i = 0; i<50; i++);
//	}
//}


void ELMO_Single_Enable(uint32_t elmo_id)
{
    uint32_t can_id = 0x000;
	
    CanTxMsg Tx_message;
    
//    Tx_message.IDE = CAN_ID_STD;    //标准帧
//    Tx_message.RTR = CAN_RTR_DATA;  //数据帧
//    Tx_message.DLC = 0x08;          //帧长度为8

	can_id = elmo_id;
    
//    Tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID
    
    Tx_message.Data[0] = 0xE0;
    Tx_message.Data[1] = 0x55;
    Tx_message.Data[2] = 0x55;
    Tx_message.Data[3] = 0x55;
    Tx_message.Data[4] = 0x55;
    Tx_message.Data[5] = 0x55;
    Tx_message.Data[6] = 0x55;
    Tx_message.Data[7] = 0x0E;
    
//    can_tx_success_flag = 0;
//    CAN_Transmit(CAN1,&Tx_message);
//    
//    CAN_Time_Out = 0;
//    while(can_tx_success_flag == 0)
//    {
//        CAN_Delay_Us(1);
//        CAN_Time_Out++;
//        if(CAN_Time_Out>100)
//        {
//            break;
//        }
//    }
	CAN1_SendMsg(Tx_message.Data,can_id);
}


void ELMO_Single_Init(uint32_t elmo_id)
{
    uint32_t can_id = 0x000;
	
    CanTxMsg Tx_message;
    
//    Tx_message.IDE = CAN_ID_STD;    //标准帧
//    Tx_message.RTR = CAN_RTR_DATA;  //数据帧
//    Tx_message.DLC = 0x08;          //帧长度为8

	can_id = elmo_id;
    
//    Tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID
    
    Tx_message.Data[0] = 0xE1;
    Tx_message.Data[1] = 0x55;
    Tx_message.Data[2] = 0x55;
    Tx_message.Data[3] = 0x55;
    Tx_message.Data[4] = 0x55;
    Tx_message.Data[5] = 0x55;
    Tx_message.Data[6] = 0x55;
    Tx_message.Data[7] = 0x1E;
    
//    can_tx_success_flag = 0;
//    CAN_Transmit(CAN1,&Tx_message);
//    
//    CAN_Time_Out = 0;
//    while(can_tx_success_flag == 0)
//    {
//        CAN_Delay_Us(1);
//        CAN_Time_Out++;
//        if(CAN_Time_Out>100)
//        {
//            break;
//        }
//    }
	CAN1_SendMsg(Tx_message.Data,can_id);
}


void ELMO_Single_Disenable(uint32_t elmo_id)
{
    uint32_t can_id = 0x000;
	
    CanTxMsg Tx_message;
    
//    Tx_message.IDE = CAN_ID_STD;    //标准帧
//    Tx_message.RTR = CAN_RTR_DATA;  //数据帧
//    Tx_message.DLC = 0x08;          //帧长度为8

	can_id = elmo_id;
    
//    Tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID
    
    Tx_message.Data[0] = 0xE2;
    Tx_message.Data[1] = 0x55;
    Tx_message.Data[2] = 0x55;
    Tx_message.Data[3] = 0x55;
    Tx_message.Data[4] = 0x55;
    Tx_message.Data[5] = 0x55;
    Tx_message.Data[6] = 0x55;
    Tx_message.Data[7] = 0x2E;
    
//    can_tx_success_flag = 0;
//    CAN_Transmit(CAN1,&Tx_message);
//    
//    CAN_Time_Out = 0;
//    while(can_tx_success_flag == 0)
//    {
//        CAN_Delay_Us(1);
//        CAN_Time_Out++;
//        if(CAN_Time_Out>100)
//        {
//            break;
//        }
//    }
	CAN1_SendMsg(Tx_message.Data,can_id);
}


void ELMO_Single_STOP(uint32_t elmo_id)
{
    uint32_t can_id = 0x000;
	
    CanTxMsg Tx_message;
    
//    Tx_message.IDE = CAN_ID_STD;    //标准帧
//    Tx_message.RTR = CAN_RTR_DATA;  //数据帧
//    Tx_message.DLC = 0x08;          //帧长度为8

	can_id = elmo_id;
    
//    Tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID
    
    Tx_message.Data[0] = 0xE3;
    Tx_message.Data[1] = 0x55;
    Tx_message.Data[2] = 0x55;
    Tx_message.Data[3] = 0x55;
    Tx_message.Data[4] = 0x55;
    Tx_message.Data[5] = 0x55;
    Tx_message.Data[6] = 0x55;
    Tx_message.Data[7] = 0x3E;
    
//    can_tx_success_flag = 0;
//    CAN_Transmit(CAN1,&Tx_message);
//    
//    CAN_Time_Out = 0;
//    while(can_tx_success_flag == 0)
//    {
//        CAN_Delay_Us(1);
//        CAN_Time_Out++;
//        if(CAN_Time_Out>100)
//        {
//            break;
//        }
//    }
	CAN1_SendMsg(Tx_message.Data,can_id);
}


void ELMO_Single_BEGIN(uint32_t elmo_id)
{
    uint32_t can_id = 0x000;
	
    CanTxMsg Tx_message;
    
//    Tx_message.IDE = CAN_ID_STD;    //标准帧
//    Tx_message.RTR = CAN_RTR_DATA;  //数据帧
//    Tx_message.DLC = 0x08;          //帧长度为8

	can_id = elmo_id;
    
//    Tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID
    
    Tx_message.Data[0] = 0xE4;
    Tx_message.Data[1] = 0x55;
    Tx_message.Data[2] = 0x55;
    Tx_message.Data[3] = 0x55;
    Tx_message.Data[4] = 0x55;
    Tx_message.Data[5] = 0x55;
    Tx_message.Data[6] = 0x55;
    Tx_message.Data[7] = 0x4E;
    
//    can_tx_success_flag = 0;
//    CAN_Transmit(CAN1,&Tx_message);
	CAN1_SendMsg(Tx_message.Data,can_id);
 
}


void ELMO_Single_Velocity(uint32_t elmo_id,int32_t v1)
{
    uint32_t can_id = 0x000;
	
    CanTxMsg Tx_message;
    
//    Tx_message.IDE = CAN_ID_STD;    //标准帧
//    Tx_message.RTR = CAN_RTR_DATA;  //数据帧
//    Tx_message.DLC = 0x08;          //帧长度为8

	can_id = elmo_id;
    
//    Tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID
    
    Tx_message.Data[0] = 0xE5;
//    Tx_message.Data[1] = 0x55;
//    Tx_message.Data[2] = 0x55;
    Tx_message.Data[3] = 0x55;
    Tx_message.Data[4] = 0x55;
    Tx_message.Data[5] = 0x55;
    Tx_message.Data[6] = 0x55;
    Tx_message.Data[7] = 0x5E;
	
	if(v1<0)
    {
        v1=-v1;
        v1=v1|0x8000;
        Tx_message.Data[1] = (u8)((v1&0xff00) >> 8);
        Tx_message.Data[2] = (u8)(v1&0x00ff);
    }
    else
    {
        Tx_message.Data[1] = (u8)((v1&0xff00) >> 8);
        Tx_message.Data[2] = (u8)(v1&0x00ff);
    }
    
//    can_tx_success_flag = 0;
//    CAN_Transmit(CAN1,&Tx_message);
//    
//    CAN_Time_Out = 0;
//    while(can_tx_success_flag == 0)
//    {
//        CAN_Delay_Us(1);
//        CAN_Time_Out++;
//        if(CAN_Time_Out>100)
//        {
//            break;
//        }
//    }
	CAN1_SendMsg(Tx_message.Data,can_id);
}


void ELMO_Single_PTP_PA(uint32_t elmo_id,int32_t p1)
{
    uint32_t can_id = 0x000;
	
    CanTxMsg Tx_message;
    
//    Tx_message.IDE = CAN_ID_STD;    //标准帧
//    Tx_message.RTR = CAN_RTR_DATA;  //数据帧
//    Tx_message.DLC = 0x08;          //帧长度为8

	can_id = elmo_id;
    
//    Tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID
    
    Tx_message.Data[0] = 0xE6;
//    Tx_message.Data[1] = 0x55;
//    Tx_message.Data[2] = 0x55;
//    Tx_message.Data[3] = 0x55;
//    Tx_message.Data[4] = 0x55;
    Tx_message.Data[5] = 0x55;
    Tx_message.Data[6] = 0x55;
    Tx_message.Data[7] = 0x6E;
	
	if(p1<0)
    {
        p1=-p1;
        p1=p1|0x80000000;
        Tx_message.Data[1] = (u8)((p1&0xff000000) >> 24);
        Tx_message.Data[2] = (u8)((p1&0x00ff0000) >> 16);
		Tx_message.Data[3] = (u8)((p1&0x0000ff00) >> 8);
		Tx_message.Data[4] = (u8)(p1&0x000000ff);
    }
    else
    {
        Tx_message.Data[1] = (u8)((p1&0xff000000) >> 24);
        Tx_message.Data[2] = (u8)((p1&0x00ff0000) >> 16);
		Tx_message.Data[3] = (u8)((p1&0x0000ff00) >> 8);
		Tx_message.Data[4] = (u8)(p1&0x000000ff);
    }
    
//    can_tx_success_flag = 0;
//    CAN_Transmit(CAN1,&Tx_message);
//    
//    CAN_Time_Out = 0;
//    while(can_tx_success_flag == 0)
//    {
//        CAN_Delay_Us(1);
//        CAN_Time_Out++;
//        if(CAN_Time_Out>100)
//        {
//            break;
//        }
//    }
	CAN1_SendMsg(Tx_message.Data,can_id);
}


void ELMO_Single_PTP_PR(uint32_t elmo_id,int32_t p2)
{
    uint32_t can_id = 0x000;
	
    CanTxMsg Tx_message;
    
//    Tx_message.IDE = CAN_ID_STD;    //标准帧
//    Tx_message.RTR = CAN_RTR_DATA;  //数据帧
//    Tx_message.DLC = 0x08;          //帧长度为8

	can_id = elmo_id;
    
//    Tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID
    
    Tx_message.Data[0] = 0xE7;
//    Tx_message.Data[1] = 0x55;
//    Tx_message.Data[2] = 0x55;
//    Tx_message.Data[3] = 0x55;
//    Tx_message.Data[4] = 0x55;
    Tx_message.Data[5] = 0x55;
    Tx_message.Data[6] = 0x55;
    Tx_message.Data[7] = 0x7E;
	
	if(p2<0)
    {
        p2=-p2;
        p2=p2|0x80000000;
        Tx_message.Data[1] = (u8)((p2&0xff000000) >> 24);
        Tx_message.Data[2] = (u8)((p2&0x00ff0000) >> 16);
		Tx_message.Data[3] = (u8)((p2&0x0000ff00) >> 8);
		Tx_message.Data[4] = (u8)(p2&0x000000ff);
    }
    else
    {
        Tx_message.Data[1] = (u8)((p2&0xff000000) >> 24);
        Tx_message.Data[2] = (u8)((p2&0x00ff0000) >> 16);
		Tx_message.Data[3] = (u8)((p2&0x0000ff00) >> 8);
		Tx_message.Data[4] = (u8)(p2&0x000000ff);
    }
    
//    can_tx_success_flag = 0;
//    CAN_Transmit(CAN1,&Tx_message);
//    
//    CAN_Time_Out = 0;
//    while(can_tx_success_flag == 0)
//    {
//        CAN_Delay_Us(1);
//        CAN_Time_Out++;
//        if(CAN_Time_Out>100)
//        {
//            break;
//        }
//    }
	CAN1_SendMsg(Tx_message.Data,can_id);
}


void ELMO_Enable(void)
{
	ELMO_Single_Enable(elmo1_id);
	delay_us(elmo_delay);
	ELMO_Single_Enable(elmo2_id);
	delay_us(elmo_delay);
	ELMO_Single_Enable(elmo3_id);
	delay_us(elmo_delay);
	ELMO_Single_Enable(elmo4_id);
	delay_us(elmo_delay);
}


void ELMO_Init(void)
{
	ELMO_Single_Init(elmo1_id);
	delay_us(elmo_delay);
	ELMO_Single_Init(elmo2_id);
	delay_us(elmo_delay);
	ELMO_Single_Init(elmo3_id);
	delay_us(elmo_delay);
	ELMO_Single_Init(elmo4_id);//后期需加delay
	delay_us(elmo_delay);
}


void ELMO_Disenable(void)
{
	ELMO_Single_Disenable(elmo1_id);
	delay_us(elmo_delay);
	ELMO_Single_Disenable(elmo2_id);
	delay_us(elmo_delay);
	ELMO_Single_Disenable(elmo3_id);
	delay_us(elmo_delay);
	ELMO_Single_Disenable(elmo4_id);
	delay_us(elmo_delay);
}


void ELMO_STOP(void)
{
	ELMO_Single_STOP(elmo1_id);
	delay_us(elmo_delay);
	ELMO_Single_STOP(elmo2_id);
	delay_us(elmo_delay);
	ELMO_Single_STOP(elmo3_id);
	delay_us(elmo_delay);
	ELMO_Single_STOP(elmo4_id);	
	delay_us(elmo_delay);
}


void ELMO_BEGIN(void)
{
	ELMO_Single_BEGIN(elmo1_id);
	delay_us(elmo_delay);
	ELMO_Single_BEGIN(elmo2_id);
	delay_us(elmo_delay);
	ELMO_Single_BEGIN(elmo3_id);
	delay_us(elmo_delay);
	ELMO_Single_BEGIN(elmo4_id);	
	delay_us(elmo_delay);
}


void ELMO_Velocity(int32_t v1,int32_t v2,int32_t v3,int32_t v4)
{
	ELMO_Single_Velocity(elmo1_id,v1);	
	delay_us(elmo_delay);
	ELMO_Single_Velocity(elmo2_id,v2);	
	delay_us(elmo_delay);
	ELMO_Single_Velocity(elmo3_id,v3);	
	delay_us(elmo_delay);
	ELMO_Single_Velocity(elmo4_id,v4);
	delay_us(elmo_delay);
}






