//-----------------------------------------------------------------------------
// PID_Parameter.c
//
//  Created on	: 2017-9-15
//      Author	: 李彪
//		version	: V1.0
//		brief	:
//-----------------------------------------------------------------------------
// Attention:
//

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include "PID_Parameter.h"
#include <stdint.h>
#include "Vect.h"

////PID环参数:Kp Ki Kd Out_Max -Out_Max Iteg_Max Dead_Zone A B  2017-10-29数据p500/100  d100/100
//sPID_TypeDef G_PID_Parameter = {
//    /*角度环*/
//    {   { 14000,0,500,800,-800,800,0,30,20 }
//        ,{0,0,0,0,0}
//    },
//    /*位置环y*/
//    {   { 500,0,120,1000,-1000,1000,0,30,20 }
//        ,{0,0,0,0,0}
//    },
//    /*位置环x*/
//    {   { 1,0,200,2000,-2000,1000,0,30,20 }
//        ,{0,0,0,0,0}
//    }
//};

sPID_TypeDef G_PID_Parameter[7] =  {
		/*普通角度环*/
	{////PID环参数:Kp Ki Kd Out_Max -Out_Max Iteg_Max Dead_Zone A B 
		{ { 10000,0,5,1000,-1000,800,0,30,20 }
		,{ 0,0,0,0,0 }
		},
	/*位置环x*/
	{ { 200,0,50,1000,-1000,1000,0,30,20 }
	,{ 0,0,0,0,0 }
	},
	/*位置环y*/
	{ { 350,0,150,1000,-1000,1000,0,30,20 }
	,{ 0,0,0,0,0 }
	}
	},
	{/*巡线X角度环*///
	//PID环参数:Kp Ki Kd Out_Max -Out_Max Iteg_Max Dead_Zone A B  
		{ { 10000,0,50,2000,-2000,800,0,30,20 }
		,{ 0,0,0,0,0 }
		},
	/*位置环x*/
		{ { 500,0,50,2000,-2000,1000,0,30,20 }
	,{ 0,0,0,0,0 }
		},
	/*位置环y*/
		{ { 650,0,50,2000,-2000,1000,0,30,20 }
	,{ 0,0,0,0,0 }
		}
	},
	{/*巡线摄像头Y角度环*///
	//PID环参数:Kp Ki Kd Out_Max -Out_Max Iteg_Max Dead_Zone A B  
		{ { 10000,0,50,2000,-2000,800,0,30,20 }
		,{ 0,0,0,0,0 }
		},
	/*位置环x*/
		{ { 550,0,50,2000,-2000,1000,0,30,20 }
	,{ 0,0,0,0,0 }
		},
	/*位置环y*/
		{ { 650,0,50,2000,-2000,1000,0,30,20 }
	,{ 0,0,0,0,0 }
		}
	}
};

//#define SPEED_L
//#define SPEED_M
#define SPEED_H
/*前馈参数*/
#if defined(SPEED_L)
int32_t g_Speed[4] = { 1000,0,1000,1000 };    /*设置速度:unif end aclt decr，单位: mm/s*/
int32_t forwardParam[7] = { 170,1,2,0,0,2000,-2000 };	/*Kf Kp_a Kp_d Kd_a Kd_d Max Min*/
#elif defined(SPEED_M)
int32_t g_Speed[4] = { 2000,0,1000,1000 };    /*设置速度:unif end aclt decr，单位: mm/s*/
int32_t forwardParam[7] = { 170,1,2,0,0,3000,-3000 };	/*Kf Kp_a Kp_d Kd_a Kd_d Max Min*/
#elif defined(SPEED_H)
int32_t X_Speed[4] = { 4000,0,1750,3000 };//这个x方向的运动加速的时候比较抖，所以就加速度给的小点，减速度给大点，因为刹车的时候不怎么抖，这样可以让匀速时间久点
int32_t Y_Speed[4] = { 4500,0,3000,3000 };      /*设置速度:unif end aclt decr，单位: mm/s*/
int32_t X_forwardParam[7] = { 40,2,10,1,20,4000,-4000 };	/*Kf Kp_a Kp_d Kd_a Kd_d Max Min*/
int32_t Y_forwardParam1[7] = { 60,6,0,0,0,4500,-4500 };	/*Kf Kp_a Kp_d Kd_a Kd_d Max Min*/
int32_t Y_forwardParam2[7] = { 55,0,0,0,0,4500,-4500 };	/*Kf Kp_a Kp_d Kd_a Kd_d Max Min*/
									  //int32_t forwardParam[7] = { 321,2,4,0,0,6000,-6000 };	/*Kf Kp_a Kp_d Kd_a Kd_d Max Min*/2017
int32_t L_Speed[4] = { 4500,0,2500,2500 };
int32_t H_Speed[4] = { 4500,0,2500,2500 };      /*设置速度:unif end aclt decr，单位: mm/s*/
int32_t forwardParam[7] = { 40,1,1,1,1,5000,-5000 };	/*Kf Kp_a Kp_d Kd_a Kd_d Max Min*/
//int32_t forwardParam[7] = { 321,2,4,0,0,6000,-6000 };	/*Kf Kp_a Kp_d Kd_a Kd_d Max Min*/2017
#endif

/*交叉点位置参数*/
/* 红： 启动 橙 紫 绿 蓝 红 灰 粉 装盘
   蓝： 启动 橙 紫 蓝 红 绿 红 粉 装盘*/
int32_t Junction_x[][9] = {
    { 0, 0, 0, 0, 260, 0, 0, 0, 0 },    //红场数据
    { 0, 0, 0, 0, 210, 0, 0, 0, 0 },   //蓝场数据
};
int32_t Junction_y[][9] = {
    { 0, 1540, 3585, 5655, 5920, 6625, 7500, 9485, 13600 },    //红场数据
    { 0,-4140,-5960,-6890,-7950,-7950,-9950,-11950,-13350 },   //蓝场数据
};

/*Line Adjust时速度*/
int32_t velocity = 320;


/*前馈参数计算*/
/*麦克纳姆轮周长mm                         150 * 3.1415926
电机减速比                                 12.0(21.0)
车轮的最大转速r/min                        7200
车实际速度v和转速out的转化，v是mm/s        v =(out / 12(21.0) ) * 150 * 3.1415926 / 60

误差mm        dist
周期ms        t
预期速度mm/s  v
v = dist / (t / 1000);

Kf = out / dist
这样算出来的理论值应该是152.7887479(267.380308825)
*/

//L_Bill
/*前馈参数计算*/
/*麦克纳姆轮周长mm                         125 * 3.1415926
电机减速比                                 (21.0)
车轮的最大转速r/min                        9030
车实际速度v和转速out的转化，v是mm/s        v =(out / (21.0) ) * 125 * 3.1415926 / 60    out=3.21*v

误差mm        dist
周期ms        t
预期速度mm/s  v    就是给下一次的速度赋值
v = dist / (t / 1000);    v = dist*Kf;           

Kf = dist / v
这样算出来的理论值应该是100
*/

/******************* (C) COPYRIGHT 2017 L_Bill ***********END OF FILE***********/
