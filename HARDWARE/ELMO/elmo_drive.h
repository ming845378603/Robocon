#ifndef __ELMO_DRIVE_H
#define __ELMO_DRIVE_H
#include "sys.h"

#define elmo1_id 0xF1
#define elmo2_id 0xF2
#define elmo3_id 0xF3
#define elmo4_id 0xF4

void ELMO_Delay(int8_t time);

void ELMO_Single_Enable(uint32_t elmo_id);
void ELMO_Single_Init(uint32_t elmo_id);
void ELMO_Single_Disenable(uint32_t elmo_id);
void ELMO_Single_STOP(uint32_t elmo_id);
void ELMO_Single_BEGIN(uint32_t elmo_id);
void ELMO_Single_Velocity(uint32_t elmo_id,int32_t v1);
void ELMO_Single_PTP_PA(uint32_t elmo_id,int32_t p1);
void ELMO_Single_PTP_PR(uint32_t elmo_id,int32_t p2);

void ELMO_Enable(void);
void ELMO_Init(void);
void ELMO_Disenable(void);
void ELMO_STOP(void);
void ELMO_BEGIN(void);
void ELMO_Velocity(int32_t v1,int32_t v2,int32_t v3,int32_t v4);

#endif


