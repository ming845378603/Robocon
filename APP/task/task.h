#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "My_Math.h"
#include <stdint.h>
#include "Timer.h"

typedef struct
{
	u8 check_flag;
	u16 err_flag;
	s16 cnt_10ms;
	s16 cnt_20ms;
	s16 cnt_50ms;
	s16 cnt_100ms;
	s16 cnt_200ms;
	s16 cnt_500ms;
	u16 time;
}loop_t;

void Loop_check(void);

void main_loop(void);
void Duty_10ms(void);
void Duty_20ms(void);
void Duty_50ms(void);
void Duty_100ms(void);
void Duty_200ms(void);
void Duty_500ms(void);

