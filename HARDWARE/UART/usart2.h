#include "stm32f4xx.h"

extern int32_t Stop_flag;
void u2_putbuff(u8 *buff, u32 len);
void usart2_init(u32 bound);
