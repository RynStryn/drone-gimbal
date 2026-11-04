#ifndef __USART1_H__
#define __USART1_H__
 
#include "stm32f4xx.h"
#define  BSP_USART1_DMA_RX_BUF_LEN                64u
#define  RC_FRAME_LENGTH                          18u
void USART1_Init(uint32_t baud_rate);

#endif 


