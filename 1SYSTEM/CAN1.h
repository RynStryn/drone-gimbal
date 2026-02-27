#ifndef __CAN1_H__
#define __CAN1_H__

#include "stm32f4xx.h"

void CAN1_Init(void);
void CAN1_Transmit(uint32_t ID, uint8_t Length, uint8_t *Data);
void CANx_Transmit(CAN_TypeDef *CANx,uint32_t ID, uint8_t Length, uint8_t *Data);
#endif
