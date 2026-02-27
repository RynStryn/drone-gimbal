#ifndef __MOTOR_DJI6020_H__
#define __MOTOR_DJI6020_H__
#include "stm32f4xx.h"

#define DJI6020_Yaw_ID 0x205
#define DJI6020_Yaw_Send_ID 0x1FE

typedef struct
{
	float	pos_degs_last;
	float 	pos_degs;
	float	Init_degs;
    int32_t pos_turns;
	float	mult_degs;
    float 	speed_rpm;
	
	float 	I_O;
	
	float	Temp;
	
	uint8_t Init_Flag;
	
}DJI6020_t;

extern DJI6020_t DJI6020_Yaw;

void DJI6020_Receive_Data(DJI6020_t* My_Motor,uint8_t* Data);

#endif
