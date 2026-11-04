#ifndef __MOTOR_DJI6020_H__
#define __MOTOR_DJI6020_H__
#include "stm32f4xx.h"



typedef struct
{
	float	pos_deg_last;
	float 	pos_deg;
	float	Init_deg;
    int32_t pos_turns;
	float	multi_turns_deg;//
    float 	speed_rpm;
	
	float 	I_ofTorq;//I_O?
	
	float	Temp;
	
	uint8_t Init_Flag;
	
}DJI6020_t;

extern DJI6020_t DJI6020_Yaw;

void DJI6020_Receive_Data(DJI6020_t* DJI6020_Yaw,uint8_t* Data);//

#endif
