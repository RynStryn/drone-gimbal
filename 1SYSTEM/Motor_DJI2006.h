#ifndef __MOTOR_DJI2006_H__
#define __MOTOR_DJI2006_H__
#include "stm32f4xx.h"

#define DJI2006_Poke_ID 	0x204
#define DJI2006_Fric_ID_1 	0x201
#define DJI2006_Fric_ID_2 	0x202
#define DJI2006_Fric_ID_3 	0x203

typedef struct
{
	float	pos_degs_last;
	float 	pos_degs;
    int32_t pos_turns;
	float	mult_degs;
	
	float 	out_degs;
	
    float 	speed_degs;
    float 	speed_rpm;

	float   N_m;
	
	uint32_t ID;
	
}DJI2006_t;

extern DJI2006_t DJI2006_Pock;
extern DJI2006_t DJI2006_Fric[3];

void DJI2006_Receive_Data(DJI2006_t* My_Motor,uint8_t* DATA);
void DJI2006_Receive_Reversed_Data(DJI2006_t* My_Motor,uint8_t* DATA);

#endif
