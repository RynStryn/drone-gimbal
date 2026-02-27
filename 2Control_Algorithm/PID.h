#ifndef __PID_H__
#define __PID_H__

#include "stm32f4xx.h"

typedef enum
{
	Position_PID=0,
	Delta_PID,
}PID_Mode;
typedef enum
{
	Disable=0,
	Enable,
}DOF;
//									PID类型			微分先行	输出限幅	积分限幅	Kp			Ki			Kd
#define DJI6020_PID_V_Init 			Position_PID,	Disable,	13000,		200,		78,			2,			100
#define DJI6020_PID_P_Init 			Position_PID,	Disable,	1200,		40,			38,			0,			40

#define DM4310_PID_V_Init			Position_PID,	Disable,	18,			0,			0,			0,			0
#define DM4310_PID_P_Init			Position_PID,	Disable,	1000,		0,			0,			0,			0

#define DJI2006_Poke_PID_V_Init		Position_PID,	Disable,	4000,		1000,		12,			0.5,		2
#define DJI2006_Poke_PID_P_Init		Position_PID,	Disable,	8000,		200,		3,			0.01,		1

#define DJI2006_Fric_1_PID_V_Init	Position_PID,	Disable,	4000,		580,		12,			0.9,		9
#define DJI2006_Fric_2_PID_V_Init	Position_PID,	Disable,	4000,		580,		12,			0.9,		9.4
#define DJI2006_Fric_3_PID_V_Init	Position_PID,	Disable,	4000,		600,		15,			2,			15

typedef struct{
	float 	Kp;
	float 	Ki;
	float 	Kd;
	
	float 	Set;
	float 	Get[2];
	float 	Err[3];
	
	float 	Pout;
	float 	Iout;
	float 	Dout;
	
	float 	Out;
	
	float   Inc_Max;
	float 	Out_Max;
 PID_Mode 	PID_Mode;
	
	DOF		DOF_Flag;
	
}pid_t;

extern	pid_t 	DJI6020_PID_V,		  DJI6020_PID_P,
				DM4310_PID_V,		  DM4310_PID_P,
				DJI2006_Poke_PID_V,	  DJI2006_Poke_PID_P,
				DJI2006_Fric_PID_V_1, DJI2006_Fric_PID_V_2,	DJI2006_Fric_PID_V_3;

void PID_Cal(pid_t* pid,float Set,float Get);
void PID_StructInit(
	 pid_t 		*pid,
	 PID_Mode 	Mode,
	 DOF		Flag,
	 float 		Out_Max,
	 float 		Inc_Max,

	 float 		kp,
	 float		ki,
	 float 		kd);
void PID_Clear(pid_t* pid);

#endif
