#ifndef __GIMBAL_H__
#define __GIMBAL_H__

#include "stm32f4xx.h"

#define Yaw_Angle_MAX 50
#define Yaw_Angle_MIN -50
#define Pitch_Angle_MAX 24
#define Pitch_Angle_MIN -13

typedef enum
{
	GIMBAL_INIT       = 0 ,//³õÊ¼»¯
	GIMBAL_STOP           ,//Í£Ö¹
	GIMBAL_RELAX          ,//·ÅËÉ
	GIMBAL_REMOTE  	      ,//Ò£¿ØÆ÷
	GIMBAL_KM_NORMAL      ,//¼üÊó
} Gimbal_Mode_e;

typedef struct
{
	float Yaw_Angle_Chassis;
	float Pitch_Angle_Chassis;
	
	float Yaw_Angle_Mid;
	float Pitch_Angle_Mid;
	
	float Yaw_Angle_Set;
	float Pitch_Angle_Set;
	
	float Yaw_Angle_Actual;
	float Pitch_Angle_Actual;
	
	float Yaw_Speed_Set;
	float Pitch_Speed_Set;
	
	float Yaw_Speed_Act;
	float Pitch_Speed_Act;
	
	Gimbal_Mode_e Mode;
	Gimbal_Mode_e Mode_Last;
	
	uint8_t Flag_Init_Finish;
	
}Gimbal_t;

extern Gimbal_t Gimbal;

void Gimbal_PID_Init(void);
void DJI6020_Yaw_Control(float Angle);
void DM4310_Pitch_Control(float Angle);
void Gimbal_Set_Inc(Gimbal_t *Gimbal,float Yaw,float Pitch);
void Gimbal_Control_Task_Inc(Gimbal_t *Gimbal,float Yaw,float Pitch);
void Gimbal_Get_Angle_Data(Gimbal_t *Gimbal,CH040_t *CH040_Data,DJI6020_t* DJI6020_Yaw,DM4310_t* DM4310_Pitch);
void Gimbal_PID_Clear(void);
#endif
