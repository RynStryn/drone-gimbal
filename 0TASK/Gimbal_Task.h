#ifndef __GIMBAL_H__
#define __GIMBAL_H__

#include "stm32f4xx.h"
#include "LESO.h"

#define Yaw_Angle_CCW 80
#define Yaw_Angle_CW -260
//dm pos;-30~18
//pitch get;-23~24
#define Pitch_Angle_MIN_rev 25
#define Pitch_Angle_MAX_rev -15//

typedef enum
{
    GIMBAL_OFF  =0,
	GIMBAL_INIT        ,//³õÊ¼»¯
	GIMBAL_STOP           ,//Í£Ö¹
	GIMBAL_RELAX          ,//·ÅËÉ
	GIMBAL_RC  	      ,//Ò£¿ØÆ÷
	GIMBAL_KeyMouse_NORMAL      ,//¼üÊó

} eGimbal_Mode;

typedef struct
{
	float Yaw_Angle_Chassis;
	float Pitch_Angle_Chassis;
	
	float Yaw_Angle_Mid;
	float Pitch_Angle_Mid;
	
	float Yaw_Angle_Set;
	float Pitch_Angle_Set;
	
	float Yaw_Angle_Get;//deg
	float Pitch_Angle_Get;
	
	float Yaw_Speed_Set;
	float Pitch_Speed_Set;
	
	float Yaw_Speed_Get;
	float Pitch_Speed_Get;
	
	eGimbal_Mode Mode;
	eGimbal_Mode Mode_Last;

	gimbal_ESO_t 	Yaw_LESO;
	gimbal_ESO_t 	Pitch_LESO;
    uint8_t power_on;
}Gimbal_t;

extern Gimbal_t Gimbal;

void Gimbal_Ctrler_Init(void);
void DJI6020_Yaw_CalcSend(float Angle);
void DM4310_Pitch_Control(float Angle);
void Gimbal_Set_Inc(Gimbal_t *Gimbal,float Yaw_inc,float Pitch_inc);
void Gimbal_Control_Task_Inc(Gimbal_t *Gimbal,float Yaw_inc,float Pitch_inc);
void Gimbal_Ang_Spd_Get(Gimbal_t *Gimbal,CH040_t *CH040_Data,DJI6020_t* DJI6020_Yaw,DM4310_t* DM4310_Pitch);
void Gimbal_PID_Clear(void);
void Gimbal_Homing(void);

#endif
