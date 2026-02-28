#ifndef __SHOOT_TASK_H__
#define __SHOOT_TASK_H__

#include "stm32f4xx.h"

#define BULLET_SPEED_TARGET 			24
#define BULLET_SPEED_SELF_ADAPTATION_K  10000
#define FRICTION_SPEED					9100
#define FRICTION_SPEED_MIN				3000

typedef enum
{
	Friction_ON=0,
	Friction_OFF,
}Friction_State;

typedef enum
{
	POKE_ON=0,
	POKE_OFF,
}Poke_State;

typedef enum
{
	SHOOTER_INIT=0,
	FullAuto_FIRE,
	SINGLE_SHOT,
	CLOSE,
}Shooter_Mode;

typedef enum
{
	STOP=0,
	START,
	Poke_Get_Trap,
	Friction_Get_Trap,
}Shooter_State;

typedef struct
{
	float Error_Mea;
	float Error_Est;
	float Error_Est_Last;
	float Kalman_Gain;
	float X_hat;
	float X_hat_Last;
} First_Order_Kalman_Filter_t;

typedef struct
{
	Friction_State	Friction_State;
	
	Poke_State 		Poke_State;
	Shooter_State 	State;
	Shooter_Mode 	Mode;
	Shooter_Mode 	Mode_Last;
	
	float Friction_Get_Trap_Cnt;
	
	int16_t Poke_Get_Trap_cnt;

	uint8_t switch_Single_Mode_Flag;
	uint8_t switch_Single_Mode_Flag_Finish;
	uint8_t Init_Flag;
	
	float Freq;
	
	float Poke_Speed_Get;
	float Poke_Speed_Set;
	float Poke_Angle_Get;
	float Poke_Angle_Set;
	float Poke_Angle_Init;
	
	float Friction_Speed_1_Get;
	float Friction_Speed_2_Get;
	float Friction_Speed_3_Get;
	float Friction_Speed_Set;
	
	float Bullet_Speed;
	
	float Bullet_Shot_CNT;
	
	int16_t Stop_Fire_CNT;
	
	float Heat;
	
	int16_t FullAuto_Fire_CNT;

	First_Order_Kalman_Filter_t Bullet_Speed_Kalman;
	
}Shooter_t;

extern Shooter_t Shooter;

void Shooter_PID_Init(void);
void Friction_Speed_Get(Shooter_t* Shooter);
void Shooter_Speed_CalcSend_FullAutoFire(float Speed,int8_t Freq);
void Shooter_Speed_CalcSend_Single(float Speed,uint8_t Fire_Flag);
void Shooter_Speed_CalcSend_GetTrap(float Speed,float Freq);
void    Shooter_Angle_Clear(void);
void Shooter_PID_Clear(void);
void Poke_Data_Get(Shooter_t* Shooter);
void Poke_Get_Trap_Judge(void);
void Friction_Get_Trap_Judge(void);
uint8_t Shooter_Cooling_Judge(void);
void Stop_Fire_Judge(void);
void Shooter_Bullet_Speed_Fbd_Set(Shooter_t *Shooter,float Bullet_Speed);
float First_Order_Kalman_Filter_Cal(First_Order_Kalman_Filter_t *_First_Order_Kalman_Filter,float _Z);
float Shooter_Bullet_Speed_Self_Adaptation(float Bullet_Speed);

#endif
