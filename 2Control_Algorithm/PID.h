#ifndef __PID_H__
#define __PID_H__

#include "stm32f4xx.h"

//a=1.0 : None filter || ndef ...
#define filter
// PID_mode, D on fdbk, out max, Iout max, ff_out max
//	Kp, Ki, Kd ;; ff: Kv, Ka ;;filter: a
#define DJI6020_PID_V_Init \
Position_PID, Disable, 13000,200,\
 78, 2, 100

#define DJI6020_PID_P_Init \
Position_PID, Disable, 1200, 40, \
38, 0, 40

#define DM4310_PID_V_Init \
Position_PID, Disable, 18, 0, \
0, 0, 0

//---gimbal end---

#define DM4310_PID_P_Init \
Position_PID, Disable, 1000, 0, \
0, 0, 0

#define DJI2006_Poke_PID_V_Init \
Position_PID, Disable, 4000, 1000, \
12, 0.5, 2

#define DJI2006_Poke_PID_P_Init \
Position_PID, Disable, 8000, 200, \
3, 0.01, 1

#define DJI2006_Fric_1_PID_V_Init \
Position_PID, Disable, 4000, 580, \
12, 0.9, 9

#define DJI2006_Fric_2_PID_V_Init \
Position_PID, Disable, 4000, 580, \
12, 0.9, 9.4

#define DJI2006_Fric_3_PID_V_Init \
Position_PID, Disable, 4000, 600, \
15, 2, 15

typedef enum
{
    Position_PID=0,
    Delta_PID,
}PID_Mode;
typedef enum
{
    Disable=0,
    Enable,
}Enable_D_on_Fdbk;

typedef struct {
    float Kv;
    float Ka;
    float v_max;
    float a_max;
    float out_max;
    float T;
    uint8_t E_2nd_ff;
    //smaller index indicate a newer value
    float x[2];
    float v[2];
    float a;
    float v_out;
    float a_out;
}FdFwd_t;

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

	Enable_D_on_Fdbk		Enable_D_on_Fdbk;
	FdFwd_t FdFwd;
    float 	filter_a;
    float   Dout_last;
}pidlc_t;

extern	pidlc_t 	DJI6020_PID_V,		  DJI6020_PID_P,
				DM4310_PID_V,		  DM4310_PID_P,
				DJI2006_Poke_PID_V,	  DJI2006_Poke_PID_P,
				DJI2006_Fric_PID_V_1, DJI2006_Fric_PID_V_2,	DJI2006_Fric_PID_V_3;

void PID_Cal(pidlc_t* pid,float Set,float Get);
void PID_Init(pidlc_t* pid,
    PID_Mode           Mode,
    Enable_D_on_Fdbk                Flag,
    float              Out_Max,
    float              Inc_Max,

    float kp,
    float ki,
    float kd,
    float FKv,
    float FKa,
    float Ftr_alpha);
void PID_Clear(pidlc_t* pid);
void Feed_Forward_Calc(FdFwd_t* FF, float get);
#endif
