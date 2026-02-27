#include "main.h"

pid_t DJI6020_PID_V,		DJI6020_PID_P,
	  DM4310_PID_P,
	  DJI2006_Poke_PID_V,	DJI2006_Poke_PID_P,
	  DJI2006_Fric_PID_V_1,	DJI2006_Fric_PID_V_2, DJI2006_Fric_PID_V_3;
	  

void PID_Cal(pid_t* pid,float Set,float Get)
{
	pid->Get[1]=pid->Get[0];
	pid->Get[0]=Get;
	pid->Set=Set;
	
	pid->Err[2]  =	pid->Err[1]; 
	pid->Err[1]  =	pid->Err[0];
	pid->Err[0]  =	pid->Set-pid->Get[0];

	if(pid->PID_Mode==Position_PID)
	{
		if(pid->Ki==0)
			pid->Iout=0;
		
		pid->Pout = pid->Kp  *  pid->Err[0];
		
		pid->Iout+= pid->Ki  *  pid->Err[0];
		
		VAL_LIMIT(pid->Iout,-pid->Inc_Max,pid->Inc_Max);
		if(pid->DOF_Flag)
		{
			pid->Dout = pid->Kd  * (pid->Get[1] -pid->Get[0]);
		}
		else
		{
			pid->Dout = pid->Kd  * (pid->Err[0] -pid->Err[1]);
		}
		
		pid->Out  =  pid->Pout + pid->Iout + pid->Dout;
		
		VAL_LIMIT(pid->Out,-pid->Out_Max,pid->Out_Max);
	}
	else if(pid->PID_Mode==Delta_PID)
	{
		pid->Pout = pid->Kp * (pid->Err[0] - pid->Err[1]);
		pid->Iout = pid->Ki * pid->Err[0];
		pid->Dout = pid->Kd * (pid->Err[0] - 2 * pid->Err[1] + pid->Err[2]);
		
		pid->Out += pid->Pout + pid->Iout + pid->Dout;
		
		VAL_LIMIT(pid->Out,-pid->Out_Max,pid->Out_Max);
	}
	
//	if(pid->Out>-PID_DEAD_ZONE&&pid->Out<PID_DEAD_ZONE)
//	{
//		pid->Out=0;
//	}
}

void PID_StructInit(
	 pid_t 		*pid,
	 PID_Mode 	Mode,
	 DOF		Flag,
	 float 		Out_Max,
	 float 		Inc_Max,

	 float 		kp,
	 float		ki,
	 float 		kd)
{
pid->PID_Mode   =	Mode;
pid->Out_Max	=	Out_Max;
pid->Inc_Max	=	Inc_Max;
	
pid->Kp			=	kp;
pid->Ki			=	ki;
pid->Kd			=	kd;		
}

void PID_Clear(pid_t* pid)
{
	pid->Get[0]=0;
	pid->Get[1]=0;
	pid->Set=0;
	pid->Err[0]=0;
	pid->Err[1]=0;
	pid->Err[2]=0;
	pid->Pout=0;
	pid->Iout=0;
	pid->Dout=0;
	pid->Out=0;
}

