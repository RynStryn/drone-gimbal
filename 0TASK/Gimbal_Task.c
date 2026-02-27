#include "main.h"

Gimbal_t Gimbal=
{
	.Yaw_Angle_Set=0,
	.Pitch_Angle_Set=0,
};

void Gimbal_PID_Init(void)
{
	PID_StructInit(&DJI6020_PID_V,DJI6020_PID_V_Init);
	PID_StructInit(&DJI6020_PID_P,DJI6020_PID_P_Init);
	PID_StructInit(&DM4310_PID_P,DM4310_PID_P_Init);
	
}
void Gimbal_PID_Clear(void)
{
	PID_Clear(&DJI6020_PID_V);
	PID_Clear(&DJI6020_PID_P);
	PID_Clear(&DM4310_PID_P);
}

void DJI6020_Yaw_Control(float Angle)
{
	uint8_t DATA[8];
	memset(DATA,0,8);
	PID_Cal(&DJI6020_PID_P,Angle,Gimbal.Yaw_Angle_Actual);
	PID_Cal(&DJI6020_PID_V,DJI6020_PID_P.Out,Gimbal.Yaw_Speed_Act);
	DJI6020_PID_V.Out*=-1;
	DATA[0]=((int)DJI6020_PID_V.Out >> 8) & 0xFF;
	DATA[1]=(int)DJI6020_PID_V.Out;
	CAN2_Transmit(DJI6020_Yaw_Send_ID,8,DATA);
}
void DM4310_Pitch_Control(float Angle)
{
		DM4310_PV_Ctrl(DM4310_Pitch_ID_Send,Angle+DM4310_Pitch.POS+Gimbal.Pitch_Angle_Actual,60);
}


void Gimbal_Set_Inc(Gimbal_t* Gimbal,float Yaw,float Pitch)
{
	if(DJI6020_Yaw.mult_degs+Yaw<=Yaw_Angle_MIN&&Yaw>=0)
	{
		Yaw=0;
		Gimbal->Yaw_Angle_Set=Gimbal->Yaw_Angle_Actual;
	}
	else if(DJI6020_Yaw.mult_degs+Yaw>=Yaw_Angle_MAX&&Yaw<=0)
	{
		Yaw=0;
		Gimbal->Yaw_Angle_Set=Gimbal->Yaw_Angle_Actual;
	}
	else
	{
	Gimbal->Yaw_Angle_Set+=Yaw;
	}
	if(DM4310_Pitch.POS+Pitch<=Pitch_Angle_MIN&&Pitch<=0)
	{
		Pitch=0;
		Gimbal->Pitch_Angle_Set=-Gimbal->Pitch_Angle_Actual;
	}
	else if(DM4310_Pitch.POS+Pitch>=Pitch_Angle_MAX&&Pitch>=0)
	{
		Pitch=0;
		Gimbal->Pitch_Angle_Set=-Gimbal->Pitch_Angle_Actual;
	}
	else
	{
	Gimbal->Pitch_Angle_Set+=Pitch;
	}
	
}
//Incremental
void Gimbal_Control_Task_Inc(Gimbal_t* Gimbal,float Yaw,float Pitch)
{
	static volatile uint16_t Control_Flag=0;
	Control_Flag++;
	Gimbal_Set_Inc(Gimbal,Yaw,Pitch);
	if(Control_Flag%2==0)
		DJI6020_Yaw_Control(Gimbal->Yaw_Angle_Set);
	if(Control_Flag%2-1==0)
		DM4310_Pitch_Control(Gimbal->Pitch_Angle_Set);
	if(Control_Flag>=10000)
	{
		Control_Flag=0;
	}
	
}

void Gimbal_Get_Angle_Data(Gimbal_t *Gimbal,CH040_t *CH040_Data,DJI6020_t* DJI6020_Yaw,DM4310_t* DM4310_Pitch)
{
	Gimbal->Yaw_Angle_Actual	=	CH040_Data->yaw_a;
	Gimbal->Pitch_Angle_Actual	=	CH040_Data->pitch_a;
	Gimbal->Yaw_Speed_Act	=	CH040_Data->gry_z;
	Gimbal->Pitch_Speed_Act	=	CH040_Data->gry_x;
	Gimbal->Yaw_Angle_Chassis=DJI6020_Yaw->mult_degs-CH040_Data->yaw_a-DJI6020_Yaw->Init_degs+CH040_Data->Init_yaw_a;
}


