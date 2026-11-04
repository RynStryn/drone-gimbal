#include "main.h"

Gimbal_t Gimbal = {
    .Yaw_Angle_Set   = 0,
    .Pitch_Angle_Set = -24,//_
};

void Gimbal_PID_Init(void)
{
    Ctrler_Init(&DJI6020_PID_V, DJI6020_PID_V_Init);
    Ctrler_Init(&DJI6020_PID_P, DJI6020_PID_P_Init);
    Ctrler_Init(&DM4310_PID_P, DM4310_PID_P_Init);
    Ctrler_Init(&DM4310_PID_V, DM4310_PID_V_Init);
}

void Gimbal_PID_Clear(void)
{
    PID_Clear(&DJI6020_PID_V);
    PID_Clear(&DJI6020_PID_P);
    PID_Clear(&DM4310_PID_P);
    PID_Clear(&DM4310_PID_V);
}
// 500Hz,2ms
//angle->deg,speed->deg/s;T=2ms,pid,ff省略 (*/)T;
#ifdef YAW_TEST
float Target_Angle=0;
float A=30;
float omega=0.005;
float Time=0;
uint16_t runit =0;
uint16_t isInit=0;
uint16_t wave=1;
float stepTA=0;

#endif

#ifdef DJI6020_TEST
float testEffort=0;
#endif
float TempAngle;
float Effort=0;
    void DJI6020_Yaw_CalcSend(float Angle)
{
    uint8_t DATA[8];
    memset(DATA, 0, 8);
#ifdef YAW_TEST


        if (runit == 1)
        {Time++;}
        // else
        // {isInit++;
        //    if (isInit<20)
        //    {Target_Angle=Gimbal.Pitch_Angle_Get;
        //        Gimbal_PID_Clear();
        //        }
        //    if(isInit==20)
        //    {TempAngle=Target_Angle;}
        //    if(isInit>20&&isInit<222)
        //    {Target_Angle-=TempAngle/200;}
        //    if(isInit>1000)
        //    {isInit=1000;}
        // }
    if (wave == 1)
        {Target_Angle = A * sin(omega * Time);}
    else
    {
        if (Target_Angle - stepTA > 20 || Target_Angle - stepTA < -20)
        {
            stepTA       = Gimbal.Yaw_Angle_Get;
            Target_Angle = Gimbal.Yaw_Angle_Get;
        }
        Target_Angle = stepTA;
    }
    Feed_Forward_Calc(&DJI6020_PID_P.FdFwd, Target_Angle);
    PID_Cal(&DJI6020_PID_P, Target_Angle, Gimbal.Yaw_Angle_Get);
    float Speed = DJI6020_PID_P.Out + DJI6020_PID_P.FdFwd.v_out;
    PID_Cal(&DJI6020_PID_V, Speed, Gimbal.Yaw_Speed_Get);
    float Effort = (DJI6020_PID_V.Out + DJI6020_PID_P.FdFwd.a_out);

		if(isInit<20)
{Effort=0;}
#else

    Feed_Forward_Calc(&DJI6020_PID_P.FdFwd, Angle);
    PID_Cal(&DJI6020_PID_P, Angle, Gimbal.Yaw_Angle_Get);
    float Speed=DJI6020_PID_P.Out+DJI6020_PID_P.FdFwd.v_out;
    PID_Cal(&DJI6020_PID_V, Speed, Gimbal.Yaw_Speed_Get);
    Effort =(DJI6020_PID_V.Out+DJI6020_PID_P.FdFwd.a_out);
#endif
    Effort *=-1;
#ifdef DJI6020_TEST
        Effort=testEffort;
#endif

//Effort=0;
    DATA[0] = ((int) Effort >> 8) & 0xFF;
    DATA[1] = (int) Effort;
    CAN2_Transmit(DJI6020_Yaw_Send_ID, 8, DATA);

}

/*
float dmpos=0;
void DM4310_Pitch_Control(float Angle)
    {
    dmpos = Angle + DM4310_Pitch.POS + Gimbal.Pitch_Angle_Get;
        DM4310_PV_Ctrl(DM4310_Pitch_ID_Send,
            dmpos,
            60);
    }
*/

//500Hz,2ms;n.m
//angle->deg,speed->deg/s;T=2ms,pid,ff省略 (*/)T;

float torq;
#ifdef PITCH_TEST
float Target_Angle=0;
float A=20;
float omega=0.002;
float Time=0;
uint16_t runit =0;
uint16_t isInit=1;
uint16_t wave=1;
float stepTA=0;

#endif

#ifdef DM4310_TEST
float testEffort=0;
#endif
float TempAngle;
float Speed;
float kg=0.38;//0.4大致平衡,但很可能把摩擦也带上了
//float kgg=0.15;//?->0.4
    void DM4310_Pitch_Control(float Angle)
{
    #ifndef MIT
        DM4310_PV_Ctrl(DM4310_Pitch_ID_Send,
            Angle + DM4310_Pitch.POS + Gimbal.Pitch_Angle_Get,
            60);
    #else
    // TempAngle = Angle + Gimbal.Pitch_Angle_Get;
    Feed_Forward_Calc(&DM4310_PID_P.FdFwd, Angle);
    PID_Cal(&DM4310_PID_P, Angle, Gimbal.Pitch_Angle_Get);
    Speed = DM4310_PID_P.Out + DM4310_PID_P.FdFwd.v_out;
    PID_Cal(&DM4310_PID_V, Speed, Gimbal.Pitch_Speed_Get);
        float theta=CH040_Data.pitch_agl_get*DEG2RAD;
    torq = (DM4310_PID_V.Out + DM4310_PID_P.FdFwd.a_out+(kg)*cos( theta));
#endif
#ifdef DM4310_TEST
    torq = ( testEffort+(-kg)*cos( theta))//+(-kgg)*sin(theta);
#endif
   
        torq*=-1;
	  //torq=0;
    DM4310_MIT_Ctrl(DM4310_Pitch_ID_Send, 0, 0, 0, 0, torq);
}

void Gimbal_Set_Inc(Gimbal_t* Gimbal, float Yaw_inc, float Pitch_inc)
{
    //_sgn
    /*if (DJI6020_Yaw.multi_turns_deg + Yaw_inc <= Yaw_Angle_MIN && Yaw_inc >= 0)
    {
        Yaw_inc               = 0;
        Gimbal->Yaw_Angle_Set = Gimbal->Yaw_Angle_Get;
    } else if (DJI6020_Yaw.multi_turns_deg + Yaw_inc >= Yaw_Angle_MAX
        && Yaw_inc <= 0)
    {
        Yaw_inc               = 0;
        Gimbal->Yaw_Angle_Set = Gimbal->Yaw_Angle_Get;
    } else*/ if (Yaw_inc< -0.01||Yaw_inc>+0.01)//_deadzone
    {
        Gimbal->Yaw_Angle_Set += Yaw_inc;
    }

    if (-DM4310_Pitch.POS + Pitch_inc >= -Pitch_Angle_MIN && Pitch_inc >= 0)
    {
        Pitch_inc               = 0;
        Gimbal->Pitch_Angle_Set = Gimbal->Pitch_Angle_Get;//_
    } else if (-DM4310_Pitch.POS + Pitch_inc <= -Pitch_Angle_MAX
        && Pitch_inc <= 0)
    {
        Pitch_inc               = 0;
        Gimbal->Pitch_Angle_Set = Gimbal->Pitch_Angle_Get;//_
    } else
    {
        Gimbal->Pitch_Angle_Set += Pitch_inc;
    }
}

// Incremental
void Gimbal_Control_Task_Inc(Gimbal_t* Gimbal, float Yaw_inc, float Pitch_inc)
{
    static volatile uint16_t task_timer = 0;
    task_timer++;
    Gimbal_Set_Inc(Gimbal, Yaw_inc, Pitch_inc);//_
    if (task_timer % 2 == 0)
        DJI6020_Yaw_CalcSend(Gimbal->Yaw_Angle_Set);
    if (task_timer % 2 == 1)
        DM4310_Pitch_Control(Gimbal->Pitch_Angle_Set);
    if (task_timer >= 10000)
    {
        task_timer = 0;
    }
}


void Gimbal_Get_Angle_Data(Gimbal_t* Gimbal,
    CH040_t*                         CH040_Data,
    DJI6020_t*                       DJI6020_Yaw,
    DM4310_t*                        DM4310_Pitch)
{
    Gimbal->Yaw_Angle_Get   = CH040_Data->yaw_total_agl_calib;
    Gimbal->Pitch_Angle_Get = CH040_Data->pitch_agl_calib;
    Gimbal->Yaw_Speed_Get   = CH040_Data->omega_z;
    Gimbal->Pitch_Speed_Get = CH040_Data->omega_x;
    // Gimbal->Yaw_Angle_Ch`assis = DJI6020_Yaw->multi_turns_deg - CH040_Data->
    //                             yaw_a - DJI6020_Yaw->Init_deg + CH040_Data->
    //                             Init_yaw_a;
}
