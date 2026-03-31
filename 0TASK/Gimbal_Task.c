#include "main.h"

Gimbal_t Gimbal = {
    .Yaw_Angle_Set   = 0,
    .Pitch_Angle_Set = -24,//_
};

void Gimbal_Ctrler_Init(void)
{
    Ctrler_Init(&DJI6020_PID_V, DJI6020_PID_V_Init);
    Ctrler_Init(&DJI6020_PID_P, DJI6020_PID_P_Init);
    Ctrler_Init(&DM4310_PID_P, DM4310_PID_P_Init);
    Ctrler_Init(&DM4310_PID_V, DM4310_PID_V_Init);
    gimbal_ESO_param_Init(&Gimbal.Yaw_LESO,1.3568e-04,180.0/PI,0.002,0.0029,0.0558,2500,800,0,0,0);
    gimbal_ESO_param_Init(&Gimbal.Pitch_LESO,1.3568e-04,180.0/PI,0.002,0.0082,0.5602,10000000,800,0,0,0);

}

void Gimbal_PID_Clear(void)
{
    PID_Clear(&DJI6020_PID_V);
    PID_Clear(&DJI6020_PID_P);
    PID_Clear(&DM4310_PID_P);
    PID_Clear(&DM4310_PID_V);
}
float err_yaw;
float err_pitch;
uint8_t hmFin_cnt;
void Gimbal_Homing(void)
{
    uint8_t        DATA[8];
    memset(DATA, 0, 8);

    err_yaw = DJI6020_YAW_ZERO_ENCODER - DJI6020_Yaw.pos_deg;
    if (err_yaw > 180.0f)
        err_yaw -= 360.0f;
    if (err_yaw < -180.0f)
        err_yaw += 360.0f;
    err_pitch = DM4310_Pitch_ZERO_ENCODER - DM4310_Pitch.POS;
    if (err_pitch > 180.0f)
        err_pitch -= 360.0f;
    if (err_pitch < -180.0f)
        err_pitch += 360.0f;

    if ((fabs(err_yaw) < DJI6020_YAW_HOMING_TOLERANCE)
#ifdef pitchHoming
        && (fabs(err_pitch) < DM4310_Pitch_HOMING_TOLERANCE)
#endif
    )
    {
        hmFin_cnt++;
    }
#ifdef pitchHoming
    CH040_Data.Init_pitch_agl=CH040_Data.pitch_agl_get;
#endif
    if (hmFin_cnt > 20)
    {
        DJI6020_Yaw.Init_Flag     = 1;
        DM4310_Pitch.Init_Flag    = 1;
        CH040_Data.Init_yaw_agl   = CH040_Data.yaw_agl_get;
        CH040_Data.Init_pitch_agl = CH040_Data.pitch_agl_get;
        PID_Clear(&DJI6020_PID_P);
        PID_Clear(&DJI6020_PID_V);
        Gimbal.Yaw_Angle_Set = Gimbal.Yaw_Angle_Get;

#ifdef pitchHoming
        PID_Clear(&DM4310_PID_P);
        PID_Clear(&DM4310_PID_V);
        Gimbal.Pitch_Angle_Set = Gimbal.Pitch_Angle_Get;
#endif
        DATA[0] = 0;
        DATA[1] = 0;
        CAN2_Transmit(DJI6020_Yaw_Send_ID, 8, DATA);
        return;
    }

    PID_Cal(&DJI6020_PID_P, 0, err_yaw);
    float Speed=DJI6020_PID_P.Out;
    CLAMP(Speed,-10,10);
    PID_Cal(&DJI6020_PID_V, Speed, Gimbal.Yaw_Speed_Get);
    float Effort =DJI6020_PID_V.Out;
    CLAMP(Effort,-16384,16384);

    Effort *= -1;
    DATA[0] = ((int) Effort >> 8) & 0xFF;
    DATA[1] = (int) Effort;
    CAN2_Transmit(DJI6020_Yaw_Send_ID, 8, DATA);
#ifdef pitchHoming
#    ifdef DM4310_V
    PID_Cal(&DM4310_PID_P, 0, err_pitch);
    Speed = DM4310_PID_P.Out;
    DM4310_V_Ctrl(DM4310_Pitch_ID_Send, Speed);
#    endif
#    ifdef DM4310_MIT
    PID_Cal(&DM4310_PID_P, 0, err_pitch);
    Speed = DM4310_PID_P.Out;
    PID_Cal(&DM4310_PID_V, Speed, Gimbal.Pitch_Speed_Get);
    float torq;
    torq = DM4310_PID_V.Out + 0.00668 * Gimbal.Pitch_Angle_Get + 0.2838;
    torq *= -1;
    torq = 0;
    DM4310_MIT_Ctrl(DM4310_Pitch_ID_Send, 0, 0, 0, 0, torq);
#    endif
#endif

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
    float Effort =(DJI6020_PID_V.Out+DJI6020_PID_P.FdFwd.a_out)-Gimbal.Yaw_LESO.output;
		CLAMP(Effort,-16384,16384);
		gimbal_ESO_cal(&Gimbal.Yaw_LESO,Effort,Gimbal.Yaw_Speed_Get);
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
#ifdef DM4310_V
    Feed_Forward_Calc(&DM4310_PID_P.FdFwd, Angle);
    PID_Cal(&DM4310_PID_P, Angle, Gimbal.Pitch_Angle_Get);
    Speed = DM4310_PID_P.Out + DM4310_PID_P.FdFwd.v_out;
    DM4310_V_Ctrl(DM4310_Pitch_ID_Send, Speed);
#endif
#ifdef DM4310_MIT
    Feed_Forward_Calc(&DM4310_PID_P.FdFwd, Angle);
    PID_Cal(&DM4310_PID_P, Angle, Gimbal.Pitch_Angle_Get);
    Speed = DM4310_PID_P.Out + DM4310_PID_P.FdFwd.v_out;
    PID_Cal(&DM4310_PID_V, Speed, Gimbal.Pitch_Speed_Get);
    float theta = CH040_Data.pitch_agl_get * DEG2RAD;
    gimbal_ESO_cal(&Gimbal.Pitch_LESO, torq, Gimbal.Pitch_Speed_Get);
    torq = DM4310_PID_V.Out + 0.00668 * Gimbal.Pitch_Angle_Get + 0.2838
        - Gimbal.Pitch_LESO.output;
#endif
#ifdef DM4310_TEST
    // torq = ( testEffort+(-kg)*cos( theta))//+(-kgg)*sin(theta);
#endif

    torq *=-1;
	  torq=0;
    DM4310_MIT_Ctrl(DM4310_Pitch_ID_Send, 0, 0, 0, 0, torq);
}

//TODO
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


void Gimbal_Ang_Spd_Get(Gimbal_t* Gimbal,
    CH040_t*                         CH040_Data,
    DJI6020_t*                       DJI6020_Yaw,
    DM4310_t*                        DM4310_Pitch)
{
    Gimbal->Yaw_Angle_Get   = CH040_Data->yaw_total_agl_calib;
    Gimbal->Pitch_Angle_Get = CH040_Data->pitch_agl_calib;
    Gimbal->Yaw_Speed_Get   = CH040_Data->omega_z;
    Gimbal->Pitch_Speed_Get = CH040_Data->omega_x;
    // Gimbal->Yaw_Angle_Chassis = DJI6020_Yaw->multi_turns_deg - CH040_Data->
    //                             yaw_a - DJI6020_Yaw->Init_deg + CH040_Data->
    //                             Init_yaw_a;
}
