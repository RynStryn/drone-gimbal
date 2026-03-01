#include "main.h"

Gimbal_t Gimbal = {
    .Yaw_Angle_Set   = 0,
    .Pitch_Angle_Set = 0,
};

void Gimbal_PID_Init(void)
{
    PID_Init(&DJI6020_PID_V, DJI6020_PID_V_Init);
    PID_Init(&DJI6020_PID_P, DJI6020_PID_P_Init);
    PID_Init(&DM4310_PID_P, DM4310_PID_P_Init);
}

void Gimbal_PID_Clear(void)
{
    PID_Clear(&DJI6020_PID_V);
    PID_Clear(&DJI6020_PID_P);
    PID_Clear(&DM4310_PID_P);
}

void DJI6020_Yaw_CalcSend(float Angle)
{
    uint8_t DATA[8];
    memset(DATA, 0, 8);

    Feed_Forward_Calc(&DJI6020_PID_P.FdFwd, Angle);
    PID_Cal(&DJI6020_PID_P, Angle, Gimbal.Yaw_Angle_Get);
    float Speed=DJI6020_PID_P.Out+DJI6020_PID_P.FdFwd.v_out;
    PID_Cal(&DJI6020_PID_V, Speed, Gimbal.Yaw_Speed_Get);
    float Effort =(DJI6020_PID_V.Out+DJI6020_PID_P.FdFwd.a_out);
    Effort *=-1;
    DATA[0] = ((int) Effort >> 8) & 0xFF;
    DATA[1] = (int) Effort;
    CAN2_Transmit(DJI6020_Yaw_Send_ID, 8, DATA);
}

void DM4310_Pitch_Control(float Angle)
{//TODO
    DM4310_PV_Ctrl(DM4310_Pitch_ID_Send,
        Angle + DM4310_Pitch.POS + Gimbal.Pitch_Angle_Get,//_?
        60);
}


void Gimbal_Set_Inc(Gimbal_t* Gimbal, float Yaw_inc, float Pitch_inc)
{
    //_sgn(gm6020×°·´ÁË)
    if (DJI6020_Yaw.multi_turns_deg + Yaw_inc <= Yaw_Angle_MIN && Yaw_inc >= 0)
    {
        Yaw_inc               = 0;
        Gimbal->Yaw_Angle_Set = Gimbal->Yaw_Angle_Get;
    } else if (DJI6020_Yaw.multi_turns_deg + Yaw_inc >= Yaw_Angle_MAX
        && Yaw_inc <= 0)
    {
        Yaw_inc               = 0;
        Gimbal->Yaw_Angle_Set = Gimbal->Yaw_Angle_Get;
    } else
    {
        Gimbal->Yaw_Angle_Set += Yaw_inc;
    }

    if (DM4310_Pitch.POS + Pitch_inc <= Pitch_Angle_MIN && Pitch_inc <= 0)
    {
        Pitch_inc               = 0;
        Gimbal->Pitch_Angle_Set = -Gimbal->Pitch_Angle_Get;
    } else if (DM4310_Pitch.POS + Pitch_inc >= Pitch_Angle_MAX
        && Pitch_inc >= 0)
    {
        Pitch_inc               = 0;
        Gimbal->Pitch_Angle_Set = -Gimbal->Pitch_Angle_Get;
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
    Gimbal_Set_Inc(Gimbal, Yaw_inc, Pitch_inc);
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
    // Gimbal->Yaw_Angle_Chassis = DJI6020_Yaw->multi_turns_deg - CH040_Data->
    //                             yaw_a - DJI6020_Yaw->Init_deg + CH040_Data->
    //                             Init_yaw_a;
}
