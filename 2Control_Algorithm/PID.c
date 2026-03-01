#include "main.h"

pidlc_t DJI6020_PID_V, DJI6020_PID_P, DM4310_PID_P, DJI2006_Poke_PID_V,
    DJI2006_Poke_PID_P, DJI2006_Fric_PID_V_1, DJI2006_Fric_PID_V_2,
    DJI2006_Fric_PID_V_3;


void PID_Cal(pidlc_t* pid, float Set, float Get)
{
    pid->Get[1] = pid->Get[0];
    pid->Get[0] = Get;
    pid->Set    = Set;

    pid->Err[2]    = pid->Err[1];
    pid->Err[1]    = pid->Err[0];
    pid->Err[0]    = pid->Set - pid->Get[0];
    pid->Dout_last = pid->Dout;
    if (pid->PID_Mode == Position_PID)
    {
        if (pid->Ki == 0)
            pid->Iout = 0;

        pid->Pout = pid->Kp * pid->Err[0];

        pid->Iout += pid->Ki * pid->Err[0];

        CLAMP(pid->Iout, -pid->Inc_Max, pid->Inc_Max);

        if (pid->Enable_D_on_Fdbk)
        {
            pid->Dout = pid->Kd * (pid->Get[1] - pid->Get[0]);
        }
        else
        {
            pid->Dout = pid->Kd * (pid->Err[0] - pid->Err[1]);
        }
#ifdef filter
        pid->Dout = pid->filter_a * pid->Dout
            + (1.0f - pid->filter_a) * pid->Dout_last;
#endif
        pid->Out = pid->Pout + pid->Iout + pid->Dout;

        CLAMP(pid->Out, -pid->Out_Max, pid->Out_Max);
    }
    else if (pid->PID_Mode == Delta_PID)
    {
        pid->Pout = pid->Kp * (pid->Err[0] - pid->Err[1]);
        pid->Iout = pid->Ki * pid->Err[0];
        pid->Dout = pid->Kd * (pid->Err[0] - 2 * pid->Err[1] + pid->Err[2]);

        pid->Out += pid->Pout + pid->Iout + pid->Dout;

        CLAMP(pid->Out, -pid->Out_Max, pid->Out_Max);
    }

    //	if(pid->Out> -PID_DEAD_ZONE&&pid->Out<PID_DEAD_ZONE)
    //	{
    //		pid->Out=0;
    //	}
}

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
    float Ftr_alpha)
{
    pid->PID_Mode = Mode;
    pid->Out_Max  = Out_Max;
    pid->Inc_Max  = Inc_Max;

    pid->Kp       = kp;
    pid->Ki       = ki;
    pid->Kd       = kd;
    pid->FdFwd.Kv = FKv;
    pid->filter_a=Ftr_alpha;
    if (pid->FdFwd.E_2nd_ff == 0 || pid->FdFwd.Ka == 0)
    {
        pid->FdFwd.Ka       = 0;
        pid->FdFwd.E_2nd_ff = 0;
    }
    else if (pid->FdFwd.E_2nd_ff == 1 && pid->FdFwd.Ka != 0)
    {
        pid->FdFwd.Ka = FKa;
    }
}

void PID_Clear(pidlc_t* pid)
{
    pid->Get[0]     = 0;
    pid->Get[1]     = 0;
    pid->Set        = 0;
    pid->Err[0]     = 0;
    pid->Err[1]     = 0;
    pid->Err[2]     = 0;
    pid->Pout       = 0;
    pid->Iout       = 0;
    pid->Dout       = 0;
    pid->Dout_last  = 0;
    pid->Out        = 0;

    pid->FdFwd.v[0] = 0;
    pid->FdFwd.v[1] = 0;
    pid->FdFwd.x[0] = 0;
    pid->FdFwd.x[1] = 0;
    pid->FdFwd.a    = 0;
}

void Feed_Forward_Calc(FdFwd_t* FF, float get)
{
    // smaller index indicate a newer value
    FF->x[1] = FF->x[0];
    FF->v[1] = FF->v[0];

    FF->x[0] = get;
    FF->v[0] = FF->x[0] - FF->x[1];
    FF->a    = FF->v[0] - FF->v[1];
    // ·ÀÖ¹Ìø±ä
    CLAMP(FF->v[0], -FF->v_max, FF->v_max);
    CLAMP(FF->a, -FF->a_max, FF->a_max);
    FF->v_out = FF->Kv * FF->v[0];
    FF->a_out = FF->Ka * FF->a;
    CLAMP(FF->v_out,-FF->out_max,FF->out_max);
    CLAMP(FF->a_out,-FF->out_max,FF->out_max);
}

