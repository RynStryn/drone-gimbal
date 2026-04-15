#include "main.h"

pidlc_t DJI6020_PID_V, DJI6020_PID_P, DM4310_PID_P, DM4310_PID_V,
    DJI2006_Poke_PID_V, DJI2006_Poke_PID_P,
    DJI2006_Fric_PID_V_1,DJI2006_Fric_PID_V_2, DJI2006_Fric_PID_V_3;


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

        CLAMP(pid->Iout, -pid->I_Max, pid->I_Max);

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

//IN:struct,arg
void Ctrler_Init(pidlc_t* pid,
    PID_Mode              Mode,
    Enable_D_on_Fdbk      isEnable,

    float Out_Max,
    float I_Max,
    float ff_v_outMax,
    float ff_a_outMax,

    float kp,
    float ki,
    float kd,
    float Ftr_alpha,

    float FKv,
    float FKa)
{
    Pid_Init(pid, Mode, isEnable, Out_Max, I_Max, kp, ki, kd, Ftr_alpha);
    FeedForward_Init(&pid->FdFwd, ff_v_outMax, ff_a_outMax, FKv, FKa);
    if (pid==&DM4310_PID_P)
    {pid->FdFwd.Kgrv=DM_kGRV;}
}
void Pid_Init(pidlc_t* pid,
    PID_Mode           Mode,
    Enable_D_on_Fdbk   isEnable,

    float Out_Max,
    float I_Max,


    float kp,
    float ki,
    float kd,
    float Ftr_alpha)
{
    pid->PID_Mode         = Mode;
    pid->Enable_D_on_Fdbk = isEnable;

    pid->Out_Max = Out_Max;
    pid->I_Max   = I_Max;

    pid->Kp       = kp;
    pid->Ki       = ki;
    pid->Kd       = kd;
    pid->filter_a = Ftr_alpha;
}
void FeedForward_Init(FdFwd_t* FF,
    float                      ff_v_outMax,
    float                      ff_a_outMax,
    float                      FKv,
    float                      FKa)
{
    FF->a_outMax = ff_a_outMax;
    FF->v_outMax = ff_v_outMax;

    FF->Kv = FKv;
    FF->Ka = FKa;
}
void PID_Clear(pidlc_t* pid)
{
    pid->Get[0]    = 0;
    pid->Get[1]    = 0;
    pid->Set       = 0;
    pid->Err[0]    = 0;
    pid->Err[1]    = 0;
    pid->Err[2]    = 0;
    pid->Pout      = 0;
    pid->Iout      = 0;
    pid->Dout      = 0;
    pid->Dout_last = 0;
    pid->Out       = 0;

    pid->FdFwd.v[0] = 0;
    pid->FdFwd.v[1] = 0;
    pid->FdFwd.x[0] = 0;
    pid->FdFwd.x[1] = 0;
    pid->FdFwd.a    = 0;
}
pidlc_t* pidGrp[]=
{
&DJI6020_PID_V, &DJI6020_PID_P, &DM4310_PID_P, &DM4310_PID_V
// &DJI2006_Poke_PID_V, &DJI2006_Poke_PID_P,
// &DJI2006_Fric_PID_V_1,&DJI2006_Fric_PID_V_2, &DJI2006_Fric_PID_V_3
};
void PID_AllClear(void)
{
    for (int i = 0; i < sizeof(pidGrp) / sizeof(pidGrp[0]); i++)
    {PID_Clear(pidGrp[i]);}
};
void Feed_Forward_Calc(FdFwd_t* f, float get)
{
    // smaller index indicate a newer value
    f->x[1] = f->x[0];
    f->v[1] = f->v[0];

    f->x[0] = get;
    f->v[0] = f->x[0] - f->x[1];
    f->a    = f->v[0] - f->v[1];
    // ·ÀÖ¹Ìø±ä
   // CLAMP(FF->v[0], -FF->v_max, FF->v_max);
    //CLAMP(FF->a, -FF->a_max, FF->a_max);
    // float frc=0;
    // if (f->v[0]< -f->frc_deadzone)
    // {frc=-f->Kfrc;}
    // if (f->v[0]> f->frc_deadzone)
    // {frc=f->Kfrc;}
    f->v_out =500* f->Kv * f->v[0];
    f->a_out = (250000 * f->Ka * f->a) + f->Kgrv * cos(get * DEG2RAD);

    CLAMP(f->v_out, -f->v_outMax, f->v_outMax);
    CLAMP(f->a_out, -f->a_outMax, f->a_outMax);

}
