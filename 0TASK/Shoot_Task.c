#include "main.h"

int fired_bullet_cnt = 0;

Shooter_t Shooter = {
    .Bullet_Speed_Kalman.X_hat       = BULLET_SPEED_TARGET,
    .Bullet_Speed_Kalman.Error_Mea   = 0.01,
    .Bullet_Speed_Kalman.Error_Est   = 1000,
    .Bullet_Speed_Kalman.Kalman_Gain = 0.02,
    //	.Bullet_Speed_Kalman.Kalman_Gain=0.022,
    .Friction_Speed_Set = FRICTION_SPEED,
    .Stop_Fire_CNT      = 4000,
    .Freq               = 5,
};

void Shooter_PID_Init(void)
{
    Ctrler_Init(&DJI2006_Poke_PID_V, DJI2006_Poke_PID_V_Init);
    Ctrler_Init(&DJI2006_Poke_PID_P, DJI2006_Poke_PID_P_Init);

    Ctrler_Init(&DJI2006_Fric_PID_V_1, DJI2006_Fric_1_PID_V_Init);
    Ctrler_Init(&DJI2006_Fric_PID_V_2, DJI2006_Fric_2_PID_V_Init);
    Ctrler_Init(&DJI2006_Fric_PID_V_3, DJI2006_Fric_3_PID_V_Init);
}

void Shooter_PID_Clear(void)
{
    PID_Clear(&DJI2006_Poke_PID_V);
    PID_Clear(&DJI2006_Poke_PID_P);
    PID_Clear(&DJI2006_Fric_PID_V_1);
    PID_Clear(&DJI2006_Fric_PID_V_2);
    PID_Clear(&DJI2006_Fric_PID_V_3);
}

void Friction_Spd_Get(Shooter_t* Shooter)
{
    Shooter->Friction_Speed_1_Get = DJI2006_Fric[0].speed_rpm;
    Shooter->Friction_Speed_2_Get = DJI2006_Fric[1].speed_rpm;
    Shooter->Friction_Speed_3_Get = DJI2006_Fric[2].speed_rpm;
}

void Shooter_Speed_CalcSend_Single(float Speed, uint8_t Fire_Flag)
{
    static volatile uint8_t Fire_Flag_Last = 0;
    // 只有...才...
    if (Fire_Flag == 1 && Fire_Flag_Last == 0)
    {
        Shooter.Poke_Angle_Set += 1620;  //单发(45 deg)
    }
    Fire_Flag_Last = Fire_Flag;
    PID_Cal(&DJI2006_Fric_PID_V_1, Speed, Shooter.Friction_Speed_1_Get);
    PID_Cal(&DJI2006_Fric_PID_V_2, Speed, Shooter.Friction_Speed_2_Get);
    PID_Cal(&DJI2006_Fric_PID_V_3, Speed, Shooter.Friction_Speed_3_Get);
    PID_Cal(
        &DJI2006_Poke_PID_P, Shooter.Poke_Angle_Set, Shooter.Poke_Angle_Get);
    Shooter.Poke_Speed_Set = DJI2006_Poke_PID_P.Out;
    PID_Cal(
        &DJI2006_Poke_PID_V, Shooter.Poke_Speed_Set, Shooter.Poke_Speed_Get);
    uint8_t DATA[8];

    DATA[0] = ((int) DJI2006_Fric_PID_V_1.Out >> 8) & 0xFF;
    DATA[1] = (int) DJI2006_Fric_PID_V_1.Out;
    DATA[2] = ((int) -DJI2006_Fric_PID_V_2.Out >> 8) & 0xFF;
    DATA[3] = (int) -DJI2006_Fric_PID_V_2.Out;
    DATA[4] = ((int) DJI2006_Fric_PID_V_3.Out >> 8) & 0xFF;
    DATA[5] = (int) DJI2006_Fric_PID_V_3.Out;
    DATA[6] = ((int) DJI2006_Poke_PID_V.Out >> 8) & 0xFF;
    DATA[7] = (int) DJI2006_Poke_PID_V.Out;
    CAN1_Transmit(0x200, 8, DATA);
}

void Shooter_Speed_CalcSend_FullAutoFire(float Speed, int8_t Freq)
{
    if (Freq == 0)
    {
        Shooter_Speed_CalcSend_Single(Speed, 0);
    }
    else if (Shooter.FullAuto_Fire_CNT % (1000 / Freq) == 0)
    {
        Shooter_Speed_CalcSend_Single(Speed, 1);
    }
    else
    {
        Shooter_Speed_CalcSend_Single(Speed, 0);
    }
    Shooter.FullAuto_Fire_CNT++;
    if (Shooter.FullAuto_Fire_CNT >= 10000)
    {
        Shooter.FullAuto_Fire_CNT = 1;
    }
}

void Shooter_Speed_CalcSend_GetTrap(float Speed, float Freq)
{
    PID_Cal(&DJI2006_Fric_PID_V_1, Speed, Shooter.Friction_Speed_1_Get);
    PID_Cal(&DJI2006_Fric_PID_V_2, Speed, Shooter.Friction_Speed_2_Get);
    PID_Cal(&DJI2006_Fric_PID_V_3, Speed, Shooter.Friction_Speed_3_Get);
    Shooter.Poke_Speed_Set = 270 * Freq;
    PID_Cal(
        &DJI2006_Poke_PID_V, Shooter.Poke_Speed_Set, Shooter.Poke_Speed_Get);
    uint8_t DATA[8];
    DATA[0] = ((int) DJI2006_Fric_PID_V_1.Out >> 8) & 0xFF;
    DATA[1] = (int) DJI2006_Fric_PID_V_1.Out;
    DATA[2] = ((int) -DJI2006_Fric_PID_V_2.Out >> 8) & 0xFF;
    DATA[3] = (int) -DJI2006_Fric_PID_V_2.Out;
    DATA[4] = ((int) DJI2006_Fric_PID_V_3.Out >> 8) & 0xFF;
    DATA[5] = (int) DJI2006_Fric_PID_V_3.Out;
    DATA[6] = ((int) DJI2006_Poke_PID_V.Out >> 8) & 0xFF;
    DATA[7] = (int) DJI2006_Poke_PID_V.Out;
    CAN1_Transmit(0x200, 8, DATA);
}

//in:Shooter.switch_Single_Mode_Flag
//out:Shooter.switch_Single_Mode_Flag_Finish
//似乎是回转
void Shooter_Angle_Clear(void)
{
    static volatile int16_t Mode_Change_Cnt = 0;
    if (Shooter.switch_Single_Mode_Flag == 1 && Mode_Change_Cnt < 200)
    {
        //		PID_Cal(&DJI2006_Fric_PID_V_1,0,Shooter.Friction_Speed_1_Act);
        //		PID_Cal(&DJI2006_Fric_PID_V_2,0,Shooter.Friction_Speed_2_Act);
        //		PID_Cal(&DJI2006_Fric_PID_V_3,0,Shooter.Friction_Speed_3_Act);

        Shooter.Poke_Speed_Set = -2700;
        PID_Cal(&DJI2006_Poke_PID_V,
            Shooter.Poke_Speed_Set,
            Shooter.Poke_Speed_Get);
        uint8_t DATA[8];
        DATA[0] = ((int) 0 >> 8) & 0xFF;
        DATA[1] = (int) 0;
        DATA[2] = ((int) 0 >> 8) & 0xFF;
        DATA[3] = (int) 0;
        DATA[4] = ((int) 0 >> 8) & 0xFF;
        DATA[5] = (int) 0;
        DATA[6] = ((int) DJI2006_Poke_PID_V.Out >> 8) & 0xFF;
        DATA[7] = (int) DJI2006_Poke_PID_V.Out;
        CAN1_Transmit(0x200, 8, DATA);
    }
    else if (Mode_Change_Cnt >= 200)
    {
        DJI2006_Pock.pos_turns                 = 0;
        Shooter.Poke_Angle_Set                 = DJI2006_Pock.pos_deg + 900;
        Shooter.Poke_Angle_Init                = Shooter.Poke_Angle_Set;
        Shooter.switch_Single_Mode_Flag_Finish = 1;
        Mode_Change_Cnt                        = 0;
    }
    Mode_Change_Cnt++;
}

void Poke_Ang_Spd_Get(Shooter_t* Shooter)
{
    Shooter->Poke_Angle_Get = DJI2006_Pock.multi_turns_deg;
    Shooter->Poke_Speed_Get = DJI2006_Pock.speed_rpm;
}

void Poke_Get_Trap_Judge(void)
{
    if (Shooter.Poke_Speed_Get + 2000 <= Shooter.Poke_Speed_Set)
    {
        if (Shooter.State == START)
        {
            Shooter.Poke_Get_Trap_cnt++;
        }
    }
    if (Shooter.Poke_Speed_Get + 2000 >= Shooter.Poke_Speed_Set)
    {
        if (Shooter.State == START)
        {
            if (Shooter.Poke_Get_Trap_cnt > 0)
            {
                Shooter.Poke_Get_Trap_cnt--;
            }
        }
    }
    if (Shooter.Poke_Get_Trap_cnt >= 100)
    {
        Shooter.State = Poke_Get_Trap;
    }
    else if (Shooter.Poke_Get_Trap_cnt <= 0
        && Shooter.State != Friction_Get_Trap)
    {
        Shooter.State = START;
    }

    if (Shooter.State == Poke_Get_Trap && Shooter.Poke_Get_Trap_cnt > 0)
    {
        Shooter.Poke_Get_Trap_cnt--;
    }
}

void Friction_Get_Trap_Judge(void)
{
    if ((DJI2006_Fric[0].speed_rpm < FRICTION_SPEED_MIN
            || DJI2006_Fric[1].speed_rpm < FRICTION_SPEED_MIN
            || DJI2006_Fric[2].speed_rpm < FRICTION_SPEED_MIN)
        && Shooter.State == START)
    {
        Shooter.Friction_Get_Trap_Cnt++;
    }
    if ((DJI2006_Fric[0].speed_rpm > FRICTION_SPEED_MIN
            && DJI2006_Fric[1].speed_rpm > FRICTION_SPEED_MIN
            && DJI2006_Fric[2].speed_rpm > FRICTION_SPEED_MIN)
        && Shooter.State == START)
    {
        if (Shooter.Friction_Get_Trap_Cnt > 0)
            Shooter.Friction_Get_Trap_Cnt--;
    }
    if (Shooter.Friction_Get_Trap_Cnt >= 1000)
    {
        Shooter.State = Friction_Get_Trap;
    }
    else if (Shooter.Friction_Get_Trap_Cnt <= 0
        && Shooter.State != Poke_Get_Trap)
    {
        Shooter.State = START;
    }
    if (Shooter.State == Friction_Get_Trap && Shooter.Friction_Get_Trap_Cnt > 0)
    {
        Shooter.Friction_Get_Trap_Cnt -= 0.5;
    }
}

// 停火校验（停火时缓慢增加摩擦轮转速，减少因为摩擦轮冷却收缩导致的弹速大幅度降低）
void Stop_Fire_Judge(void)
{
    static volatile float Friction_Speed_Last;

    if (Shooter.Poke_State == POKE_OFF
        || (Shooter.Poke_Speed_Set - Shooter.Poke_Speed_Get > 1000  // trapped
            || Shooter.Poke_Speed_Set < 0))                         // 反转
    {
        if (Shooter.Friction_Speed_Set < FRICTION_SPEED + 200)
        {
            // target:匀加速
            Shooter.Friction_Speed_Set
                = FRICTION_SPEED + 0.05 * Shooter.Stop_Fire_CNT;
            CLAMP(Shooter.Friction_Speed_Set,
                FRICTION_SPEED,
                FRICTION_SPEED + 200);
        }
        if (Shooter.Stop_Fire_CNT < 4000)
        {
            Shooter.Stop_Fire_CNT++;
        }
        Friction_Speed_Last = Shooter.Friction_Speed_Set;
    }
    else if (Shooter.Poke_State == POKE_ON)
    {
        if (Shooter.Friction_Speed_Set > FRICTION_SPEED)
        {
            Shooter.Friction_Speed_Set
                = FRICTION_SPEED + 0.05 * Shooter.Stop_Fire_CNT;
            CLAMP(Shooter.Friction_Speed_Set,
                FRICTION_SPEED,
                FRICTION_SPEED + 200);
        }
        if (Shooter.Stop_Fire_CNT > 0)
            Shooter.Stop_Fire_CNT--;
    }
}

uint8_t Shooter_Cooling_Judge(void)
{
    if (judge_rece_mesg.game_robot_state.shooter_barrel_heat_limit
            - Shooter.Heat
        <= 20)
    {
        return 1;
    }
    else if (judge_rece_mesg.game_robot_state.shooter_barrel_heat_limit
            - Shooter.Heat
        >= 30)
    {
        return 0;
    }
    else
        return 0;
}


float Shooter_Bullet_Speed_Self_Adaptation(float Bullet_Speed)
{
    float static bullet_k = BULLET_SPEED_SELF_ADAPTATION_K;
    float static Bullet_Speed_Error;
    Bullet_Speed_Error = (BULLET_SPEED_TARGET - Bullet_Speed);
    float error_calib  = Bullet_Speed_Error * bullet_k;
    if (error_calib > 200)
        return 200;
    else if (error_calib < -1200)
        return -1200;
    else
        return error_calib;
}
float First_Order_Kalman_Filter_Cal(
    First_Order_Kalman_Filter_t* _First_Order_Kalman_Filter,
    float                        _Z /*测量值*/)
{
    // 更新上一次的值
    _First_Order_Kalman_Filter->Error_Est_Last
        = _First_Order_Kalman_Filter->Error_Est;
    _First_Order_Kalman_Filter->X_hat_Last = _First_Order_Kalman_Filter->X_hat;
    // 更新上一次的值
    _First_Order_Kalman_Filter->Error_Est_Last
        = _First_Order_Kalman_Filter->Error_Est;
    _First_Order_Kalman_Filter->X_hat_Last = _First_Order_Kalman_Filter->X_hat;
    // Step1:Kalman_Gain计算
    _First_Order_Kalman_Filter->Kalman_Gain
        = _First_Order_Kalman_Filter->Error_Est_Last
        / (_First_Order_Kalman_Filter->Error_Est_Last
            + _First_Order_Kalman_Filter->Error_Mea);
    // Step2:计算预测值
    _First_Order_Kalman_Filter->X_hat = _First_Order_Kalman_Filter->X_hat_Last
        + _First_Order_Kalman_Filter->Kalman_Gain
            * (_Z - _First_Order_Kalman_Filter->X_hat_Last);
    // Step3:预测误差更新
    _First_Order_Kalman_Filter->Error_Est
        = (1 - _First_Order_Kalman_Filter->Kalman_Gain)
        * _First_Order_Kalman_Filter->Error_Est_Last;
    // 返回预测值

    return _First_Order_Kalman_Filter->X_hat;
}

// 上限25,23容错
void Shooter_Bullet_Speed_Fbd_Set(Shooter_t* Shooter, float Bullet_Speed)
{
    if (Bullet_Speed < 15)
        Shooter->Bullet_Speed = 23;
    else
        Shooter->Bullet_Speed = Bullet_Speed;

#if BULEET_SPEED_KALMAN_ENABLE == 1
    if (fired_bullet_cnt < 15)
    {
        fired_bullet_cnt++;
        First_Order_Kalman_Filter_Cal(&Shooter->Bullet_Speed_Kalman, 23);
    }
    else
    {
        First_Order_Kalman_Filter_Cal(
            &Shooter->Bullet_Speed_Kalman, Bullet_Speed);
    }
#endif
}

//TODO
u8 Shoot_Cooling_Judge(u16 Shooter_Heat )
{
    u8 static Flag_Coolin = 0;

    uint8_t level  = judge_rece_mesg.game_robot_state.robot_level;
    u16 heat_limit = judge_rece_mesg.game_robot_state.shooter_barrel_heat_limit;
    if ((level > 2)
        && (Gimbal.Mode == GIMBAL_RC || Gimbal.Mode == GIMBAL_KeyMouse_NORMAL))
    {
        if (level > 5)
        {
            if (heat_limit - Shooter_Heat
                < 60)  // judge_rece_mesg.game_robot_state.shooter_barrel_cooling_value)
            {
                Flag_Coolin = 1;
            }
            else if (heat_limit - Shooter_Heat > 85)
            {
                Flag_Coolin = 0;
            }
        }

        else
        {
            if (heat_limit - Shooter_Heat
                < 40)  // judge_rece_mesg.game_robot_state.shooter_barrel_cooling_value)
            {
                Flag_Coolin = 1;
            }
            else if (heat_limit - Shooter_Heat > 55)
            {
                Flag_Coolin = 0;
            }
        }
    }

    else
    {
        if (level > 2)
        {
            if (heat_limit - Shooter_Heat
                < 40)  // judge_rece_mesg.game_robot_state.shooter_barrel_cooling_value)
            {
                Flag_Coolin = 1;
            }
            else if (heat_limit - Shooter_Heat > 55)
            {
                Flag_Coolin = 0;
            }
        }
        else
        {
            if (heat_limit - Shooter_Heat
                < 25)  // judge_rece_mesg.game_robot_state.shooter_barrel_cooling_value)
            {
                Flag_Coolin = 1;
            }
            else if (heat_limit - Shooter_Heat > 30)
            {
                Flag_Coolin = 0;
            }
        }
    }


    return Flag_Coolin;
}
