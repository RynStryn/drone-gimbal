#include "main.h"

void Gimbal_Mode_Control(Gimbal_t *Gimbal)
{
    switch (Gimbal->Mode)
    {
        //云台初始化时不断尝试使能达妙电机，
        //直到收到达妙电机的反馈(说明电机已上电)
        case GIMBAL_INIT:
            if (DM4310_Pitch.ERR == 0x01)
            {
                Gimbal->Flag_Init_Finish = 1;
                DM4310_Init(
                    DM4310_Pitch_ID_Send,
                    DM4310_Mode_PV,
                    DM4310_Init_Disable);
            } else
            {
                DM4310_Init(
                    DM4310_Pitch_ID_Send,
                    DM4310_Mode_PV,
                    DM4310_Init_Enable);
            }
            if (CH040_Data.Init_Flag == 1 && Gimbal->Flag_Init_Finish == 1)
            {
                Gimbal->Mode = GIMBAL_RELAX;
            }
            break;
        case GIMBAL_RELAX:
            //云台放松，将实际值作为设定值，防跳变
            if (Gimbal->Mode != Gimbal->Mode_Last)
            {
                //_sgn
                Gimbal->Pitch_Angle_Set = -Gimbal->Pitch_Angle_Actual;
                Gimbal->Yaw_Angle_Set   = Gimbal->Yaw_Angle_Actual;
            }
            //失能达妙
            DM4310_Init(DM4310_Pitch_ID_Send,
                        DM4310_Mode_PV,
                        DM4310_Init_Disable);
            //PID清零
            Gimbal_PID_Clear();
            break;
        case GIMBAL_STOP:
            //云台静止（暂时用不到）
            if (Gimbal->Mode != Gimbal->Mode_Last)
            {
                Gimbal->Pitch_Angle_Set = -Gimbal->Pitch_Angle_Actual;
                Gimbal->Yaw_Angle_Set   = Gimbal->Yaw_Angle_Actual;
            }

            break;
        case GIMBAL_REMOTE:
            if (Gimbal->Mode != Gimbal->Mode_Last
                /*||DM4310_Pitch.ERR!=0x01*/)
            {
                Gimbal->Pitch_Angle_Set = -Gimbal->Pitch_Angle_Actual;
                Gimbal->Yaw_Angle_Set   = Gimbal->Yaw_Angle_Actual;
                DM4310_Init(
                    DM4310_Pitch_ID_Send,
                    DM4310_Mode_PV,
                    DM4310_Init_Enable);
            }
            //积分设置角度
            Gimbal_Control_Task_Inc(Gimbal,
                                    -(Remote_data.Remote_Control.ch2 - 1024)
                                    * Remote_Sensitivity,
                                    -(Remote_data.Remote_Control.ch3 - 1024)
                                    * Remote_Sensitivity);
            break;
        case GIMBAL_KM_NORMAL:
            if (Gimbal->Mode != Gimbal->Mode_Last
                /*||DM4310_Pitch.ERR!=0x01*/)
            {
                Gimbal->Pitch_Angle_Set = -Gimbal->Pitch_Angle_Actual;
                Gimbal->Yaw_Angle_Set   = Gimbal->Yaw_Angle_Actual;
                DM4310_Init(
                    DM4310_Pitch_ID_Send,
                    DM4310_Mode_PV,
                    DM4310_Init_Enable);
            }
            //同理
            Gimbal_Control_Task_Inc(Gimbal,
                                    -(Remote_data.Remote_Mouse.x) *
                                    Mouse_Sensitivity,
                                    -(Remote_data.Remote_Mouse.y) *
                                    Mouse_Sensitivity);
            break;
        default:
            //云台放松，将实际值设为设定值，防跳变
            if (Gimbal->Mode != Gimbal->Mode_Last)
            {
                Gimbal->Pitch_Angle_Set = -Gimbal->Pitch_Angle_Actual;
                Gimbal->Yaw_Angle_Set   = Gimbal->Yaw_Angle_Actual;
            }
            //失能达妙
            DM4310_Init(DM4310_Pitch_ID_Send,
                        DM4310_Mode_PV,
                        DM4310_Init_Disable);
            //PID清零
            Gimbal_PID_Clear();
            break;
    }
}

void Shooter_Mode_Control(Shooter_t *Shooter)
{
    //		if(Shooter_Cooling_Judge())//热量限制
    //		{
    //			Shooter->Poke_State=POKE_OFF;
    //		}
    //停火校验（停火时缓慢增加摩擦轮转速，减少因为摩擦轮冷却收缩导致的弹速大幅度降低）
    Stop_Fire_Judge();
    if (Shooter->Mode == SHOOTER_INIT)
    {
        if (Shooter->Poke_Angle_Act != 0 || DJI2006_Pock.N_m != 0)
        {
            Shooter->Into_Single_Mode_Flag        = 1;
            Shooter->Into_Single_Mode_Flag_Finish = 0;
        }
        if (Shooter->Into_Single_Mode_Flag == 1 && Shooter->
            Into_Single_Mode_Flag_Finish == 0)
        {
            //拨盘角度清零
            Shooter_Angle_Clear();
        }
        if (Shooter->Into_Single_Mode_Flag == 1 && Shooter->
            Into_Single_Mode_Flag_Finish == 1)
        {
            Shooter->Mode = RUNNING_FIRE;
        }
    }
    if (Shooter->Friction_State == Friction_ON)
    {
        switch (Shooter->Mode)
        {
            case RUNNING_FIRE:
                //					if(	Shooter->Mode_Last!=Shooter->Mode)
                //					{
                //						Shooter->Into_Single_Mode_Flag=0;
                //						Shooter->Into_Single_Mode_Flag_Finish=0;
                //					}
                //拨盘卡弹校验
                Poke_Get_Trap_Judge();
                //摩擦轮卡弹校验
                Friction_Get_Trap_Judge();
                //拨盘卡弹反转
                if (Shooter->State == Poke_Get_Trap)
                {
                    Shooter_Speed_Set_Get_Trap(
                        Shooter->Friction_Speed_Set +
                        Shooter_Bullet_Speed_Self_Adaptation(
                            Shooter->Bullet_Speed_Kalman.X_hat),
                        -20);
                }
                //摩擦轮卡弹则反转
                else if (Shooter->State == Friction_Get_Trap)
                {
                    Shooter_Speed_Set_Get_Trap(-12000, 0);
                }
                //停火
                else if (Shooter->Poke_State == POKE_OFF)
                {
                    Shooter_Speed_Set_Running(
                        Shooter->Friction_Speed_Set +
                        Shooter_Bullet_Speed_Self_Adaptation(
                            Shooter->Bullet_Speed_Kalman.X_hat),
                        0);
                    Shooter->Running_Fire_CNT = 0;
                }
                //开火
                else if (Shooter->State == START && (
                             DJI2006_Fric[0].speed_rpm > FRICTION_SPEED_MIN
                             && DJI2006_Fric[1].speed_rpm >
                             FRICTION_SPEED_MIN && DJI2006_Fric[2].speed_rpm
                             > FRICTION_SPEED_MIN))
                {
                    Shooter_Speed_Set_Running(
                        Shooter->Friction_Speed_Set +
                        Shooter_Bullet_Speed_Self_Adaptation(
                            Shooter->Bullet_Speed_Kalman.X_hat),
                        Shooter->Freq);
                }
                break;
            case SINGLE_SHOT: {
                Poke_Get_Trap_Judge();
                Friction_Get_Trap_Judge();
                if (Shooter->State == Poke_Get_Trap)
                {
                    Shooter_Speed_Set_Get_Trap(
                        Shooter->Friction_Speed_Set +
                        Shooter_Bullet_Speed_Self_Adaptation(
                            Shooter->Bullet_Speed_Kalman.X_hat),
                        -20);
                } else if (Shooter->State == Friction_Get_Trap)
                {
                    Shooter_Speed_Set_Get_Trap(-12000, -1);
                } else if (Shooter->Poke_State == POKE_ON && (
                               DJI2006_Fric[0].speed_rpm >
                               FRICTION_SPEED_MIN && DJI2006_Fric[1].
                               speed_rpm > FRICTION_SPEED_MIN &&
                               DJI2006_Fric[2].speed_rpm >
                               FRICTION_SPEED_MIN))
                {
                    Shooter_Speed_Set_Single(
                        Shooter->Friction_Speed_Set +
                        Shooter_Bullet_Speed_Self_Adaptation(
                            Shooter->Bullet_Speed_Kalman.X_hat),
                        1);
                } else if (Shooter->Poke_State == POKE_OFF)
                {
                    Shooter_Speed_Set_Single(
                        Shooter->Friction_Speed_Set +
                        Shooter_Bullet_Speed_Self_Adaptation(
                            Shooter->Bullet_Speed_Kalman.X_hat),
                        0);
                }
            }
            break;
            case SHOOTER_INIT:

                break;
            default:
                Shooter_PID_Clear();
                Shooter->Poke_Get_Trap_cnt     = 0;
                Shooter->Friction_Get_Trap_Cnt = 0;
                break;
        }
    } else
    {
        Shooter_Speed_Set_Running(0, 0);
        Shooter->Poke_Get_Trap_cnt     = 0;
        Shooter->Friction_Get_Trap_Cnt = 0;
    }
}

void Mode_Control(Gimbal_t *Gimbal, Shooter_t *Shooter)
{
    //	static volatile int DM_Control_Flag;
    //	DM_Control_Flag++;
    //	if(DM_Control_Flag>=10000)
    //	{
    //		DM_Control_Flag=1;
    //	}
    Gimbal_Mode_Control(Gimbal);
    Shooter_Mode_Control(Shooter);


    Gimbal->Mode_Last  = Gimbal->Mode;
    Shooter->Mode_Last = Shooter->Mode;
}
