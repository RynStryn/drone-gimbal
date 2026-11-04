#include "main.h"
int  Control_timer = 0;
void Control_Task()
{
    Control_timer++;

    Peripheral_State_Judge(&Peripheral_State, 0.001);  // 模块心跳检测

    Remote_Control();  // 遥控数据

    if (Peripheral_State.Gimbal_Yaw_Gyro.Link_State
        == NORMAL)  // 陀螺仪连接正常
    {
        Gimbal_Get_Angle_Data(
            &Gimbal, &CH040_Data, &DJI6020_Yaw, &DM4310_Pitch);
        Poke_Data_Get(&Shooter);
        Friction_Speed_Get(&Shooter);
    } else  // 异常:失能
    {
        Gimbal.Mode            = GIMBAL_RELAX;
        Shooter.Poke_State     = POKE_OFF;
        Shooter.Friction_State = Friction_OFF;
    }
    Mode_Control(&Gimbal, &Shooter);  // 对应模式下的任务处理

    if (Control_timer >= 10000)
    {
        Control_timer = 0;
    }
}
