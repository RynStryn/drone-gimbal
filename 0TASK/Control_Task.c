#include "main.h"
int  Control_timer = 0;
    //float out[3];
void Control_Task()
{
    Control_timer++;
#ifdef DJI2006_TEST
    // DJI2006_Fric_PID_V_1.Out=out[0];
    // DJI2006_Fric_PID_V_2.Out=out[1];
    // DJI2006_Fric_PID_V_3.Out=out[2];
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
#else
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

#endif
    if (Control_timer >= 10000)
    {
        Control_timer = 0;
    }
}
