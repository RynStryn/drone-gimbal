#include "main.h"
//即使overflow也是从0计数
uint32_t  Control_timer = 0;
//u8 sys_isSafe=1;
uint8_t EverythingAllRight=1;
int8_t last_s2;
void Control_Task()
{
    Gimbal.power_on=1;
    Control_timer++;
#ifdef DJI2006_TEST
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
    Cmd_Get();  // 遥控数据

#ifdef TEST
    Peripheral_State.Gimbal_Yaw_Gyro.Link_State=CONNECTED;
    Peripheral_State.Remote_Control.Link_State=CONNECTED;
#endif
    Poke_Ang_Spd_Get(&Shooter);
    Friction_Spd_Get(&Shooter);

        Gimbal_Ang_Spd_Get(&Gimbal, &CH040_Data, &DJI6020_Yaw, &DM4310_Pitch);
        RmtCtrl_Input_Interpreter();
    Auto_Gravity_Data_Collection();
    Main_Task(&Gimbal, &Shooter);
#endif
}
