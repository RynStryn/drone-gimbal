#include "main.h"
//即使overflow也是从0计数
uint32_t  Control_timer = 0;
//u8 sys_isSafe=1;
uint8_t EverythingAllRight=1;
int8_t last_s2;
int16_t x;
uint32_t t;
void Control_Task()
{
    #if 1
    {    Gimbal.power_on=1;
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
        if((Peripheral_State.Remote_Control.Link_State==CONNECTED &&Peripheral_State.VTM_Remote.Link_State==CONNECTED)
        ||(Peripheral_State.Remote_Control.Link_State==DISCONNECTED&&Peripheral_State.VTM_Remote.Link_State==DISCONNECTED))
        {
            RemoteCtrl_Mode=RC_RELAX;
        }
        else if(Peripheral_State.Remote_Control.Link_State==CONNECTED)
        {
            RC_Type_e=DR16_Remote;
            Rmt_Input_Mode();
        }
        else
        {
            RC_Type_e=VTM_Remote;
            Rmt_Input_Mode();
            VTM_Switch_Action_Get();
        }
        Cmd_Get();  // 遥控键鼠数据

#ifdef TEST
        Peripheral_State.Gimbal_Yaw_Gyro.Link_State=CONNECTED;
        Peripheral_State.Remote_Control.Link_State=CONNECTED;
#endif
        Poke_Ang_Spd_Get(&Shooter);
        Friction_Spd_Get(&Shooter);

        Gimbal_Ang_Spd_Get(&Gimbal, &CH040_Data, &DJI6020_Yaw, &DM4310_Pitch);
        RmtCtrl_Input_Interpreter();
        Main_Task(&Gimbal, &Shooter);
#endif
    }
#endif
    // Friction_Spd_Get(&Shooter);
    // PID_Cal(&DJI2006_Fric_PID_V_3, x, Shooter.Friction_Speed_3_Get);
    t+=1;
    x=1000*sin(0.00314*t);
    uint8_t DATA[8];
    DATA[0] = ((int) 0 >> 8) & 0xFF;
    DATA[1] = (int) 0;
    DATA[2] = ((int) 0 >> 8) & 0xFF;
    DATA[3] = (int) -0;
    DATA[4] = ((int) x >> 8) & 0xFF;
    DATA[5] = (int) x;
    DATA[6] = ((int) 0 >> 8) & 0xFF;
    DATA[7] = (int) 0;
    CAN1_Transmit(0x200, 8, DATA);
}
