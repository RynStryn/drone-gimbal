#include "main.h"

Peripheral_State_t Peripheral_State;


/**********************
*@Brief:设备连接状态设置函数
*@Call:内部或外部
*@Param:设备状态结构体
                连接状态结构体
*@Note:无
*@RetVal:无
**********************/
void Link_State_Set(Equipment_State_t* Equipment_State, Link_State_e Link_State)
{
    Equipment_State->Link_State = Link_State;
}

/**********************
 *@Brief:设备计数器清零函数
 *@Call:内部或外部
 *@Param:设备状态结构体
 *@Note:在相应中断中调用此函数，当计数器超过一定值时说明改设备连接异常。
 *@RetVal:无
 **********************/
void Equipment_Cnt_Clear(Equipment_State_t* Equipment_State)
{
    Equipment_State->Counter = 0;
}


/**********************
 *@Brief:设备状态初始化函数
 *@Call:内部或外部
 *@Param:
 *@Note:无
 *@RetVal:无
 **********************/
void Peripheral_State_Init(Peripheral_State_t* Peripheral_State)
{
    Peripheral_State->Remote_Control.Link_State = CONNECTED;
    Peripheral_State->Remote_Control.Counter    = 0;

    Peripheral_State->Gimbal_Yaw_Gyro.Link_State = CONNECTED;
    Peripheral_State->Gimbal_Yaw_Gyro.Counter    = 0;

    Peripheral_State->Equipment_Visual_Equipment_Auto_Aim.Link_State = CONNECTED;
    Peripheral_State->Equipment_Visual_Equipment_Auto_Aim.Counter    = 0;
}
/**********************
*@Brief:设备状态判断函数
*@Call:内部或外部
*@Param:Equipment_State：设备状态结构体
                T,函数执行周期，单位：秒
*@Note:无
*@RetVal:无
**********************/
void Equipment_State_Judge(Equipment_State_t* Equipment_State, float T)
{

    Equipment_State->Counter++;
    if (Equipment_State->Counter * T > 1)
        Equipment_State->Link_State = DISCONNECTED;
    else
        Equipment_State->Link_State = CONNECTED;
}
/**********************
 *@Brief:函数
 *@Call:内部或外部
 *@Param:
 *@Note:无
 *@RetVal:无
 **********************/
void Peripheral_State_Judge(Peripheral_State_t* Peripheral_State, float T)
{

    Equipment_State_Judge(&Peripheral_State->Remote_Control, T);

    Equipment_State_Judge(&Peripheral_State->Gimbal_Yaw_Gyro, T);
	
    Equipment_State_Judge(&Peripheral_State->Equipment_Visual_Equipment_Auto_Aim, T);
	
	Equipment_State_Judge(&Peripheral_State->VTM_Remote, T);
}
void Offline_Protect(void)
{
        RemoteCtrl_Mode    = OFFLINE;
        Gimbal.Mode = GIMBAL_OFF;
        Gimbal.power_on = 0;
        Shooter.Friction_State =FRICTION_OFF;
#if 0
    DM4310_Init(
        DM4310_Pitch_ID_Send, DM4310_Mode_MIT, DM4310_Init_Disable);
#    ifdef DM4310_V
    DM4310_Init(
        DM4310_Pitch_ID_Send, DM4310_Mode_V, DM4310_Init_Disable);
#    endif
#endif
        PID_AllClear();
}
