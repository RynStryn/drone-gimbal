#ifndef __FAULT_DIAGNOSIS_TASK_H__
#define __FAULT_DIAGNOSIS_TASK_H__
#include "stm32f4xx.h"
typedef enum{//_?
	DISCONNECT          ,//正常
	NORMAL       		,//连接异常
	INACTIVE         	,//关闭状态
}Link_State_e;//连接状态枚举

typedef struct{
	int Counter;//计数器
	Link_State_e Link_State;//设备连接状态
}Equipment_State_t;//设备状态结构体


typedef struct{
	Equipment_State_t Remote_Control;
	Equipment_State_t Gimbal_Yaw_Gyro;//云台yaw轴陀螺仪对应状态
	
	Equipment_State_t Equipment_Visual_Equipment_Auto_Aim;//下云台视觉设备
	Equipment_State_t Equipment_Visual_Equipment_Radar;//雷达视觉运算设备
}Peripheral_State_t;


extern Peripheral_State_t Peripheral_State;


void Link_State_Set(Equipment_State_t* Equipment_State,Link_State_e Link_State);
void Peripheral_State_Init(Peripheral_State_t* Peripheral_State);
void Equipment_Cnt_Clear(Equipment_State_t* Equipment_State);
void Peripheral_State_Judge(Peripheral_State_t* Peripheral_State,float T);



#endif /*_FAULT_DIAGNOSIS_TASK_H_*/













