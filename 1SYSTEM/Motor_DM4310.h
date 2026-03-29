#ifndef __MOTOR_DM4310_H__
#define __MOTOR_DM4310_H__

#include <stm32f4xx.h>

#define DM4310_Pitch_ID_Recieve 0x11
#define DM4310_Pitch_ID_Send 0x02

typedef enum
{
	DM4310_Init_Enable=0,
	DM4310_Init_Disable,
	DM4310_Init_Set_Zero_Position,
	DM4310_Init_Clear_Error,
}DM4310_Init_State;

typedef enum
{
	DM4310_Mode_MIT=0,
	DM4310_Mode_PV,
	DM4310_Mode_V,
}eDM4310_Mode;

/********************************************************************************
  * ID	表示电机的ID（可在达妙调试助手中修改电机ID）
  * ERR 表示电机状态：	
  *	 0??失能			B??MOS过温
  *  1??使能			C??电机线圈过温
  *  8??超压			D??通讯丢失
  *  9??欠压			E??过载
  *  A??过电流
  * POS 表示电机的位置信息
  * VEL 表示电机的速度信息
  * T 表示电机的扭矩信息
  * T_MOS表示驱动上MOS的平均温度，单位℃
  * T_Rotor 表示电机内部线圈的平均温度，单位℃
*********************************************************************************/
typedef struct {
    uint8_t ID;
    /* ERR 表示电机状态：
     *  0??失能			B??MOS过温
     *  1??使能			C??电机线圈过温
     *  8??超压			D??通讯丢失
     *  9??欠压			E??过载
     *  A??过电流
     */
    uint8_t ERR;

    float POS;
    float VEL;
	float T;
	
	float T_MOS;
	float T_Rotor;
	
	float Init_deg;
    float Init_Flag;
	float Acked;
}DM4310_t;
 
extern DM4310_t DM4310_Pitch;

void DM4310_Receive_Data(DM4310_t* DM4310_Pitch,uint8_t *DATA);
void DM4310_V_Ctrl(uint8_t ID,float _vel);
void DM4310_PV_Ctrl(uint8_t ID, float _pos, float _velMax);
void DM4310_MIT_Ctrl(uint8_t ID, float pos, float vel,float kp, float kd, float torq);
void DM4310_Init(uint8_t ID,eDM4310_Mode Mode,DM4310_Init_State State);


#endif
