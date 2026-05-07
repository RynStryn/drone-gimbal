#ifndef __MAIN_H__
#define __MAIN_H__

/***without RC***/
//#define TEST
//#define YAW_TEST

//#define PITCH_TEST

#define DM4310_MIT
//#define DM4310_V
/***a new motor***/
//#define DJI6020_TEST
//#define DM4310_TEST
//#define DJI2006_TEST

#define DJI6020_YAW_ZERO_ENCODER 176.0f//236-30-30
#define DJI6020_YAW_HOMING_DIR   1
#define DJI6020_YAW_HOMING_TOLERANCE 0.5f
//#define pitchHoming
// #define DM4310_Pitch_ZERO_ENCODER -60.0f
// #define DM4310_Pitch_HOMING_DIR   1
// #define DM4310_Pitch_HOMING_TOLERANCE 0.5f

#define DJI6020_Yaw_ID 0x208
//ID.5:0x205+4
#define DJI6020_Yaw_Send_ID 0x1FE
//I_loop 0x1FE;0x2FE for 5~7
#define DATAPOS 6

//#define CANoff



#define BULEET_SPEED_KALMAN_ENABLE 1

#ifndef PI
#define PI 3.14159265358979f
#endif

//角度转弧度的比例
#ifndef DEG2RAD
#define DEG2RAD 0.01745329251994329576923690768489f
#endif

//弧度转角度的比例
#ifndef RAD2DEG
#define RAD2DEG 57.295779513082320876798154814105f
#endif



//数值限制
#define CLAMP(val, min, max) do{\
if(val<min)\
{\
	val = min;\
}\
else if(val>max)\
{\
	val = max;\
}\
}while (0)

#endif
#include "stm32f4xx.h"
#include "string.h"     // memcopy()
#include "math.h"		// fabs()
#include "stdlib.h"		//abs()
/*************↓硬件↓*************/

#include "Motor_DM4310.h"
#include "Motor_DJI6020.h"
#include "Motor_DJI2006.h"
#include "CAN1.h"
#include "CAN2.h"
#include "Timer.h"
#include "USART1.h"
#include "CH040.h"
#include "Judge_System.h"
#include "VTM_Remote.h"

/*************↑硬件↑*************/

/*************↓任务↓*************/
#include "Init_Task.h"
#include "Remote_Control_Task.h"
#include "Gimbal_Task.h"
#include "Control_Task.h"
#include "Mode_Task.h"
#include "Shoot_Task.h"
#include "Fault_Diagnosis_Task.h"
/*************↑任务↑*************/

/*************↓算法↓*************/
#include "PID.h"
#include "LESO.h"
#include "signal.h"
#include "data_sample.h"
#include "gravity.h"
/*************↑算法↑*************/
#include "00debug.h"
