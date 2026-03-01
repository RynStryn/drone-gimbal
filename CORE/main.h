#ifndef __MAIN_H__
#define __MAIN_H__

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
/*************↑算法↑*************/


#ifndef PI
#define PI 3.14159265358979f
#endif

//角度转弧度的比例
#ifndef ANGLE_TO_RAD
#define ANGLE_TO_RAD 0.01745329251994329576923690768489f
#endif

//弧度转角度的比例
#ifndef RAD_TO_ANGLE
#define RAD_TO_ANGLE 57.295779513082320876798154814105f
#endif

//是否使能卡尔曼滤波
#define BULEET_SPEED_FILTER_ENABLE 1

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
