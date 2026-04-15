//
// Created by RynStryn on 2026/4/11.
//

#include "gravity.h"
#include <stdint.h>
#include <math.h>
#include "PID.h"
#include "Gimbal_Task.h"


typedef enum {
    Sample_IDLE,
    Sample_MOVING,
    Sample_RECORDING,
    Sample_DONE
} SampleState_e;


float start_angle = -20.0f;
float end_angle = 10.0f;
float step_angle = 1.0f;
float Angle = -20.0f;
uint8_t   sample_times = 5;    // 每个点采集次数
uint8_t isOK = 0;
// 计算，预留多一点防止溢出
#define MAX_SAMPLE_POINTS 35

typedef struct {
    float angle;
    float torque;
} SampleData_t;

SampleData_t gravity_sample_data[MAX_SAMPLE_POINTS];
int sample_data_count = 0; // 当前采集了多少个点
// -------------------------------------------------------------

void Pid(float* Angle)
{
    PID_Cal(&DM4310_PID_P, *Angle, Gimbal.Pitch_Angle_Get);
    float Speed = DM4310_PID_P.Out;
    CLAMP(Speed, -10.0f, 10.0f);
    PID_Cal(&DM4310_PID_V, Speed, Gimbal.Pitch_Speed_Get);
    DM4310_MIT_Ctrl(DM4310_Pitch_ID_Send, 0, 0, 0, 0, DM4310_PID_V.Out);

}
void Auto_Gravity_Data_Collection() {
    static SampleState_e state = Sample_IDLE;
    static float Angle = -20.0f;
    static uint32_t timer = 0;
    static float torque_sum = 0;
    static uint8_t move_fin = 0;
    static uint8_t point_sample_fin = 0;
    static uint32_t timeStamp = 0;

timer++;
    switch (state) {
        case Sample_IDLE:
            if (isOK) {
                Angle = start_angle;
                sample_data_count = 0; // 每次重新开始时，清空数组计数器
                state = Sample_MOVING;
            }
            break;

        case Sample_MOVING:
            Pid(&Angle);
            if (fabs(Gimbal.Pitch_Angle_Get - Angle) < 0.5f) {
                    move_fin++;
                if (move_fin >= 10)
                {
                    state = Sample_RECORDING;
                    timeStamp = timer;
                }
            }
            break;

        case Sample_RECORDING:
            Pid(&Angle);

            torque_sum += DM4310_PID_V.Out;
            point_sample_fin++;

            if (point_sample_fin >= sample_times)
            {
                float avg_torque = torque_sum / sample_times;

                if (sample_data_count < MAX_SAMPLE_POINTS)
                {
                    gravity_sample_data[sample_data_count].angle  = Angle;
                    gravity_sample_data[sample_data_count].torque = avg_torque;
                    sample_data_count++;
                }

                Angle += step_angle;
                if (Angle > end_angle)
                {
                    state = Sample_DONE;
                }
                else
                {
                    state = Sample_MOVING;
                }
            }
            break;

        case Sample_DONE:
             state = Sample_IDLE;
            break;
    }

}