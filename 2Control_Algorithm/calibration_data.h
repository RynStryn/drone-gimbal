#ifndef INC_26DRONEMSTD_CALIBRATION_DATA_H
#define INC_26DRONEMSTD_CALIBRATION_DATA_H

#define CALIB_POINTS_COUNT 100

// 仅声明变量和函数
extern const float calib_angles[CALIB_POINTS_COUNT];
extern const float calib_torques[CALIB_POINTS_COUNT];

float Get_Interpolated_Torque(float input_angle);

#endif // INC_26DRONEMSTD_CALIBRATION_DATA_H