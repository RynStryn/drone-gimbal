//
// Created by RynStryn on 2026/4/12.
//

#include "calibration_data.h"
#include "main.h"


const float calib_angles[CALIB_POINTS_COUNT] = {
    -8.596541f,     -8.287765f,     -7.978988f,     -7.670211f,     -7.361435f,     -7.052658f,     -6.743882f,     -6.435105f,     -6.126328f,     -5.817552f,     -5.508775f,     -5.199998f,     -4.891222f,     -4.582445f,     -4.273669f,     -3.964892f,     -3.656115f,     -3.347339f,     -3.038562f,     -2.729786f,     -2.421009f,     -2.112232f,     -1.803456f,     -1.494679f,     -1.185903f,     -0.877126f,     -0.568349f,     -0.259573f,     0.049204f,     0.357981f,     0.666757f,     0.975534f,     1.284310f,     1.593087f,     1.901864f,     2.210640f,     2.519417f,     2.828193f,     3.136970f,     3.445747f,     3.754523f,     4.063300f,     4.372076f,     4.680853f,     4.989630f,     5.298406f,     5.607183f,     5.915960f,     6.224736f,     6.533513f,     6.842289f,     7.151066f,     7.459843f,     7.768619f,     8.077396f,     8.386172f,     8.694949f,     9.003726f,     9.312502f,     9.621279f,     9.930056f,     10.238832f,     10.547609f,     10.856385f,     11.165162f,     11.473939f,     11.782715f,     12.091492f,     12.400268f,     12.709045f,     13.017822f,     13.326598f,     13.635375f,     13.944151f,     14.252928f,     14.561705f,     14.870481f,     15.179258f,     15.488035f,     15.796811f,     16.105588f,     16.414364f,     16.723141f,     17.031918f,     17.340694f,     17.649471f,     17.958247f,     18.267024f,     18.575801f,     18.884577f,     19.193354f,     19.502131f,     19.810907f,     20.119684f,     20.428460f,     20.737237f,     21.046014f,     21.354790f,     21.663567f, 21.972343f
};

const float calib_torques[CALIB_POINTS_COUNT] = {
    -0.899643f,     -0.820829f,     -0.742015f,     -0.717192f,     -0.697986f,     -0.678780f,     -0.684822f,     -0.702680f,     -0.720537f,     -0.737059f,     -0.724019f,     -0.704703f,     -0.685727f,     -0.666689f,     -0.630195f,     -0.601011f,     -0.599455f,     -0.623831f,     -0.648207f,     -0.655352f,     -0.642755f,     -0.621896f,     -0.619812f,     -0.621341f,     -0.593533f,     -0.561240f,     -0.545969f,     -0.566844f,     -0.586727f,     -0.596526f,     -0.603982f,     -0.594838f,     -0.590084f,     -0.585330f,     -0.565453f,     -0.544318f,     -0.524823f,     -0.539620f,     -0.539023f,     -0.538861f,     -0.540117f,     -0.542553f,     -0.532392f,     -0.522231f,     -0.507437f,     -0.499653f,     -0.491870f,     -0.479786f,     -0.467548f,     -0.455311f,     -0.459879f,     -0.464184f,     -0.466310f,     -0.468939f,     -0.482391f,     -0.491347f,     -0.500303f,     -0.487746f,     -0.437076f,     -0.381004f,     -0.378773f,     -0.400425f,     -0.426134f,     -0.451843f,     -0.443845f,     -0.435349f,     -0.426852f,     -0.402175f,     -0.377445f,     -0.352750f,     -0.361497f,     -0.372390f,     -0.379906f,     -0.383868f,     -0.384688f,     -0.386985f,     -0.389281f,     -0.384381f,     -0.383737f,     -0.383093f,     -0.341869f,     -0.271532f,     -0.196195f,     -0.205932f,     -0.256728f,     -0.308098f,     -0.359468f,     -0.335180f,     -0.300364f,     -0.280827f,     -0.311626f,     -0.342030f,     -0.372433f,     -0.381901f,     -0.252761f,     -0.123620f,     -0.137781f,     -0.200205f,     -0.262629f, -0.325053f
};


/**
 * @brief 线性插值查表函数
 * @param input_angle 实时测得的角度
 * @return float 计算出的平滑扭矩
 */
#include "calibration_data.h"

/**
 * @brief 线性插值与外推函数
 * @param input_angle 实时测得的角度
 * @return float 计算出的平滑扭矩（超出校准范围时进行线性外推）
 */
float Get_Interpolated_Torque(float input_angle) {
    int i = 0;
    int low = 0;
    int high = CALIB_POINTS_COUNT - 2;
    int index = 0;

    // 1. Boundary Handling (Clamping input or handling extrapolation)
    if (input_angle <= calib_angles[0]) {
        index = 0;
    } else if (input_angle >= calib_angles[CALIB_POINTS_COUNT - 1]) {
        index = CALIB_POINTS_COUNT - 2;
    } else {
        // 2. Binary Search for Interval (O(log N))
        while (low <= high) {
            int mid = low + (high - low) / 2;
            if (input_angle >= calib_angles[mid] && input_angle <= calib_angles[mid + 1]) {
                index = mid;
                break;
            } else if (input_angle < calib_angles[mid]) {
                high = mid - 1;
            } else {
                low = mid + 1;
            }
        }
    }

    // 3. Perform Linear Interpolation
    float x0 = calib_angles[index];
    float x1 = calib_angles[index + 1];
    float y0 = calib_torques[index];
    float y1 = calib_torques[index + 1];

    // Linear formula: y = y0 + (x - x0) * (y1 - y0) / (x1 - x0)
    // Note: (x1 - x0) is pre-calculated by MATLAB's linspace as a constant delta
    float output_torque = y0 + (input_angle - x0) * (y1 - y0) / (x1 - x0);

    // 4. Safety Clamp
    if (output_torque > 1.4f)  output_torque = 1.4f;
    if (output_torque < -1.4f) output_torque = -1.4f;

    return output_torque;
}