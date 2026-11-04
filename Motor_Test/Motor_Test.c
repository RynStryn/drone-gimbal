/**
 * @file    Motor_Test.c
 * @brief   电机测试文件 ? CAN ID 与发送指令实现
 *
 * 本文件集中了项目中 3 种电机的 CAN 通信控制函数，
 * 方便单独调试各电机而无需引用整个项目框架。
 */

#include "Motor_Test.h"

/* 外部 CAN 发送函数声明（由项目中的 CAN1.c / CAN2.c 提供） */
extern void CAN1_Transmit(uint32_t ID, uint8_t Length, uint8_t *Data);
extern void CAN2_Transmit(uint32_t ID, uint8_t Length, uint8_t *Data);

/*============================================================================
 *  内部工具函数
 *============================================================================*/

/**
 * @brief   无符号整数 → 浮点数（线性映射）
 */
static float _uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/**
 * @brief   浮点数 → 无符号整数（线性映射）
 */
static int _float_to_uint(float x_float, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}

/*============================================================================
 *  DM4310 (CAN2) ? 达妙电机
 *============================================================================*/

void MT_DM4310_Init(uint8_t ID, MT_DM4310_Mode_t mode, MT_DM4310_InitState_t state)
{
    uint8_t DATA[8];
    memset(DATA, 0xFF, sizeof(DATA));

    uint16_t ID_offset = 0;
    switch (mode) {
        case MT_DM4310_MODE_MIT: ID_offset = MT_DM4310_MODE_OFFSET_MIT; break;
        case MT_DM4310_MODE_PV:  ID_offset = MT_DM4310_MODE_OFFSET_PV;  break;
        case MT_DM4310_MODE_V:   ID_offset = MT_DM4310_MODE_OFFSET_V;   break;
        default:                  ID_offset = 0;                         break;
    }

    switch (state) {
        case MT_DM4310_INIT_ENABLE:
            DATA[7] = MT_DM4310_CMD_ENABLE;
            CAN2_Transmit(ID + ID_offset, 8, DATA);
            break;
        case MT_DM4310_INIT_DISABLE:
            DATA[7] = MT_DM4310_CMD_DISABLE;
            CAN2_Transmit(ID + ID_offset, 8, DATA);
            break;
        case MT_DM4310_INIT_SET_ZERO:
            DATA[7] = MT_DM4310_CMD_SET_ZERO;
            CAN2_Transmit(ID + ID_offset, 8, DATA);
            break;
        case MT_DM4310_INIT_CLEAR_ERROR:
            DATA[7] = MT_DM4310_CMD_CLEAR_ERROR;
            CAN2_Transmit(ID + ID_offset, 8, DATA);
            break;
        default:
            DATA[7] = MT_DM4310_CMD_DISABLE;  /* 未知状态则失能 */
            CAN2_Transmit(ID + ID_offset, 8, DATA);
            break;
    }
}

void MT_DM4310_MIT_Ctrl(uint8_t ID, float pos, float vel,
                        float kp, float kd, float torq)
{
    uint8_t DATA[8];
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;

    /* MIT 协议：16bit(pos) + 12bit(vel) + 12bit(kp) + 12bit(kd) + 12bit(torq) */
    vel /= 6.0f;  /* 达妙速度系数 */

    pos_tmp = _float_to_uint(pos,   -3.14f,   3.14f,   16);
    vel_tmp = _float_to_uint(vel,   -120.0f,  120.0f,  12);
    kp_tmp  = _float_to_uint(kp,     0.0f,    500.0f,  12);
    kd_tmp  = _float_to_uint(kd,     0.0f,      5.0f,  12);
    tor_tmp = _float_to_uint(torq,  -10.0f,    10.0f,  12);

    /* 打包 */
    DATA[0] = (pos_tmp >> 8);
    DATA[1] =  pos_tmp;
    DATA[2] = (vel_tmp >> 4);
    DATA[3] = ((vel_tmp & 0x0F) << 4) | (kp_tmp >> 8);
    DATA[4] =  kp_tmp;
    DATA[5] = (kd_tmp >> 4);
    DATA[6] = ((kd_tmp & 0x0F) << 4) | (tor_tmp >> 8);
    DATA[7] =  tor_tmp;

    CAN2_Transmit(ID, 8, DATA);
}

void MT_DM4310_PV_Ctrl(uint8_t ID, float pos, float velMax)
{
    uint8_t DATA[8];
    pos *= DEG2RAD;                       /* deg → rad */
    velMax /= 6.0f;                       /* 达妙速度系数 */
    memcpy(DATA,      &pos,    4);
    memcpy(&DATA[4],  &velMax, 4);
    CAN2_Transmit(ID + MT_DM4310_MODE_OFFSET_PV, 8, DATA);
}

void MT_DM4310_V_Ctrl(uint8_t ID, float vel)
{
    uint8_t DATA[8];
    vel /= 6.0f;                          /* 达妙速度系数 */
    memcpy(DATA, &vel, 4);
    CAN2_Transmit(ID + MT_DM4310_MODE_OFFSET_V, 8, DATA);
}

void MT_DM4310_ParseData(const uint8_t *data, MT_DM4310_Data_t *out)
{
    if (!out || !data) return;

    out->ID      = data[0] & 0x0F;
    out->ERR     = data[0] >> 4;
    out->POS     = _uint_to_float((data[1] << 8) | data[2],     -180.0f,  180.0f, 16);
    out->VEL     = _uint_to_float((data[3] << 4) | (data[4] >> 4), -120.0f, 120.0f, 12);
    out->T       = _uint_to_float(((data[4] & 0x0F) << 8) | data[5], -18.0f, 18.0f, 12);
    out->T_MOS   = (float)data[6];
    out->T_Rotor = (float)data[7];
}

/*============================================================================
 *  DJI6020 (CAN2) ? 大疆 6020
 *============================================================================*/

void MT_DJI6020_CurrentCtrl(int16_t current, uint32_t send_id)
{
    uint8_t DATA[8] = {0};
    DATA[0] = (uint8_t)(current >> 8);
    DATA[1] = (uint8_t)(current & 0xFF);
    CAN2_Transmit(send_id, 8, DATA);
}

void MT_DJI6020_ParseData(const uint8_t *data, MT_DJI6020_Data_t *out)
{
    if (!out || !data) return;

    out->pos_deg         = ((data[0] << 8) | data[1]) / 8192.0f * 360.0f;
    out->speed_rpm       = (int16_t)((data[2] << 8) | data[3]);
    out->I_ofTorq        = (int16_t)((data[4] << 8) | data[5]);
    out->Temp            = (float)data[6];
    out->multi_turns_deg = out->pos_deg;  /* 简易单圈；如需多圈请外部累加 */
}

void MT_DJI6020_Stop(void)
{
    MT_DJI6020_CurrentCtrl(0, MT_DJI6020_ID_YAW_SEND);
}

/*============================================================================
 *  DJI2006 (CAN1) ? 大疆 2006
 *============================================================================*/

void MT_DJI2006_CurrentCtrl(uint32_t can_id, int16_t current)
{
    uint8_t DATA[8] = {0};
    DATA[0] = (uint8_t)(current >> 8);
    DATA[1] = (uint8_t)(current & 0xFF);
    CAN1_Transmit(can_id, 8, DATA);
}

void MT_DJI2006_GroupCtrl(uint32_t can_id,
                          int16_t c1, int16_t c2,
                          int16_t c3, int16_t c4)
{
    uint8_t DATA[8];
    DATA[0] = (uint8_t)(c1 >> 8);
    DATA[1] = (uint8_t)(c1 & 0xFF);
    DATA[2] = (uint8_t)(c2 >> 8);
    DATA[3] = (uint8_t)(c2 & 0xFF);
    DATA[4] = (uint8_t)(c3 >> 8);
    DATA[5] = (uint8_t)(c3 & 0xFF);
    DATA[6] = (uint8_t)(c4 >> 8);
    DATA[7] = (uint8_t)(c4 & 0xFF);
    CAN1_Transmit(can_id, 8, DATA);
}

void MT_DJI2006_ParseData(const uint8_t *data, MT_DJI2006_Data_t *out)
{
    if (!out || !data) return;

    out->pos_deg         = ((data[0] << 8) | data[1]) / 8192.0f * 360.0f;
    out->speed_rpm       = (int16_t)((data[2] << 8) | data[3]);
    out->speed_deg       = out->speed_rpm * 6.0f;
    out->Torq            = (int16_t)((data[4] << 8) | data[5]);
    out->multi_turns_deg = out->pos_deg;  /* 简易单圈；如需多圈请外部累加 */
}

void MT_DJI2006_Stop(uint32_t can_id)
{
    MT_DJI2006_CurrentCtrl(can_id, 0);
}
