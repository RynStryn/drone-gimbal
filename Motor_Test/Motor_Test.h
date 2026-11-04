
#ifndef __MOTOR_TEST_H__


#define __MOTOR_TEST_H__

#include <stm32f4xx.h>
#include <string.h>
#include <math.h>

/*============================================================================
 *  电机测试文件 ? CAN ID 与发送指令汇总
 *
 *  本文件摘录自项目各电机驱动模块，集中管理所有电机的：
 *    - CAN 接收 ID（电机反馈）
 *    - CAN 发送 ID（控制指令）
 *    - 发送控制指令的函数
 *
 *  支持以下 3 种电机：
 *    1) 达妙 DM4310      (CAN2)
 *    2) 大疆 DJI6020     (CAN2)
 *    3) 大疆 DJI2006     (CAN1)
 *
 *  使用方法：
 *    1. 在需要使用电机测试的文件中 #include "Motor_Test.h"
 *    2. 确保 CAN1_Init() / CAN2_Init() 已调用
 *    3. 调用 MT_xxx() 函数进行测试
 *============================================================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*=========================== 角度 / 弧度 宏 ================================*/
#ifndef PI
#define PI 3.14159265358979f
#endif
#ifndef DEG2RAD
#define DEG2RAD 0.01745329251994329576923690768489f
#endif
#ifndef RAD2DEG
#define RAD2DEG 57.295779513082320876798154814105f
#endif

/*=========================== 数值限幅宏 ====================================*/
#define MT_CLAMP(val, min, max)  do {               \
    if ((val) < (min))       (val) = (min);         \
    else if ((val) > (max))  (val) = (max);         \
} while(0)

/*============================================================================
 *  一、达妙 DM4310 电机 (CAN2)
 *============================================================================
 *  CAN 总线：CAN2 (GPIOB12-TX, GPIOB13-RX)
 *
 *  ┌────────────┬────────────────────┐
 *  │  用途      │  CAN ID (标准帧)    │
 *  ├────────────┼────────────────────┤
 *  │ 接收(反馈) │  0x11 (Pitch)      │
 *  │ MIT 发送   │  0x02 (Pitch)      │
 *  │ PV 发送    │  0x02 + 0x100      │
 *  │ V  发送    │  0x02 + 0x200      │
 *  └────────────┴────────────────────┘
 *
 *  控制模式：
 *    MIT 模式 ? 位置/速度/刚度/阻尼/力矩
 *    PV  模式 ? 位置 + 最大速度
 *    V   模式 ? 纯速度
 *
 *  初始化指令 (DATA[7])：
 *    0xFC  ? 使能
 *    0xFD  ? 失能
 *    0xFE  ? 当前位置设为零点
 *    0xFB  ? 清除错误
 *============================================================================*/

/* DM4310 CAN ID 定义 */
#define MT_DM4310_ID_PITCH_RECV     0x11   /* 接收（反馈）ID */
#define MT_DM4310_ID_PITCH_SEND     0x02   /* MIT 模式发送 ID */

/* DM4310 模式偏移 */
#define MT_DM4310_MODE_OFFSET_MIT   0x000
#define MT_DM4310_MODE_OFFSET_PV    0x100
#define MT_DM4310_MODE_OFFSET_V     0x200

/* DM4310 初始化命令 (DATA[7]) */
#define MT_DM4310_CMD_ENABLE        0xFC
#define MT_DM4310_CMD_DISABLE       0xFD
#define MT_DM4310_CMD_SET_ZERO      0xFE
#define MT_DM4310_CMD_CLEAR_ERROR   0xFB

/* DM4310 数据类型 */
typedef struct {
    uint8_t  ID;        /* 电机 ID (0x00~0x0F) */
    uint8_t  ERR;       /* 错误码 */
    float    POS;       /* 位置 (deg) */
    float    VEL;       /* 速度 (deg/s) */
    float    T;         /* 扭矩 (N?m) */
    float    T_MOS;     /* MOS 温度 (℃) */
    float    T_Rotor;   /* 线圈温度 (℃) */
} MT_DM4310_Data_t;

/* DM4310 模式枚举 */
typedef enum {
    MT_DM4310_MODE_MIT = 0,
    MT_DM4310_MODE_PV,
    MT_DM4310_MODE_V,
} MT_DM4310_Mode_t;

/* DM4310 初始化状态枚举 */
typedef enum {
    MT_DM4310_INIT_ENABLE       = 0,
    MT_DM4310_INIT_DISABLE,
    MT_DM4310_INIT_SET_ZERO,
    MT_DM4310_INIT_CLEAR_ERROR,
} MT_DM4310_InitState_t;

/*============================================================================
 *  二、大疆 DJI6020 电机 (CAN2)
 *============================================================================
 *  CAN 总线：CAN2 (与 DM4310 共用)
 *
 *  ┌────────────┬────────────────────┐
 *  │  用途      │  CAN ID (标准帧)    │
 *  ├────────────┼────────────────────┤
 *  │ 接收(反馈) │  0x208 (Yaw)       │
 *  │ 发送(电流) │  0x1FE             │
 *  └────────────┴────────────────────┘
 *
 *  控制方式：发送电流值 (-16384 ~ 16384)
 *  反馈数据：位置(0~8191 对应 0~360°)、速度(rpm)、电流、温度
 *============================================================================*/

#define MT_DJI6020_ID_YAW_RECV      0x208   /* Yaw 接收 ID */
#define MT_DJI6020_ID_YAW_SEND      0x1FE   /* Yaw 电流控制发送 ID */

/* DJI6020 数据类型 */
typedef struct {
    float    pos_deg;         /* 单圈位置 (deg) */
    float    multi_turns_deg; /* 多圈位置 (deg) */
    float    speed_rpm;       /* 速度 (rpm) */
    float    I_ofTorq;        /* 力矩电流 */
    float    Temp;            /* 温度 (℃) */
} MT_DJI6020_Data_t;

/*============================================================================
 *  三、大疆 DJI2006 电机 (CAN1)
 *============================================================================
 *  CAN 总线：CAN1 (GPIOA11-RX, GPIOA12-TX)
 *
 *  ┌──────────────┬────────────────────┐
 *  │  用途        │  CAN ID (标准帧)    │
 *  ├──────────────┼────────────────────┤
 *  │ 拨弹电机     │  0x204             │
 *  │ 摩擦轮 1     │  0x201             │
 *  │ 摩擦轮 2     │  0x202             │
 *  │ 摩擦轮 3     │  0x203             │
 *  └──────────────┴────────────────────┘
 *
 *  控制方式：向 ID 发送 8 字节电流数据
 *  电流格式：Data[0..1]=电流1, Data[2..3]=电流2, Data[4..5]=电流3, Data[6..7]=电流4
 *  （DJI C610 电调协议：每个电机对应一个字节对）
 *============================================================================*/

#define MT_DJI2006_ID_POKE         0x204   /* 拨弹电机 */
#define MT_DJI2006_ID_FRIC_1       0x201   /* 摩擦轮 1 */
#define MT_DJI2006_ID_FRIC_2       0x202   /* 摩擦轮 2 */
#define MT_DJI2006_ID_FRIC_3       0x203   /* 摩擦轮 3 */

/* DJI2006 数据类型 */
typedef struct {
    uint32_t ID;             /* CAN ID */
    float    pos_deg;        /* 单圈位置 (deg) */
    float    multi_turns_deg;/* 多圈位置 (deg) */
    float    speed_rpm;      /* 速度 (rpm) */
    float    speed_deg;      /* 速度 (deg/s) */
    float    Torq;           /* 扭矩 (N?m) */
} MT_DJI2006_Data_t;

/*============================================================================
 *  函数声明
 *============================================================================*/

/* ---------- DM4310 (CAN2) ---------- */

/**
 * @brief   DM4310 初始化（使能/失能/设零/清错）
 * @param   ID    电机 ID
 * @param   mode  MIT / PV / V
 * @param   state 初始化状态
 */
void MT_DM4310_Init(uint8_t ID, MT_DM4310_Mode_t mode, MT_DM4310_InitState_t state);

/**
 * @brief   DM4310 MIT 模式控制
 * @param   ID    电机 ID
 * @param   pos   目标位置 (rad)    范围 [-3.14, 3.14]
 * @param   vel   目标速度 (rad/s)  范围 [-120, 120]
 * @param   kp    位置刚度          范围 [0, 500]
 * @param   kd    速度阻尼          范围 [0, 5]
 * @param   torq  前馈力矩 (N?m)    范围 [-10, 10]
 */
void MT_DM4310_MIT_Ctrl(uint8_t ID, float pos, float vel,
                        float kp, float kd, float torq);

/**
 * @brief   DM4310 PV 模式控制
 * @param   ID      电机 ID
 * @param   pos     目标位置 (deg)
 * @param   velMax  最大速度 (deg/s) ? 达妙协议内部会除以 6
 */
void MT_DM4310_PV_Ctrl(uint8_t ID, float pos, float velMax);

/**
 * @brief   DM4310 V 模式控制
 * @param   ID    电机 ID
 * @param   vel   目标速度 (rad/s) ? 达妙协议内部会除以 6
 */
void MT_DM4310_V_Ctrl(uint8_t ID, float vel);

/**
 * @brief   解析 DM4310 接收数据
 * @param   data  电机反馈的 8 字节 CAN 数据
 * @param   out   输出结构体指针
 */
void MT_DM4310_ParseData(const uint8_t *data, MT_DM4310_Data_t *out);

/* ---------- DJI6020 (CAN2) ---------- */

/**
 * @brief   DJI6020 电流控制（发送电流值）
 * @param   current  电流值 (-16384 ~ 16384)
 * @param   send_id  CAN 发送 ID (默认 0x1FE)
 */
void MT_DJI6020_CurrentCtrl(int16_t current, uint32_t send_id);

/**
 * @brief   解析 DJI6020 接收数据
 * @param   data  电机反馈的 8 字节 CAN 数据
 * @param   out   输出结构体指针
 */
void MT_DJI6020_ParseData(const uint8_t *data, MT_DJI6020_Data_t *out);

/**
 * @brief   发送 DJI6020 停止指令（电流=0）
 */
void MT_DJI6020_Stop(void);

/* ---------- DJI2006 (CAN1) ---------- */

/**
 * @brief   DJI2006 电流控制（通过 C610 电调）
 * @param   can_id   目标电机 CAN ID
 * @param   current  电流值 (-16384 ~ 16384)
 * @note    大疆 C610 协议：ID 相同则共享电流值
 */
void MT_DJI2006_CurrentCtrl(uint32_t can_id, int16_t current);

/**
 * @brief   DJI2006 多电机同时控制（4 个电流值打包发送）
 * @param   can_id  目标电机 CAN ID
 * @param   c1~c4   4 个通道电流值
 * @note    适用于 0x200/0x1FF/0x1FE 等标准 C610 控制 ID
 */
void MT_DJI2006_GroupCtrl(uint32_t can_id,
                          int16_t c1, int16_t c2,
                          int16_t c3, int16_t c4);

/**
 * @brief   解析 DJI2006 接收数据
 * @param   data  电机反馈的 8 字节 CAN 数据
 * @param   out   输出结构体指针
 */
void MT_DJI2006_ParseData(const uint8_t *data, MT_DJI2006_Data_t *out);

/**
 * @brief   发送 DJI2006 停止指令
 * @param   can_id  目标电机 CAN ID
 */
void MT_DJI2006_Stop(uint32_t can_id);

/* ---------- 通用测试辅助函数 ---------- */

/**
 * @brief   便捷函数：初始化 DM4310 并使能
 * @param   ID  电机 ID
 */
static inline void MT_DM4310_Enable(uint8_t ID)
{
    MT_DM4310_Init(ID, MT_DM4310_MODE_MIT, MT_DM4310_INIT_ENABLE);
}

/**
 * @brief   便捷函数：失能 DM4310
 * @param   ID  电机 ID
 */
static inline void MT_DM4310_Disable(uint8_t ID)
{
    MT_DM4310_Init(ID, MT_DM4310_MODE_MIT, MT_DM4310_INIT_DISABLE);
}

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_TEST_H__ */
