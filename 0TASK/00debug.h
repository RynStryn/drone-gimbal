//
// Created by RynStryn on 2026/3/29.
//

#ifndef INC_26DRONEMSTD_00DEBUG_H
#define INC_26DRONEMSTD_00DEBUG_H
#include <stdint.h>
typedef enum {
    HIT_CNT_GIMBAL_HOMING = 0,  // Gimbal_Homing() iterations + Main_Task() cycle tally
    HIT_CNT_INIT_POLL,          // DM4310 init polling in GIMBAL_INIT + OFFLINE mode entries
    HIT_CNT_RELAX_MODE,         // GIMBAL_RELAX / RC_RELAX mode entries
    HIT_CNT_OFF_ERR_PATH,       // GIMBAL_OFF path + REMOTE mode entries
    HIT_CNT_MAX
} hit_cnt_id_t;
const uint8_t debug_cnt_num_max = 20;
volatile uint16_t debug_HitCnt[debug_cnt_num_max] = {0};
#endif  // INC_26DRONEMSTD_00DEBUG_H
