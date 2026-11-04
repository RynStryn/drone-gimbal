//
// Created by RynStryn on 2026/5/8.
//

#include "Judge_System.h"


typedef enum {
    not_activated = 0,
    activated = 1,
    being_activated = 2
} e_activate_state;


typedef struct {
    const uint16_t *activation_timings;
    uint8_t timing_count;
    uint8_t ready;
    e_activate_state state;

} power_buff_t;





extern power_buff_t lite_buff;
extern power_buff_t big_buff;
extern receive_judge_t judge_rece_mesg;
extern uint8_t g_popup;
extern uint8_t popup_cnt;


void buff_init(void);
void buff_state_estimate_task(void);