//
// Created by RynStryn on 2026/5/7.
//
#include "buff_state_estimated.h"

static const uint16_t total_competition_duration= 420;
static const uint8_t buff_recovery_time= 20;
static const uint8_t during_the_competition= 4;
static const uint16_t lite_buff_activation_timing[] = {0, 90};
static const uint16_t big_buff_activation_timing[] = {180, 255, 330};
static const uint16_t popup_time=5000;
 power_buff_t  lite_buff;
 power_buff_t  big_buff;
uint8_t g_popup=0;
uint8_t popup_cnt=0;
extern receive_judge_t judge_rece_mesg;

void buff_init()
{
    lite_buff.activation_timings = lite_buff_activation_timing;
    lite_buff.timing_count = sizeof(lite_buff_activation_timing) / sizeof(lite_buff_activation_timing[0]);
    lite_buff.state = not_activated;

    big_buff.activation_timings = big_buff_activation_timing;
    big_buff.timing_count = sizeof(big_buff_activation_timing) / sizeof(big_buff_activation_timing[0]);
    big_buff.state = not_activated;
}


int16_t game_time(ext_game_status_t game_state)
{
    int16_t time;
    if (game_state.game_progress == during_the_competition) {
        time = total_competition_duration - game_state.stage_remain_time;
    } else {
        time = -1;
    }
    return time;
}



static void buff_state_update(power_buff_t *buff, uint8_t state)
{
    switch (state) {
        case 0: buff->state = not_activated;   break;
        case 1: buff->state = activated;       break;
        case 2: buff->state = being_activated; break;
        default: break;
    }
}

void event_data_process(ext_event_data_t event_data)
{

    uint8_t power_byte = event_data.event_type;
    uint8_t small_state = (power_byte >> 3) & 0x03;
    uint8_t big_state   = (power_byte >> 5) & 0x03;
    buff_state_update(& lite_buff, small_state);
    buff_state_update(& big_buff, big_state);
}
uint8_t buff_ready_check(power_buff_t* buff,uint16_t t)
{

    for (uint8_t i = 0; i <  buff->timing_count; i++)
    {
        if (t == buff->activation_timings[i])
        {
            buff->ready=1;
        }
        else buff->ready=0;
    }
    return buff->ready;
}
void buff_state_estimate_task()
{
    int16_t time = game_time(judge_rece_mesg.game_state);
    uint8_t buff_ready;
    event_data_process(judge_rece_mesg.event_data);
    if (time < 0) {
        g_popup=0;
        popup_cnt=0;
        return;
    }
    else if (time<=180-10)
    {
        buff_ready=buff_ready_check(&lite_buff,time);
    }
    else
    {
        buff_ready = buff_ready_check(&big_buff, time);
    }
    if (buff_ready)
    {
        g_popup   = 1;
        popup_cnt = popup_time;
    }
    if (popup_cnt >=1)
    {
        //这部分接到UI
        popup_cnt--;
        if (popup_cnt <= 0)
        {
            g_popup = 0;
            popup_cnt=0;
        }
    }
}