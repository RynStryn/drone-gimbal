#ifndef __DATA_SAMPLE_H__
#define __DATA_SAMPLE_H__
#include <main.h>




#define PI 3.1415926535f




void sample_task(int frequency);
void sample_control_task(void);
void sample_task_Init(void);



extern float sys_input;
extern int16_t disterb;


#endif
