#include "main.h"

void Tim2_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	TIM_TimeBaseInitTypeDef My_Timer;
    My_Timer.TIM_Prescaler = 8400-1;
    My_Timer.TIM_CounterMode = TIM_CounterMode_Up;
    My_Timer.TIM_Period = 10-1;
    My_Timer.TIM_ClockDivision = TIM_CKD_DIV1;
    My_Timer.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &My_Timer);
	
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    
    NVIC_InitTypeDef My_NVIC;
    My_NVIC.NVIC_IRQChannel = TIM2_IRQn;
    My_NVIC.NVIC_IRQChannelCmd = ENABLE;
    My_NVIC.NVIC_IRQChannelPreemptionPriority = 1;
    My_NVIC.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&My_NVIC);
    
    TIM_Cmd(TIM2, ENABLE);
}
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
		Control_Task();
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}
