#include "main.h"
#include "Gyroscope4Comparison.h"
void Drone_akaFighter_init(void)
{
#ifndef CANoff
    CAN1_Init();
    CAN2_Init();
#endif
    USART1_Init(100000);
    CH040_USART3_Init();
	CH040_UART4_Init();//
    USART5_DMA_R_T_JUDGE_Init();
	usart6_init();
    Gimbal_Ctrler_Init();
    Shooter_PID_Init();

    Tim2_Init();
}


void SoftReset(void)
{
    __set_FAULTMASK(1);  // 关闭所有中断
    NVIC_SystemReset();  // 复位
}
