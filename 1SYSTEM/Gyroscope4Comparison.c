
#include "Gyroscope4Comparison.h"

#include <string.h>

#include "CH040.h"
#include "Fault_Diagnosis_Task.h"

uint8_t CH040_Rx_Buffer2[CH040_RX_BUFF_SIZE];
CH040_t CH040_Data2;
void CH040_UART4_Init(void)
{
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    DMA_InitTypeDef  DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 时钟使能 */
    RCC_AHB1PeriphClockCmd(USART_CH040_2_TX_GPIO_CLK | USART_CH040_2_RX_GPIO_CLK, ENABLE);
    USART_CH040_2_CLK_INIT(USART_CH040_2_CLK, ENABLE);
    RCC_AHB1PeriphClockCmd(USART_CH040_2_DMAx_CLK, ENABLE);

    /* GPIO 复用配置 */
    GPIO_PinAFConfig(USART_CH040_2_TX_GPIO_PORT, USART_CH040_2_TX_SOURCE, USART_CH040_2_TX_AF);
    GPIO_PinAFConfig(USART_CH040_2_RX_GPIO_PORT, USART_CH040_2_RX_SOURCE, USART_CH040_2_RX_AF);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

    GPIO_InitStructure.GPIO_Pin = USART_CH040_2_TX_PIN;
    GPIO_Init(USART_CH040_2_TX_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = USART_CH040_2_RX_PIN;
    GPIO_Init(USART_CH040_2_RX_GPIO_PORT, &GPIO_InitStructure);

    /* NVIC 配置 */
    NVIC_InitStructure.NVIC_IRQChannel = USART_CH040_2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; // 稍微改一下优先级以区分
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* UART 配置 */
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART_CH040_2, &USART_InitStructure);

    /* 中断与清除标志 */
    USART_ITConfig(USART_CH040_2, USART_IT_IDLE, ENABLE);
    USART_ClearFlag(USART_CH040_2, USART_FLAG_TC | USART_FLAG_IDLE);

    /* DMA RX 配置 */

    DMA_InitStructure.DMA_BufferSize = CH040_RX_BUFF_SIZE;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(USART_CH040_2->DR));
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;

    DMA_InitStructure.DMA_Channel = USART_CH040_2_RX_DMA_CHANNEL;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)CH040_Rx_Buffer2;
    DMA_Init(USART_CH040_2_RX_DMA_STREAM, &DMA_InitStructure);

    DMA_Cmd(USART_CH040_2_RX_DMA_STREAM, ENABLE);
    USART_DMACmd(USART_CH040_2, USART_DMAReq_Rx, ENABLE);
    USART_Cmd(USART_CH040_2, ENABLE);
}
//
void UART4_SendByte(uint8_t data)
{
    while (USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
    USART_SendData(UART4, data);
    while (USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET);
}

void CH040_Data_Cal2(void)
{

    volatile static float Last_yaw_temp2 = 0, Yaw_temp2 = 0;
    volatile static int Yaw_count2 = 0;

    Last_yaw_temp2 = Yaw_temp2;
    Yaw_temp2 = CH040_Data2.yaw_agl_get;

    if(Yaw_temp2 - Last_yaw_temp2 >= 324)
    {
        Yaw_count2--;
    }
    else if (Yaw_temp2 - Last_yaw_temp2 <= -324)
    {
        Yaw_count2++;
    }

    CH040_Data2.yaw_total_agl_calib = CH040_Data2.yaw_agl_get + Yaw_count2 * 360.0f - CH040_Data2.Init_yaw_agl;
    CH040_Data2.pitch_agl_calib = CH040_Data2.pitch_agl_get - CH040_Data2.Init_pitch_agl - 24;


}

void USART_CH040_2_IRQHandler(void)
{
    if(USART_GetITStatus(USART_CH040_2,USART_IT_IDLE)!=RESET)
    {
        USART_ReceiveData(USART_CH040_2);
        USART_ClearITPendingBit(USART_CH040_2,USART_IT_IDLE);
        DMA_Cmd(USART_CH040_2_RX_DMA_STREAM,DISABLE);
        USART_DMACmd(USART_CH040_2,USART_DMAReq_Rx,DISABLE);


        memcpy(&CH040_Data2,&CH040_Rx_Buffer2[6],76);

        Equipment_Cnt_Clear(&Peripheral_State.Gimbal_Yaw_Gyro);

        CH040_Data_Cal2();

        USART_DMACmd(USART_CH040_2,USART_DMAReq_Rx,ENABLE);
        DMA_Cmd(USART_CH040_2_RX_DMA_STREAM,ENABLE);
    }
}