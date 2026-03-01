#include "main.h"

uint8_t CH040_Rx_Buffer[CH040_RX_BUFF_SIZE];

 CH040_t CH040_Data;

void CH040_USART3_Init(void)
{
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	DMA_InitTypeDef  DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(USART_CH040_TX_GPIO_CLK | USART_CH040_RX_GPIO_CLK, ENABLE);//GPIOC6 7
	USART_CH040_CLK_INIT(USART_CH040_CLK, ENABLE);

	RCC_AHB1PeriphClockCmd(USART_CH040_DMAx_CLK, ENABLE);

	GPIO_PinAFConfig(USART_CH040_TX_GPIO_PORT, USART_CH040_TX_SOURCE, USART_CH040_TX_AF);
	GPIO_PinAFConfig(USART_CH040_RX_GPIO_PORT, USART_CH040_RX_SOURCE, USART_CH040_RX_AF);
  
  /* Configure USART Tx and Rx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  
	GPIO_InitStructure.GPIO_Pin = USART_CH040_TX_PIN;
	GPIO_Init(USART_CH040_TX_GPIO_PORT, &GPIO_InitStructure);
	  
	GPIO_InitStructure.GPIO_Pin = USART_CH040_RX_PIN;
	GPIO_Init(USART_CH040_RX_GPIO_PORT, &GPIO_InitStructure);
 
 
	NVIC_InitStructure.NVIC_IRQChannel = USART_CH040_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);

	/* Enable the USART OverSampling by 8 */
	USART_InitStructure.USART_BaudRate = 921600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	/* When using Parity the word length must be configured to 9 bits */
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART_CH040, &USART_InitStructure);

	/*Ê¹ÄÜ¿ÕÏÐÖ¡ÖÐ¶Ï*/
	USART_ITConfig(USART_CH040,USART_IT_IDLE,ENABLE);
	 USART_ClearFlag(USART_CH040,USART_FLAG_TC|USART_FLAG_IDLE);
	/* Configure DMA controller to manage USART TX and RX DMA request ----------*/ 

	DMA_InitStructure.DMA_BufferSize = CH040_RX_BUFF_SIZE ;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable ;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(USART_CH040->DR));
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	/* Configure RX DMA */
	DMA_InitStructure.DMA_Channel = USART_CH040_RX_DMA_CHANNEL ;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory ;
	DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)CH040_Rx_Buffer ; 
	DMA_Init(USART_CH040_RX_DMA_STREAM,&DMA_InitStructure);
	/* Enable DMA USART RX Stream */
	DMA_Cmd(USART_CH040_RX_DMA_STREAM,ENABLE);
	/* Enable USART DMA RX Requsts */
	USART_DMACmd(USART_CH040, USART_DMAReq_Rx, ENABLE);
	/* Enable USART */
	USART_Cmd(USART_CH040, ENABLE);
}

void CH040_Data_Cal(void)
{
	volatile static float Last_yaw_temp=0, Yaw_temp=0;
	volatile static int Yaw_count=0;
	Last_yaw_temp = Yaw_temp;
	Yaw_temp=CH040_Data.yaw_agl_get;

	if(Yaw_temp-Last_yaw_temp>=324)  
	{
		Yaw_count--;
	}
	else if (Yaw_temp-Last_yaw_temp<=-324)
	{
		Yaw_count++;
	} 
	CH040_Data.yaw_total_agl_calib=CH040_Data.yaw_agl_get+Yaw_count*360.0f-CH040_Data.Init_yaw_agl;
	CH040_Data.pitch_agl_calib=CH040_Data.pitch_agl_get-CH040_Data.Init_pitch_agl;
	if(CH040_Data.Init_Flag==0)
	{
		CH040_Data.Init_yaw_agl=CH040_Data.yaw_agl_get;
		CH040_Data.Init_pitch_agl=CH040_Data.pitch_agl_calib;
	}
}

void USART_CH040_IRQHandler(void)
{
	if(USART_GetITStatus(USART_CH040,USART_IT_IDLE)!=RESET)
	{
		USART_ReceiveData(USART_CH040);	
		USART_ClearITPendingBit(USART_CH040,USART_IT_IDLE);
		DMA_Cmd(USART_CH040_RX_DMA_STREAM,DISABLE);
		USART_DMACmd(USART_CH040,USART_DMAReq_Rx,DISABLE);
		

		memcpy(&CH040_Data,&CH040_Rx_Buffer[6],76);
		
		Equipment_Cnt_Clear(&Peripheral_State.Gimbal_Yaw_Gyro);

		CH040_Data_Cal();
		
		
		
		USART_DMACmd(USART_CH040,USART_DMAReq_Rx,ENABLE);
		DMA_Cmd(USART_CH040_RX_DMA_STREAM,ENABLE);
	}
}
