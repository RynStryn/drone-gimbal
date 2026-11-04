#ifndef __CH040_H__
#define __CH040_H__
#include "stm32f4XX.h"



  /* Definition for USART_CH040 resources ******************************************/
  #define USART_CH040                           USART3
  #define USART_CH040_CLK                       RCC_APB1Periph_USART3
  #define USART_CH040_CLK_INIT                  RCC_APB1PeriphClockCmd
  #define USART_CH040_IRQn                      USART3_IRQn
  #define USART_CH040_IRQHandler                USART3_IRQHandler

  #define USART_CH040_TX_PIN                    GPIO_Pin_10
  #define USART_CH040_TX_GPIO_PORT              GPIOB
  #define USART_CH040_TX_GPIO_CLK               RCC_AHB1Periph_GPIOB
  #define USART_CH040_TX_SOURCE                 GPIO_PinSource10
  #define USART_CH040_TX_AF                     GPIO_AF_USART3

  #define USART_CH040_RX_PIN                    GPIO_Pin_11
  #define USART_CH040_RX_GPIO_PORT              GPIOB                   
  #define USART_CH040_RX_GPIO_CLK               RCC_AHB1Periph_GPIOB
  #define USART_CH040_RX_SOURCE                 GPIO_PinSource11
  #define USART_CH040_RX_AF                     GPIO_AF_USART3

  /* Definition for DMAx resources ********************************************/
  #define USART_CH040_DR_ADDRESS                ((uint32_t)USART3 + 0x04) 

  #define USART_CH040_DMA                       DMA1
  #define USART_CH040_DMAx_CLK                  RCC_AHB1Periph_DMA1
     
  #define USART_CH040_TX_DMA_CHANNEL            DMA_Channel_4
  #define USART_CH040_TX_DMA_STREAM             DMA1_Stream3
  #define USART_CH040_TX_DMA_FLAG_FEIF          DMA_FLAG_FEIF3
  #define USART_CH040_TX_DMA_FLAG_DMEIF         DMA_FLAG_DMEIF3
  #define USART_CH040_TX_DMA_FLAG_TEIF          DMA_FLAG_TEIF3
  #define USART_CH040_TX_DMA_FLAG_HTIF          DMA_FLAG_HTIF3
  #define USART_CH040_TX_DMA_FLAG_TCIF          DMA_FLAG_TCIF3
              
  #define USART_CH040_RX_DMA_CHANNEL            DMA_Channel_4
  #define USART_CH040_RX_DMA_STREAM             DMA1_Stream1
  #define USART_CH040_RX_DMA_FLAG_FEIF          DMA_FLAG_FEIF1
  #define USART_CH040_RX_DMA_FLAG_DMEIF         DMA_FLAG_DMEIF1
  #define USART_CH040_RX_DMA_FLAG_TEIF          DMA_FLAG_TEIF1
  #define USART_CH040_RX_DMA_FLAG_HTIF          DMA_FLAG_HTIF1
  #define USART_CH040_RX_DMA_FLAG_TCIF          DMA_FLAG_TCIF1
  
  
  #define CH040_RX_BUFF_SIZE 100


__packed typedef struct 
{
	uint8_t 	tag;
	uint16_t 	main_status;
	int8_t		temperature;
	float		air_pressure;
	uint32_t	system_time;
	float		acc_y;
	float		acc_x;
	float		acc_z;
	
	float		omega_y;
	float		omega_x;
	float		omega_z;
	
	float		mag_y;
	float		mag_x;
	float		mag_z;
	
	float		pitch_agl_get;
	float		roll_agl_get;
	float		yaw_agl_get;
	float		quat[4];
	
	float		Init_yaw_agl;
	float		Init_pitch_agl;
	
	float		yaw_total_agl_calib;
	float		pitch_agl_calib;
	uint8_t		Init_Flag;
	
}CH040_t;

extern CH040_t CH040_Data;
extern uint8_t Init_Flag;

void CH040_USART3_Init(void);

#endif
