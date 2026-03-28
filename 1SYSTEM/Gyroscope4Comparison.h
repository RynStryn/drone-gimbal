#ifndef INC_26DRONEMSTD_CH040_2_H
#define INC_26DRONEMSTD_CH040_2_H
/* UART4 相关定义 */
#define USART_CH040_2                UART4
#define USART_CH040_2_CLK            RCC_APB1Periph_UART4
#define USART_CH040_2_CLK_INIT       RCC_APB1PeriphClockCmd
#define USART_CH040_2_IRQn           UART4_IRQn
#define USART_CH040_2_IRQHandler     UART4_IRQHandler

/* 建议使用 PC10/PC11 (如果引脚没被占用) */
#define USART_CH040_2_TX_GPIO_CLK    RCC_AHB1Periph_GPIOC
#define USART_CH040_2_TX_GPIO_PORT   GPIOC
#define USART_CH040_2_TX_PIN         GPIO_Pin_10
#define USART_CH040_2_TX_SOURCE      GPIO_PinSource10
#define USART_CH040_2_TX_AF          GPIO_AF_UART4

#define USART_CH040_2_RX_GPIO_CLK    RCC_AHB1Periph_GPIOC
#define USART_CH040_2_RX_GPIO_PORT   GPIOC
#define USART_CH040_2_RX_PIN         GPIO_Pin_11
#define USART_CH040_2_RX_SOURCE      GPIO_PinSource11
#define USART_CH040_2_RX_AF          GPIO_AF_UART4

/* DMA 定义: UART4_RX 对应 DMA1, Stream 2, Channel 4 */
#define USART_CH040_2_DMAx_CLK       RCC_AHB1Periph_DMA1
#define USART_CH040_2_RX_DMA_CHANNEL DMA_Channel_4
#define USART_CH040_2_RX_DMA_STREAM  DMA1_Stream2

void CH040_UART4_Init();

#endif  // INC_26DRONEMSTD_CH040_2_H
