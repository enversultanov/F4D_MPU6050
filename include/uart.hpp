/*
 * uart.h
 *
 *  Created on: 20 okt 2014
 *      Author: ensul1
 */

#ifndef UART_HPP_
#define UART_HPP_

#include "stm32f4xx_hal.h"
#include "stm32f4_discovery.h"
#include <stdio.h>
#include <math.h>

#define STR(s)	#s
#define XSTR(s)	STR(s)

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* User can use this section to tailor USARTx/UARTx instance used and associated
   resources */
/* Definition for USARTx clock resources */
#define USARTx                           USART2
#define USARTx_CLK_ENABLE()              __USART2_CLK_ENABLE();
#define DMAx_CLK_ENABLE()                __DMA1_CLK_ENABLE()
#define USARTx_RX_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE()

#define USARTx_FORCE_RESET()             __USART2_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __USART2_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                    GPIO_PIN_2
#define USARTx_TX_GPIO_PORT              GPIOA
#define USARTx_TX_AF                     GPIO_AF7_USART2
#define USARTx_RX_PIN                    GPIO_PIN_3
#define USARTx_RX_GPIO_PORT              GPIOA
#define USARTx_RX_AF                     GPIO_AF7_USART2

/* Definition for USARTx's DMA */
#define USARTx_TX_DMA_CHANNEL             DMA_CHANNEL_4
#define USARTx_TX_DMA_STREAM              DMA1_Stream6
#define USARTx_RX_DMA_CHANNEL             DMA_CHANNEL_4
#define USARTx_RX_DMA_STREAM              DMA1_Stream5

/* Definition for USARTx's NVIC */
#define USARTx_DMA_TX_IRQn                DMA1_Stream6_IRQn
#define USARTx_DMA_RX_IRQn                DMA1_Stream5_IRQn
#define USARTx_DMA_TX_IRQHandler          DMA1_Stream6_IRQHandler
#define USARTx_DMA_RX_IRQHandler          DMA1_Stream5_IRQHandler

//#define IS_UART_TX_COMPLET(STATE)	((STATE) == RESET)

/* Size of Transmission buffer */
//#define TXBUFFERSIZE                      (COUNTOF(aTxBuffer) - 1)
/* Size of Reception buffer */
//#define RXBUFFERSIZE                      TXBUFFERSIZE

/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))

void UART_Init(void);

void UART_Float_TX(float f);
void UART_TX(uint8_t *pData, uint16_t Size);

int Float2String(float float_val);

//extern __IO ITStatus UartReady;

#endif /* UART_H_ */
