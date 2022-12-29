#ifndef __UART_TEST_H
#define __UART_TEST_H

#include "stm32f4xx_conf.h"
#include "event_groups.h"

#define BUFFERSIZE (100)

#define USARTx USART3
#define USARTx_CLK RCC_APB1Periph_USART3
#define USARTx_CLK_INIT RCC_APB1PeriphClockCmd
#define USARTx_IRQn USART3_IRQn
#define USARTx_IRQHandler USART3_IRQHandler

#define USARTx_TX_PIN GPIO_Pin_10
#define USARTx_TX_GPIO_PORT GPIOC
#define USARTx_TX_GPIO_CLK RCC_AHB1Periph_GPIOC
#define USARTx_TX_SOURCE GPIO_PinSource10
#define USARTx_TX_AF GPIO_AF_USART3

#define USARTx_RX_PIN GPIO_Pin_11
#define USARTx_RX_GPIO_PORT GPIOC
#define USARTx_RX_GPIO_CLK RCC_AHB1Periph_GPIOC
#define USARTx_RX_SOURCE GPIO_PinSource11
#define USARTx_RX_AF GPIO_AF_USART3

#define USARTx_DR_ADDRESS ((uint32_t)USART3 + 0x04)

#define USARTx_DMA DMA1
#define USARTx_DMAx_CLK RCC_AHB1Periph_DMA1

#define USARTx_TX_DMA_CHANNEL DMA_Channel_4
#define USARTx_TX_DMA_STREAM DMA1_Stream3
#define USARTx_TX_DMA_FLAG_FEIF DMA_FLAG_FEIF3
#define USARTx_TX_DMA_FLAG_DMEIF DMA_FLAG_DMEIF3
#define USARTx_TX_DMA_FLAG_TEIF DMA_FLAG_TEIF3
#define USARTx_TX_DMA_FLAG_HTIF DMA_FLAG_HTIF3
#define USARTx_TX_DMA_FLAG_TCIF DMA_FLAG_TCIF3

#define USARTx_RX_DMA_CHANNEL DMA_Channel_4
#define USARTx_RX_DMA_STREAM DMA1_Stream1
#define USARTx_RX_DMA_FLAG_FEIF DMA_FLAG_FEIF1
#define USARTx_RX_DMA_FLAG_DMEIF DMA_FLAG_DMEIF1
#define USARTx_RX_DMA_FLAG_TEIF DMA_FLAG_TEIF1
#define USARTx_RX_DMA_FLAG_HTIF DMA_FLAG_HTIF1
#define USARTx_RX_DMA_FLAG_TCIF DMA_FLAG_TCIF1

#define USARTx_DMA_TX_IRQn DMA1_Stream3_IRQn
#define USARTx_DMA_RX_IRQn DMA1_Stream1_IRQn
#define USARTx_DMA_TX_IRQHandler DMA1_Stream3_IRQHandler
#define USARTx_DMA_RX_IRQHandler DMA1_Stream1_IRQHandler

enum
{
  OPERATION_OK,
  PREV_OPERATION_PENDING
};

// event bits:
#define ADC_WORK_ON (1)

class UartTest
{
  static uint8_t UartTx(uint8_t *buf, uint16_t size);
  static void ProcessUARTRxTask(void *pvParameters);

  void InitUART(void);

public:
  UartTest();
  static void ProcessCommand(void);
};

extern EventGroupHandle_t xEventGroup;

#endif