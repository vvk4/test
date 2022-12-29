#include "FreeRTOS.h"
#include "uartTest.h"
#include "adcTest.h"
#include "task.h"
#include "semphr.h"
#include "stm32f4xx_it.h"
#include <string.h>
#include <iostream>

char tx_buffer[BUFFERSIZE];
char rx_buffer[BUFFERSIZE];
uint8_t uart_ready;
SemaphoreHandle_t xSemaphore = NULL;
EventGroupHandle_t xEventGroup;
UartTest uartTest;

UartTest::UartTest()
{
  xSemaphore = xSemaphoreCreateBinary();
  if (xSemaphore == NULL)
  {
    // TODO
  }
  xEventGroup = xEventGroupCreate();
  if (xEventGroup == NULL)
  {
    // TODO
  }
  InitUART();
  xTaskCreate(ProcessUARTRxTask, "ProcessUARTRxTask", 500 /*configMINIMAL_STACK_SIZE*/, (void *)NULL, 4, NULL);
}

void UartTest::ProcessUARTRxTask(void *pvParameters)
{
  while (1)
  {
    if (xSemaphoreTake(xSemaphore, (TickType_t)portMAX_DELAY) == pdTRUE)
    {
      ProcessCommand();
    }
  }
}
uint8_t UartTest::UartTx(uint8_t *buf, uint16_t size)
{
  uint16_t i, i_max;

  taskENTER_CRITICAL();
  if (!uart_ready)
  {
    taskEXIT_CRITICAL();
    return PREV_OPERATION_PENDING;
  }
  uart_ready = 0;
  taskEXIT_CRITICAL();

  if (size < BUFFERSIZE)
    i_max = size;
  else
    i_max = BUFFERSIZE;

  for (i = 0; i < i_max; i++)
    tx_buffer[i] = buf[i];

  DMA_SetCurrDataCounter(USARTx_TX_DMA_STREAM, size);
  DMA_Cmd(USARTx_TX_DMA_STREAM, ENABLE);
  USART_DMACmd(USARTx, USART_DMAReq_Tx, ENABLE);
  return OPERATION_OK;
}

void UartTest::InitUART(void)
{
  DMA_InitTypeDef DMA_InitStructure;

  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(USARTx_TX_GPIO_CLK | USARTx_RX_GPIO_CLK, ENABLE);

  USARTx_CLK_INIT(USARTx_CLK, ENABLE);

  RCC_AHB1PeriphClockCmd(USARTx_DMAx_CLK, ENABLE);

  GPIO_PinAFConfig(USARTx_TX_GPIO_PORT, USARTx_TX_SOURCE, USARTx_TX_AF);
  GPIO_PinAFConfig(USARTx_RX_GPIO_PORT, USARTx_RX_SOURCE, USARTx_RX_AF);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

  GPIO_InitStructure.GPIO_Pin = USARTx_TX_PIN;
  GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = USARTx_RX_PIN;
  GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStructure);

  USART_OverSampling8Cmd(USARTx, ENABLE);

  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USARTx, &USART_InitStructure);

  DMA_InitStructure.DMA_BufferSize = BUFFERSIZE;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(USARTx->DR));
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  // TX DMA
  DMA_InitStructure.DMA_Channel = USARTx_TX_DMA_CHANNEL;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)tx_buffer;
  DMA_Init(USARTx_TX_DMA_STREAM, &DMA_InitStructure);
  // RX DMA
  DMA_InitStructure.DMA_Channel = USARTx_RX_DMA_CHANNEL;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rx_buffer;
  DMA_Init(USARTx_RX_DMA_STREAM, &DMA_InitStructure);

  // USARTx Interrupt
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = USARTx_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  USART_Cmd(USARTx, ENABLE);
  DMA_Cmd(USARTx_RX_DMA_STREAM, ENABLE);
  USART_DMACmd(USARTx, USART_DMAReq_Rx, ENABLE);
  while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET)
    ;

  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_SetPriority(DMA1_Stream3_IRQn, 8);
  DMA_ITConfig(USARTx_TX_DMA_STREAM, DMA_IT_TC, ENABLE);

  NVIC_SetPriority(USART3_IRQn, 8);
  USART_ITConfig(USARTx, USART_IT_IDLE, ENABLE);
  USART_ITConfig(USARTx, USART_IT_TC, ENABLE);

  taskENTER_CRITICAL();
  uart_ready = 1;
  taskEXIT_CRITICAL();
}

void UartTest::ProcessCommand(void)
{
  if (!strcmp(rx_buffer, "start"))
  {
    xEventGroupSetBits(xEventGroup, ADC_WORK_ON);
    UartTx((uint8_t *)"Ok\r\n", sizeof("Ok\r\n") - 1);
  }
  else if (!strcmp(rx_buffer, "stop"))
  {
    xEventGroupClearBits(xEventGroup, ADC_WORK_ON);
    UartTx((uint8_t *)"->>-\r\n", sizeof("->>-\r\n") - 1);
  }
  else if (!strcmp(rx_buffer, "result"))
  {

    char res[50] = "\r\nInstatnt: ";
    char tmp[10];
    sprintf(tmp, "%f", res_val.instant_value);
    strcat(res, tmp);
    strcat(res, "V\r\nAverage: ");
    sprintf(tmp, "%f", res_val.average_value);
    strcat(res, tmp);
    strcat(res, "V\r\n");
    UartTx((uint8_t *)res, strlen(res) - 1);
  }
  else
  {
    char ucmd[50]="\r\nunknown command: \"";
    strcat (ucmd, rx_buffer);
    strcat (ucmd, "\"\r\n");
    UartTx((uint8_t *)ucmd, strlen(ucmd) - 1);
  }
}

void DMA1_Stream3_IRQHandler(void)
{
  if (DMA_GetITStatus(USARTx_TX_DMA_STREAM, DMA_IT_TCIF3) == SET)
    DMA_ClearITPendingBit(USARTx_TX_DMA_STREAM, DMA_IT_TCIF3);
}

void USART3_IRQHandler(void)
{
  if (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == SET)
  {
    USART_ClearFlag(USARTx, USART_FLAG_TC); // Ïåðåäà÷à çàêîí÷åíà
    uart_ready = 1;
  }

  if (USART_GetFlagStatus(USARTx, USART_FLAG_IDLE) == SET)
  {

    uint16_t val = USART_ReceiveData(USARTx);
    xSemaphoreGiveFromISR(xSemaphore, pdFALSE);

    DMA_Cmd(USARTx_RX_DMA_STREAM, DISABLE); // Ïåðåçàïóñê DMA ïðèåìíèêà
    while (DMA_GetCmdStatus(USARTx_RX_DMA_STREAM))
      ;
    uint16_t len = BUFFERSIZE - DMA_GetCurrDataCounter(USARTx_RX_DMA_STREAM);
    rx_buffer[len] = 0;
    DMA_ClearFlag(USARTx_RX_DMA_STREAM, DMA_FLAG_TCIF1);
    DMA_SetCurrDataCounter(USARTx_RX_DMA_STREAM, BUFFERSIZE);
    DMA_Cmd(USARTx_RX_DMA_STREAM, ENABLE);
  }
}
