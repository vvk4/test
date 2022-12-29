#include "FreeRTOS.h"
#include "adcTest.h"
#include "task.h"
#include "stm32f4xx_it.h"
#include "queue.h"
#include "uartTest.h"

uint8_t AdcTest::AdcState = ADC_OFF;
uint16_t avrgArray[AVRG_SIZE];
ResultVal res_val;
QueueHandle_t xQueueADC = NULL;
AdcTest adcTest;

AdcTest::AdcTest()
{
  xQueueADC = xQueueCreate(10, sizeof(uint16_t));
  AdcInit();
  Tim3Init();
  xTaskCreate(ProcessADCTask, "ProcessADCTask", 500 /*configMINIMAL_STACK_SIZE*/, (void *)NULL, 3, NULL);
};

void AdcTest::ProcessADCTask(void *pvParameters)
{
  uint16_t adr_result, i;
  uint32_t summ;
  uint8_t start = 1;
  EventBits_t ev_bits;

  while (1)
  {
    if (xQueueADC != NULL)
    {
      switch (AdcState)
      {
      case ADC_ON:
        if (xQueueReceive(xQueueADC, (void *)&adr_result, (TickType_t)portMAX_DELAY) == pdPASS)
        {

          ev_bits = xEventGroupGetBits(xEventGroup);
          if ((ev_bits & ADC_WORK_ON) != ADC_WORK_ON)
          {
            ADC_Cmd(ADC3, DISABLE);
            AdcState = ADC_OFF;
          }

          summ = 0;
          if (start)
          {
            start = 0;
            for (i = 0; i < AVRG_SIZE; i++)
            {
              avrgArray[i] = adr_result;
              summ += avrgArray[i];
            }
          }
          else
          {
            for (i = 0; i < (AVRG_SIZE - 1); i++)
            {
              avrgArray[i] = avrgArray[i + 1];
              summ += avrgArray[i];
            }
            avrgArray[i] = adr_result;
            summ += avrgArray[i];
          }
          float tmp1, tmp2;
          tmp1 = ((float)summ * 3.3) / (float)(AVRG_SIZE * 4096);
          tmp2 = (float)adr_result * 3.3 / 4096;

          taskENTER_CRITICAL();
          res_val.average_value = tmp1;
          res_val.instant_value = tmp2;
          taskEXIT_CRITICAL();
        }
        break;
      case ADC_OFF:
        ev_bits = xEventGroupGetBits(xEventGroup);
        if (ev_bits & ADC_WORK_ON == ADC_WORK_ON)
        {
          ADC_Cmd(ADC3, ENABLE);
          AdcState = ADC_ON;
        }
        vTaskDelay(1);
        break;
      }
    }
    else
      vTaskDelay(1);
  }
}

void AdcTest::AdcInit(void)
{
  ADC_InitTypeDef ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x3;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  NVIC_SetPriority(ADC_IRQn, 7); // Priority

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
  RCC_AHB1PeriphClockCmd(ADCx_CHANNEL_GPIO_CLK, ENABLE);
  RCC_APB2PeriphClockCmd(ADCx_CLK, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIO_PORT, &GPIO_InitStructure);

  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div8;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADCx, &ADC_InitStructure);

  ADC_EOCOnEachRegularChannelCmd(ADC3, ENABLE);

  ADC_RegularChannelConfig(ADC3, ADC_CHANNEL, 1, ADC_SampleTime_3Cycles);

  ADC_ITConfig(ADC3, ADC_IT_OVR, ENABLE);
  ADC_ITConfig(ADC3, ADC_IT_EOC, ENABLE);
}

void AdcTest::Tim3Init(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Period = 0xe78;
  TIM_TimeBaseStructure.TIM_Prescaler = 50;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV4;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);
  TIM3->DIER |= TIM_DIER_UIE;
  TIM_Cmd(TIM3, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}

void AdcTest::IrqADC(void)
{
  uint16_t uhADCxConvertedValue;
  GPIOD->ODR ^= GPIO_Pin_11;

  if (ADC_GetITStatus(ADC3, ADC_IT_EOC))
  {
    ADC_ClearITPendingBit(ADC3, ADC_IT_EOC);
    uhADCxConvertedValue = ADC_GetConversionValue(ADC3);
    if (xQueueADC != NULL)
    {
      if (xQueueSendFromISR(xQueueADC, (void const *)&uhADCxConvertedValue, NULL) == errQUEUE_FULL)
      {
        asm("Nop"); // TODO
      }
    }
  }
  if (ADC_GetITStatus(ADC3, ADC_IT_OVR))
    ADC_ClearITPendingBit(ADC3, ADC_IT_OVR);
}

void ADC_IRQHandler(void)
{
  adcTest.IrqADC();
}
