#ifndef __ADC_TEST_H
#define __ADC_TEST_H

#include "stm32f4xx_conf.h"

#define ADCx ADC3
#define ADC_CHANNEL ADC_Channel_2
#define ADCx_CLK RCC_APB2Periph_ADC3
#define ADCx_CHANNEL_GPIO_CLK RCC_AHB1Periph_GPIOA
#define GPIO_PIN GPIO_Pin_2
#define GPIO_PORT GPIOA
// #define DMA_CHANNELx             DMA_Channel_2
// #define DMA_STREAMx              DMA2_Stream0
#define ADCx_DR_ADDRESS ((uint32_t)0x4001224C)

#define AVRG_SIZE (10)

enum AdcState
{
  ADC_ON,
  ADC_OFF
};

class AdcTest
{
  void AdcInit(void);
  void Tim3Init(void);
  static void ProcessADCTask(void *pvParameters);
  static uint8_t AdcState;

public:
  AdcTest();
  void IrqADC(void);
};

struct ResultVal
{
  float instant_value;
  float average_value;
  float rms_value;
};

extern volatile ResultVal res_val;

#endif
