#include "ledTest.h"
#include "FreeRTOS.h"
#include "uartTest.h"
#include "task.h"
#include "stm32f4xx_it.h"

LedTest ledTest;

LedTest::LedTest()
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  xTaskCreate(ProcessLedTask, "ProcessLedTask", configMINIMAL_STACK_SIZE, (void *)NULL, 2, NULL);
};
void LedTest::ProcessLedTask(void *pvParameters)
{
  EventBits_t ev_bits;
  uint8_t led_state_machine = 0;
  uint8_t led_ind_state_machine = 0;
  while (1)
  {
    ev_bits = xEventGroupGetBits(xEventGroup);
    if ((ev_bits & ADC_WORK_ON) == ADC_WORK_ON)
    {
      if (led_state_machine != LED_ADC_ON)
      {
        led_state_machine = LED_ADC_ON;
        led_ind_state_machine = 0;
      }
    }
    else if (led_state_machine != LED_ADC_OFF)
    {
      led_state_machine = LED_ADC_OFF;
      led_ind_state_machine = 0;
    }

    switch (led_state_machine)
    {
    case LED_ADC_ON:
      GPIOD->BSRRL = GPIO_Pin_12;
      break;
    case LED_ADC_OFF:
      switch (led_ind_state_machine)
      {
      case 0:
        GPIOD->BSRRL = GPIO_Pin_12;
        vTaskDelay(100);
        led_ind_state_machine = 1;
        break;
      case 1:
        GPIOD->BSRRH = GPIO_Pin_12;
        vTaskDelay(100);
        led_ind_state_machine = 0;
        break;
      default:
        break;
      }
      break;
    default:
      break;
    }
  }
}
