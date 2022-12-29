#ifndef __LED_TEST_H
#define __LED_TEST_H

enum
{
  LED_ADC_ON,
  LED_ADC_OFF
};

class LedTest
{
  static void ProcessLedTask(void *pvParameters);

public:
  LedTest();
};

#endif