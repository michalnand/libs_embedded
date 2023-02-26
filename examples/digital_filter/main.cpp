#include "device.h"


#include <gpio.h>
#include <terminal.h>
#include <timer.h>

#include "biquad.h"

Terminal  terminal;
Timer     timer;




void SetPLL()
{
  RCC_PLLConfig(RCC_PLLSource_HSI, RCC_PLLMul_6);
  RCC_PLLCmd(ENABLE);

  // Wait for PLLRDY after enabling PLL.
  while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) != SET)
  { 
    __asm("nop");
  }

  RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);  // Select the PLL as clock source.
  SystemCoreClockUpdate();
}

int main(void)
{
  //cpu clock init
  SystemInit();

  // setup PLL, 6*HSI = 48MHz
  SetPLL();  

  Gpio<TGPIOB, 3, GPIO_MODE_OUT> led_pin;

  terminal.init(115200, USART2);
  terminal << "terminal init done\n";

  timer.init(); 

  Biquad<int32_t> filter(100, 200, 300, 400, 500);

  while (1) 
  {  
    led_pin = 1;
    int32_t time_prev  = timer.get_time();

    int32_t result = 0.0;
    for (unsigned int i = 0; i < 100000; i++)
    {
      int32_t x = i%1024;
      result+= filter.step(x); 
    } 

    int32_t time_now   = timer.get_time();
    led_pin = 0;

    float   dt          = (time_now - time_prev)*1000.0/100000.0;
  
    terminal << dt << "us" << " " << result << "\n";

    timer.delay_ms(100);
  }

  return 0;
} 