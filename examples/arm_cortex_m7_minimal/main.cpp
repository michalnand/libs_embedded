#include "device.h"
#include <clock.h>
#include <terminal.h>
#include <timer.h>
#include <gpio.h>
#include <adc_driver.h>


void delay_loops(uint32_t loops)
{
  while (loops--)
  {
    __asm("nop");
  }
}

int main(void)
{
  SetSysClock(SysClok216_24HSE);
  //SetSysClock(SysClok312_24HSE);

  Gpio<TGPIOI, 1, GPIO_MODE_OUT> led;
  led = 1; 
    
  Terminal terminal;
  terminal.init(115200, USART1);

  terminal << "\n\n\n";
  terminal << "\nstarting\n";
  terminal << "system clock = " << SystemCoreClock/1000000 << "MHz\n";

  
  Timer timer;
  timer.init();

  terminal << "timer init [done]\n";


  ADC_driver adc;
  adc.init();

  terminal << "adc init [done]\n";

  uint32_t time_prev = timer.get_time();
  uint32_t time_now = time_prev;

  while (1)
  {
    led = 1; 
    timer.delay_ms(100);

    led = 0; 
    timer.delay_ms(900);

    time_prev = time_now;
    time_now  = timer.get_time();

    for (unsigned int i = 0; i < ADC_CHANNELS_COUNT; i++)
    {
      terminal << adc.read()[i] << " ";
    }



    terminal << "\n";

    terminal << "time = " << timer.get_time() << "\n";

    terminal << "\n\n\n";
    
  }

    return 0;
} 