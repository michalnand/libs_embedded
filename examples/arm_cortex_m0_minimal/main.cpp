#include "device.h"
#include <gpio.h>

#include <Ds1302.h>

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


void delay_loops(uint32_t loops)
{
  while (loops--)
  {
    __asm("nop");
  }
}


#define DS_CLK 0
#define DS_DAT 4
#define DS_RST 16

class RtClock
{
  public:
    RtClock(): rtc(DS_CLK, DS_DAT, DS_RST)
    {

    }
  private:

    Ds1302 rtc;
    
};

int main(void)
{
  RtClock clk_chip;

  //cpu clock init
  SystemInit();

  // setup PLL, 6*HSI = 48MHz
  SetPLL();  

  Gpio<TGPIOB, 3, GPIO_MODE_OUT> led_pin;

  while (1)
  {
    led_pin = 1;
    delay_loops(1000000);
    
    led_pin = 0;
    delay_loops(1000000);
  }

  return 0;
} 