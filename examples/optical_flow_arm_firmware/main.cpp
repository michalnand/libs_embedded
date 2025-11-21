#include "device.h"


#include <gpio.h>
#include <terminal.h>
#include <timer.h>
#include <spi.h>
#include <adns3080.h>

uint8_t frame_buffer[ADNS3080_PIXELS_X*ADNS3080_PIXELS_Y];

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

  SPI spi;
  ADNS3080 sensor;

  int init_result = sensor.init(&spi);
  terminal << "optical flow sensor init result " << init_result << "\n";

  timer.delay_ms(1000);

  uint32_t time_prev = 0;
  uint32_t time_now  = 0;

  

  while (1)
  {  
    time_prev  = time_now;
    time_now   = timer.get_time();

    float   dt = time_now - time_prev;

    sensor.capture_frame(frame_buffer);

    /*
    int8_t dx, dy; 
    sensor.displacement(&dx, &dy);
    terminal << "moving = " << (int)dx << " " << (int)dy << "\n";
    */
  }

  return 0;
} 