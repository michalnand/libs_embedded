#include "device.h"


#include <gpio.h>
#include <terminal.h>
#include <timer.h>
#include <i2c.h>
#include <vl53l1/vl53l1.h>

#include <common_utils.h>
#include <kalman_filter.h>

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
  terminal << "\n\n\n\n";
  terminal << "terminal init done\n";

  timer.init(); 
  timer.delay_ms(500);


  TI2C<TGPIOA, 3, 4, 5> i2c;
  i2c.init();     


  for (unsigned int adr = 1; adr < 127; adr++)
  {
    uint8_t resp = i2c.check(adr<<1);
    if (resp != 0)  
    {
      terminal << "i2c device on adr = " << adr << "\n";
    }
  } 
  
  /*
  uint8_t adr = 0x29<<1;
 
  i2c.write_reg_extended(adr, 0x7fff, 0x00);
  uint8_t device_id   = i2c.read_reg_extended(adr, 0);
  uint8_t revision_id = i2c.read_reg_extended(adr, 1);
  i2c.write_reg_extended(adr, 0x7fff, 0x02);

 
  terminal << (uint32_t)device_id << " " << (uint32_t)revision_id << "\n"; //240, 2
  */

  VL53L1 tof_sensor;

  int init_res = tof_sensor.init(&i2c);
  terminal << "tof sensor init done with " << init_res << "\n";


  KalmanFilter filter;

  uint32_t steps = 0;
  while(1)
  {
    led_pin = 1;
    auto time_start = timer.get_time();
    VL53L1_RangingData result = tof_sensor.read();
    auto filtered_distance = filter.step(result.range_mm, 0, 2.5, 250.0, 10);

    auto time_stop = timer.get_time();

    uint32_t dt = time_stop - time_start;
    led_pin = 0;

    
    if (steps%10 == 0)
    {
      terminal << "#START\n";

      terminal << "{\n";
      terminal << "\"distance_r\" : " << result.range_mm << ",\n";
      terminal << "\"distance_f\" : " << filtered_distance << "\n";
      terminal << "}\n";
      terminal << "#END\n";
    }
    steps++;
    

    timer.delay_ms(10);
  }
  
  /*
  float k    = 0.95;
  float mean = 0.0;
  float var  = 0.0;

  while (1)
  {  
      led_pin = 1;
      VL53L1_RangingData result = tof_sensor.read();

      mean = k*mean + (1.0 - k)*result.range_mm;
      var  = k*var + (1.0 - k)*(result.range_mm - mean)*(result.range_mm - mean);

      terminal << result.range_mm << " " << mean << " " << var << "\n";
      led_pin = 0;

      timer.delay_ms(40);
  }
  */

  return 0;
} 