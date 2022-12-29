#include "device.h"


#include <gpio.h>
#include <terminal.h>
#include <timer.h>
#include <encoder.h>

#include <pwm.h>

#include <kalman_filter.h>
#include <lqc.h>

Terminal  terminal;
Timer     timer;



// setup PLL, 6*HSI = 48MHz
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
  SetPLL();  

  Gpio<TGPIOB, 3, GPIO_MODE_OUT> led_pin;
  led_pin = 1;

  terminal.init(115200, USART2);
  terminal << "terminal init done\n";

  timer.init(); 

  Encoder encoder;
  encoder.init(); 

  PWM          pwm;
  pwm.init();

  KalmanFilter filter;
  LQC<>        lqc;

  lqc.set_cm(0, 0,      7.0);
  lqc.set_cm(0, 8 + 0, -7.0);
  lqc.set_cm(0, 8 + 1, -0.1);
 
  uint32_t  time_now  = 0;
  uint32_t  time_prev = 0;
  uint32_t  steps     = 0;

  float required[]    = {0.0, 0.5, 0.0, -0.5};

  while(1)
  {
    time_prev  = time_now;
    time_now   = timer.get_time();

    auto   dt = time_now - time_prev;

    //obtained required value
    float distance_req = required[(steps/100)%4];

    //obtain raw state from encoder
    encoder.update(dt);
 
    auto distance = encoder.get_distance();
    auto velocity = encoder.get_velocity();

    //filter values, and scale
    filter.step(distance, velocity, 0.1, 0.1, dt);
    auto distance_rel = filter.get_x()/WHEEL_CIRCUMREFERENCE;
    auto velocity_rel = filter.get_v()/WHEEL_CIRCUMREFERENCE;
    

    //set LQC inputs, required value and system state
    lqc.set_required(0, distance_req);
    lqc.set_state(0, distance_rel);
    lqc.set_state(1, velocity_rel);

    //controller forward
    lqc.step();

    //get controller output and send to motor
    auto speed = lqc.get_output(0);
    pwm.set_motor(speed);

    //print
    if (steps%100 == 0)
    {
      terminal << distance_req << " " << distance_rel << " " << speed << " " << dt << "\n";

      if (led_pin)
        led_pin = 0;
      else
        led_pin = 1;
    }

    steps++;


    timer.delay_ms(5);
  }


  return 0;
} 
