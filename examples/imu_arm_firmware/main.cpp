#include "device.h"


#include <gpio.h>
#include <terminal.h>
#include <timer.h>
#include <i2c.h>
#include <mpu6050.h>
#include <hmc5883.h>

#include <imu.h>
#include <fmath.h>

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


  TI2C<TGPIOA, 0, 1, 20> i2c;
  i2c.init();     


  MPU6050 acc_gyro;

  int init_res = acc_gyro.init(&i2c);
  terminal << "mpu6050 init done with " << init_res << "\n";

  timer.delay_ms(1000);

  IMU imu;
  imu.init(0.1);
 
  uint32_t time_now  = 0;
  uint32_t time_prev = 0;
  int cnt = 0;
  
  float mps = 0;

  while (1)
  { 
    time_prev  = time_now;
    time_now   = timer.get_time();

    float   dt = 0.001*(time_now - time_prev);

    mps = 0.9*mps + 0.1*1.0/dt;

    acc_gyro.read();  

    //8g range to m/s^2
    float ax = G_const*acc_gyro.ax*8.0/32768.0;
    float ay = G_const*acc_gyro.ay*8.0/32768.0;
    float az = G_const*acc_gyro.az*8.0/32768.0;

    //1000dps range to rad/s
    float gx = (acc_gyro.gx*1000.0/32768.0)*DEG_TO_RAD;
    float gy = (acc_gyro.gy*1000.0/32768.0)*DEG_TO_RAD;
    float gz = (acc_gyro.gz*1000.0/32768.0)*DEG_TO_RAD;

   
    auto result = imu.step(ax, ay, az, gx, gy, gz, dt);

    float roll    = result.x*RAD_TO_DEG;
    float pitch   = result.y*RAD_TO_DEG;
    float yaw     = result.z*RAD_TO_DEG;
 
    if ((cnt%20) == 0)
    {   
      led_pin = 1;   
      terminal << "#JSON\n";
      terminal << "{\n";
      terminal << "\"imu_sensor\" : " << "[" << roll << ", " << pitch << ", " << yaw << ", " << mps << "]\n";
      terminal << "}";
      terminal << "#END\n\n\n";
      led_pin = 0;
    }


    cnt++;
  }

  return 0;
} 