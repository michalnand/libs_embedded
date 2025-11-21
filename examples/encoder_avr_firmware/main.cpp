#include <timer.h>
#include <gpio.h>
#include <terminal.h>
#include <i2c.h>

#include <math.h>
#include <encoder.h>
#include <kalman_filter.h>


void init_pwm()
{
  DDRD|= (1<<5)|(1<<6);
    
    // 62.5kHz non-inverted PWM on OC0A with no prescalar
    TCCR0A = (1 << COM0A1) | (1<<COM0B1) | (1<<WGM01) | (1<<WGM00); 
    TCCR0B = (1 << CS02); 

    OCR0A = 0;
    OCR0B = 0;
}

void set_pwm(int pwm)
{
    if (pwm > 255)
    {
      pwm = 255;
    }

    if (pwm < -255)
    {
      pwm = -255;
    }
     
    if (pwm == 0)
    {
      OCR0A = 0;
      OCR0B = 0;
    }
    else if (pwm > 0)
    {
      OCR0A = pwm;
      OCR0B = 0;
    }
    else if (pwm < 0)
    {
      OCR0A = 0;
      OCR0B = -pwm;
    }
}


class Identification
{
  public:
    Identification()
    {
      a00 = 0.0;
      a01 = 0.0;
      a10 = 0.0;
      a11 = 0.0;

      b0  = 0.0;
      b1  = 0.0;


      da00 = 0.0;
      da01 = 0.0;
      da10 = 0.0;
      da11 = 0.0;

      db0  = 0.0;
      db1  = 0.0;

      loss = 0.0;
    }

    void step(float x0, float x1, float x0_prev, float x1_prev, float u, float lr = 0.001, float momentum = 0.99)
    {
      float x0_hat = a00*x0_prev + a01*x1_prev + b0*u;
      float x1_hat = a10*x0_prev + a11*x1_prev + b1*u;

      float e0 = x0 - x0_hat;
      float e1 = x1 - x1_hat;

      da00 = momentum*da00 + (1 - momentum)*e0*x0_prev;
      da01 = momentum*da01 + (1 - momentum)*e0*x1_prev;
      da10 = momentum*da10 + (1 - momentum)*e1*x0_prev;
      da11 = momentum*da11 + (1 - momentum)*e1*x1_prev;

      db0 = momentum*db0 + (1 - momentum)*e0*u;
      db1 = momentum*db1 + (1 - momentum)*e1*u;

      a00+= lr*da00;
      a01+= lr*da01;
      a10+= lr*da10;      
      a11+= lr*da11;

      b0+= lr*db0;
      b1+= lr*db1;


       
      float k = 0.9;
      loss = k*loss + (1.0 - k)*(e0*e0 + e1*e1);
    }

    public:
      float a00, a01, a10, a11, b0, b1;
      float da00, da01, da10, da11, db0, db1;
      float loss;

};


int main() 
{ 
    terminal << "\n\nstarting\n";

    Timer timer; 
    timer.init();
     
    Encoder encoder;
    encoder.init();

    KalmanFilter filter;

    int cnt = 0;


    uint64_t time_now  = 0;
    uint64_t time_prev = 0;


    init_pwm();

    uint32_t  idx_r = 0;
    int32_t required[] = {0, 50, 0, -50};

    float k0 = 0.01;
    float k1 = 0.01;
    float k2 = 0.0001;

   
    Identification model;
    
    float distance_hat = 0;
    float velocity_hat = 0;
    float distance_hat_prev = 0;
    float velocity_hat_prev = 0;


    while (1)
    { 
      if ((cnt%200) == 0)
      {
        idx_r = (idx_r + 1)%4;
        terminal << model.a00 << " " <<  model.a01 << "\n";
        terminal << model.a10 << " " <<  model.a11 << "\n";
        terminal << "\n";
        terminal << model.b0 << " " <<  model.b1 << "\n";
        terminal << "\n";
        terminal << "loss = " << model.loss << "\n";
        terminal << "\n\n\n"; 
      }

     
        time_prev  = time_now;
        time_now   = timer.get_time();

        float dt = time_now - time_prev;
        encoder.update(dt); 
        
        auto distance = encoder.get_distance();
        auto velocity = encoder.get_velocity();


        filter.step(distance, velocity, 0.1, 0.1, dt*0.001);

        distance_hat_prev = distance_hat;
        velocity_hat_prev = velocity_hat;

        distance_hat = filter.get_x();
        velocity_hat = filter.get_v();

        auto distance_required = required[idx_r];

        float u = k0*distance_required - k1*distance_hat - k2*velocity_hat;
        u = 255*u;
              
        set_pwm(u);  

        float scale = 100.0;
        model.step(distance_hat/scale, velocity_hat/scale, distance_hat_prev/scale, velocity_hat_prev/scale, u/scale, 0.0001);
        


      timer.delay_ms(2);

      cnt++;
    }

  return 0;
}
