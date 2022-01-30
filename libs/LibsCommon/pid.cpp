#include "pid.h"

PID::PID()
{

}

PID::~PID()
{

}

void PID::init(float Kp, float Ki, float Kd, float out_range_min, float out_range_max, float dt)
{
    this->k0 = Kp + Ki*dt;
    this->k1 = -Kp;

    this->k0d = Kd/dt;
    this->k1d = -2.0*Kd/dt;
    this->k2d = Kd/dt;

    this->out_range_min = out_range_min;
    this->out_range_max = out_range_max;

    this->alpha = 2.0*dt/(Kd + 2.0*dt);

    reset();
}
        
void PID::reset(float output_initial)
{
    e0      = 0.0;
    e1      = 0.0;
    e2      = 0.0;
    
    d0      = 0.0;
    d1      = 0.0;
    
    y       = output_initial;
    y_der   = 0.0;
}

float PID::step(float x_setpoint, float x_measured)
{
    e2 = e1;
    e1 = e0;
    e0 = x_setpoint - x_measured;

    float y_pi    = k0*e0 + k1*e1;
    y_der   = (1.0 - alpha)*y_der + alpha*(k0d*e0 + k1d*e1 + k2d*e2);
    
    y = y + y_pi + y_der;

    if (y < out_range_min)
    {
        y = out_range_min;
    }

    if (y > out_range_max)
    {
        y = out_range_max;
    }

    return y;
}
