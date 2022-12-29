#include "kalman_filter.h"

KalmanFilter::KalmanFilter()
{
    this->x_hat = 0;
    this->v_hat = 0;
    this->p     = 0.5;
}

KalmanFilter::~KalmanFilter()
{
    
}

float KalmanFilter::step(float z, float dz, float pz, float pdz, float dt)
{
    if (dt == 0)
        dt  =1;
        
    dt = 0.001*dt;

    //store prev x_hat
    this->x_hat_prev = this->x_hat;

    //1, prediction
    //predict the state and uncertaininty
    this->x_hat  = this->x_hat + dz*dt;
    this->p      = this->p + (dt*dt)*pdz;

    //2, kalman gain
    float k      = this->p/(this->p + pz);

    //3, update
    this->x_hat  = this->x_hat + k*(z - this->x_hat);
    this->p      = (1.0 - k)*this->p;

    this->v_hat = (this->x_hat - this->x_hat_prev)/dt;
    
    return this->x_hat;
}

float KalmanFilter::get_x()
{
    return x_hat;
}

float KalmanFilter::get_v()
{
    return v_hat;
}
