#include "imu.h"


IMU::IMU()
{
    result.x = 0;
    result.y = 0;
    result.z = 0;

    this->alpha = 0.1;
}

IMU::~IMU() 
{

}

void IMU::init()
{
    result.x = 0;
    result.y = 0;
    result.z = 0;

    this->alpha = 0.1;
}

Vect3d<float> IMU::step(float ax, float ay, float az, float gx, float gy, float gz, float dt)
{
    float var_angle  = 5.0*PI/180.0;
    float var_dangle = 5.0*PI/180.0;

    var_angle   = var_angle*var_angle;
    var_dangle  = var_dangle*var_dangle;

    ax = clamp(ax, -G_const, G_const);
    ay = clamp(ay, -G_const, G_const);
    az = clamp(az, -G_const, G_const);

    Vect3d<float> acc_result;

    acc_result.x = -fatan2(az, ay); //roll
    acc_result.y = fatan2(az, ax);  //pitch
    acc_result.z = 0;               //yaw


    Vect3d<float> gyro_result;
    //shrink small noise from gyro
    gyro_result.x = -shrink(gx, -0.01f, 0.01f); //roll
    gyro_result.y = -shrink(gy, -0.01f, 0.01f); //pitch
    gyro_result.z =  shrink(gz, -0.01f, 0.01f); //yaw
   

    /*
    //shrink small noise from gyro
    gx = -shrink(gx, -0.01f, 0.01f); //roll
    gy = -shrink(gy, -0.01f, 0.01f); //pitch
    gz =  shrink(gz, -0.01f, 0.01f); //yaw

    Vect3d<float> gyro_result;
    gyro_result = convert_euler_angles(result.x, result.y, gx, gy, gz);
    */

    //complementary filter
    //result.x = alpha*acc_result.x   + (1.0 - alpha)*(result.x + gyro_result.x*dt);
    //result.y = alpha*acc_result.y   + (1.0 - alpha)*(result.y + gyro_result.y*dt);

    //kalman filter
    result.x = k_roll.step(acc_result.x, gyro_result.x,  var_angle, var_dangle, dt);
    result.y = k_pitch.step(acc_result.y, gyro_result.y, var_angle, var_dangle, dt);
    result.z = result.z             + gyro_result.z*dt*0.001;
    
    return result;
}



Vect3d<float> IMU::convert_euler_angles(float phi, float theta, float p, float q, float r)
{
    Vect3d<float> result;

    float sin_phi   = fsin(phi);
    float cos_phi   = fcos(phi);
    float tan_theta = ftan(theta);
    float sec_theta = fsec(theta); 

    result.x        = p +  sin_phi*tan_theta*q + cos_phi*tan_theta*r;
    result.y        =      cos_phi*q           - sin_phi*r;
    result.z        =      sin_phi*sec_theta*q + cos_phi*sec_theta*r;

    return result;
}


