#ifndef _IMU_H_
#define _IMU_H_

#include "fmath.h"
#include "kalman_filter.h"

#define G_const ((float)9.8067)


class IMU
{
    public:
        IMU();
        virtual ~IMU();
        void init(); 

        //accelerometer input   : ax, ay, az;   [m/s^2] 
        //gyroscope input       : gx, gy, gz;   [rad/s]
        //dt                    : [ms]
        //returns               : x, y, z;      [rad]
        Vect3d<float> step(float ax, float ay, float az, float gx, float gy, float gz, float dt = 0.01);

        Vect3d<float> convert_euler_angles(float theta, float phi, float p, float q, float r);

    public:
        Vect3d<float> result;

    private:
        float alpha;

        KalmanFilter k_roll, k_pitch;
};

#endif