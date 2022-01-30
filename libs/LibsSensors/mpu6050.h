#ifndef _MPU6050_H_
#define _MPU6050_H_

#include <i2c_interface.h>
#include <stdint.h>

class MPU6050 
{
    public:
        MPU6050();
        virtual ~MPU6050();

        int init(I2C_Interface *i2c_);

        void read();

    private:
        void delay_loops(uint32_t loops);

    public:
        int16_t ax, ay, az;
        int16_t gx, gy, gz;
        int16_t gx_offset, gy_offset, gz_offset; 

    private:
        I2C_Interface *i2c;

        uint8_t acc_scale, gyro_scale;

};

#endif