#include "mpu6050.h"
#include "mpu6050_regs.h"
#include <stdint.h>


enum Ascale 
{
    AFS_2G = 0,
    AFS_4G,
    AFS_8G,
    AFS_16G
};

enum Gscale 
{
    GFS_250DPS = 0,
    GFS_500DPS,
    GFS_1000DPS,
    GFS_2000DPS
};

MPU6050::MPU6050()
{
    ax = 0;
    ay = 0;
    ax = 0;

    gx = 0;
    gy = 0;
    gz = 0;

    gx_offset = 0;
    gy_offset = 0;
    gz_offset = 0;

    this->acc_scale    = Ascale::AFS_8G;
    this->gyro_scale   = Gscale::GFS_1000DPS;
}

MPU6050::~MPU6050()
{

}

int MPU6050::init(I2C_Interface *i2c_)
{
    ax = 0;
    ay = 0;
    ax = 0;

    gx = 0;
    gy = 0;
    gz = 0;

    gx_offset = 0;
    gy_offset = 0;
    gz_offset = 0;


    this->i2c = i2c_;

    // Initialize MPU6050 device

    //wake up device-don't need this here if using calibration function below
    i2c->write_reg(MPU6050_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors 

    delay_loops(100000);

    //260Hz bandwidth
    i2c->write_reg(MPU6050_ADDRESS, CONFIG, 0x00); 
 
    //set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    i2c->write_reg(MPU6050_ADDRESS, SMPLRT_DIV, 0x00);
    
    //set accelerometer scale
    i2c->write_reg(MPU6050_ADDRESS, ACCEL_CONFIG, acc_scale << 3);

    //set gyro scale 
    i2c->write_reg(MPU6050_ADDRESS, GYRO_CONFIG, gyro_scale << 3);
    
    delay_loops(100000); 
    
    if (i2c->read_reg(MPU6050_ADDRESS, WHO_AM_I_MPU6050) != 0x68)
    {
        return -1;
    } 
 
    uint32_t count = 100;

    int32_t gx_ofs_cal = 0;
    int32_t gy_ofs_cal = 0; 
    int32_t gz_ofs_cal = 0;

    
    for (uint32_t i = 0; i < count; i++)
    {
        int16_t gx = i2c->read_reg_16bit(MPU6050_ADDRESS, GYRO_XOUT_H);
        int16_t gy = i2c->read_reg_16bit(MPU6050_ADDRESS, GYRO_YOUT_H);
        int16_t gz = i2c->read_reg_16bit(MPU6050_ADDRESS, GYRO_ZOUT_H);

        gx_ofs_cal+= (int32_t)gx;
        gy_ofs_cal+= (int32_t)gy;
        gz_ofs_cal+= (int32_t)gz;

        delay_loops(1000);
    }  

    gx_offset = gx_ofs_cal/((int32_t)count);
    gy_offset = gy_ofs_cal/((int32_t)count);
    gz_offset = gz_ofs_cal/((int32_t)count);

    return 0; 
} 

void MPU6050::read() 
{   
    ax = i2c->read_reg_16bit(MPU6050_ADDRESS, ACCEL_XOUT_H);
    ay = i2c->read_reg_16bit(MPU6050_ADDRESS, ACCEL_YOUT_H);
    az = i2c->read_reg_16bit(MPU6050_ADDRESS, ACCEL_ZOUT_H);

    gx = i2c->read_reg_16bit(MPU6050_ADDRESS, GYRO_XOUT_H);
    gy = i2c->read_reg_16bit(MPU6050_ADDRESS, GYRO_YOUT_H);
    gz = i2c->read_reg_16bit(MPU6050_ADDRESS, GYRO_ZOUT_H);

    gx-= gx_offset;
    gy-= gy_offset; 
    gz-= gz_offset;
}

void MPU6050::delay_loops(uint32_t loops)
{
    while (loops--)
    {
        __asm volatile ("nop");
    } 
}