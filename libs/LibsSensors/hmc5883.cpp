#include "hmc5883.h"
#include <stdint.h>


#define I2C_ADDRESS ((unsigned char)0x3C)


HMC5883::HMC5883() 
{
    
}


HMC5883::~HMC5883()
{

}

int HMC5883::init(I2C_Interface *i2c_)
{
    x = 0;
    y = 0; 
    z = 0;

    i2c = i2c_;

    int result = 0;
    
    //config reg A, 75Hz data rate
    i2c->write_reg(I2C_ADDRESS, 0x00, (1<<4)|(1<<3)); 

    //config reg B, gain = 0.88Ga
    //i2c->write_reg(I2C_ADDRESS, 0x01, 0);
    i2c->write_reg(I2C_ADDRESS, 0x01, (1<<7)|(1<<6)|(1<<5));

    //mode reg, continuous measuring
    i2c->write_reg(I2C_ADDRESS, 0x02, 0);

    //read identification register
    if (i2c->read_reg(I2C_ADDRESS, 0x0A) != 0x48)
    {
        result = -1; 
    }
    else if (i2c->read_reg(I2C_ADDRESS, 0x0B) != 0x34)
    {
        result = -2;
    }
    else if (i2c->read_reg(I2C_ADDRESS, 0x0C) != 0x33)
    {
        result = -3; 
    } 

    return result;
} 

void HMC5883::read() 
{
    x = ((uint16_t)i2c->read_reg(I2C_ADDRESS, 0x03)) << 8;
    x|= (uint16_t)i2c->read_reg(I2C_ADDRESS, 0x04);

    z = ((uint16_t)i2c->read_reg(I2C_ADDRESS, 0x05)) << 8;
    z|= (uint16_t)i2c->read_reg(I2C_ADDRESS, 0x06);

    y = ((uint16_t)i2c->read_reg(I2C_ADDRESS, 0x07)) << 8;
    y|= (uint16_t)i2c->read_reg(I2C_ADDRESS, 0x08);
}
