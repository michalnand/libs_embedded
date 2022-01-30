#ifndef _HMC5883_H_
#define _HMC5883_H_

#include <i2c_interface.h>

class HMC5883
{
    public:
        HMC5883();
        virtual ~HMC5883();

        int init(I2C_Interface *i2c_);

        void read();

    public:
        int x, y, z;

    private:
        I2C_Interface *i2c;
};

#endif