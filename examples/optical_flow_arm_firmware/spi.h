#ifndef _SPI_H_
#define _SPI_H_

#include "device.h"
#include <gpio.h>


class SPI
{
    public:
        SPI();
        ~SPI();
        
        uint8_t transfer(uint8_t b);

    private:
        Gpio<TGPIOB, 3, GPIO_MODE_OUT>  pin_sck;
        Gpio<TGPIOB, 3, GPIO_MODE_OUT>  pin_mosi;
        Gpio<TGPIOB, 3, GPIO_MODE_IN_PULLUP>   pin_miso;
};

#endif
