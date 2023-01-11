#include "spi.h"



SPI::SPI()
{
    pin_sck  = 0;
    pin_mosi = 0;
}

SPI::~SPI() 
{
    
}


uint8_t SPI::transfer(uint8_t b)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        if (b&(1<<7))
        {
            pin_mosi = 1;
        }
        else
        {
            pin_mosi = 0;
        }

        pin_sck = 1;
        __asm("nop");
        __asm("nop");
        __asm("nop");
        __asm("nop");

        b<<= 1;

        if (pin_miso != 0)
        {
            b|= (1<<0);
        }

        pin_sck = 0;
        __asm("nop");
        __asm("nop");
        __asm("nop");
        __asm("nop");
    }

    return b;
}