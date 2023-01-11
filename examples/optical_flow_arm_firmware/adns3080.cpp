#include "adns3080.h"


ADNS3080::ADNS3080()
{
 
}


ADNS3080::~ADNS3080()
{

}


int ADNS3080::init(SPI *spi, bool led_mode, bool resolution)
{
    /*
    spi in mode MSB first
    */

    pin_ncs   = 1;
    pin_reset = 1;

    delay(ADNS3080_T_PW_RESET);   
    pin_reset = 0;
    delay(ADNS3080_T_IN_RST);              


    // Configure sensor:
    // LED Shutter    High resolution
    uint8_t mask = (led_mode << 6) | (resolution << 4);      
    write_register( ADNS3080_CONFIGURATION_BITS, mask );

    // Check Connection
    if(read_register(ADNS3080_PRODUCT_ID) == ADNS3080_PRODUCT_ID_VALUE) 
    {
        return 0;
    } 
    else 
    {
        return -1;
    } 
}

void ADNS3080::displacement(int8_t *dx, int8_t *dy)
{
    pin_ncs = 0;

    spi->transfer(ADNS3080_MOTION_BURST);
    delay( ADNS3080_T_SRAD_MOT );

    //check if motion occured
    uint8_t motion= spi->transfer(0x00) & (1<<7);

    if  (motion != 0)
    {
        *dx = spi->transfer(0x00);
        *dy = spi->transfer(0x00);
    }
    else
    {
        *dx = 0;
        *dy = 0;
    }
  
    pin_ncs = 1;
}


void ADNS3080::capture_frame(uint8_t *buffer)
{
    write_register(ADNS3080_FRAME_CAPTURE, 0x83);

    pin_ncs = 0;

    spi->transfer( ADNS3080_PIXEL_BURST );
    delay( ADNS3080_T_SRAD );

    uint8_t b = 0;

    while( (b & (1<<6)) == 0 ) 
    {
        b = spi->transfer(0x00);
        delay( ADNS3080_T_LOAD );  
    }

    for (unsigned int i = 0; i < ADNS3080_PIXELS_Y*ADNS3080_PIXELS_X; i++)
    {
        buffer[i] = b;
        b = spi->transfer(0x00);
        delay( ADNS3080_T_LOAD );  
    }

    pin_ncs = 1;

    delay( ADNS3080_T_LOAD + ADNS3080_T_BEXIT );
}






void ADNS3080::write_register(uint8_t reg, uint8_t value)
{
    pin_ncs = 0;

    spi->transfer(reg|(1<<7));
    spi->transfer(value);
    
    pin_ncs = 1;

    delay( ADNS3080_T_SWW );
}

uint8_t ADNS3080::read_register(uint8_t reg)
{
    pin_ncs = 0;

    spi->transfer(reg);
    delay( ADNS3080_T_SRAD_MOT );
    uint8_t b = spi->transfer(0x00);

    pin_ncs = 1;
  
    return b;
}


void ADNS3080::delay(unsigned int time)
{   
    uint32_t loops = time*50;
    while (loops--)
    {
        __asm("nop");
    }
}