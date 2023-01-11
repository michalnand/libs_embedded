#ifndef _ADNS3080_H_
#define _ADNS3080_H_

#include "spi.h"

// signal delay time, microseconds
#define ADNS3080_T_IN_RST             500
#define ADNS3080_T_PW_RESET           10
#define ADNS3080_T_SRAD_MOT           75
#define ADNS3080_T_SWW                50
#define ADNS3080_T_SRAD               50
#define ADNS3080_T_LOAD               10
#define ADNS3080_T_BEXIT              4

// pixel dimensions:
#define ADNS3080_PIXELS_X             30
#define ADNS3080_PIXELS_Y             30

// registers: 
#define ADNS3080_PRODUCT_ID           0x00
#define ADNS3080_CONFIGURATION_BITS   0x0a
#define ADNS3080_MOTION_CLEAR         0x12
#define ADNS3080_FRAME_CAPTURE        0x13
#define ADNS3080_PIXEL_BURST          0x40
#define ADNS3080_MOTION_BURST         0x50
#define ADNS3080_PRODUCT_ID_VALUE     0x17


class ADNS3080
{
    public:
        ADNS3080();
        virtual ~ADNS3080();

        int init(SPI *spi, bool led_mode = false, bool resolution = false);

        void displacement(int8_t *dx, int8_t *dy);
        void capture_frame(uint8_t *buffer);

    private:
        void write_register(uint8_t reg, uint8_t value);
        uint8_t read_register(uint8_t reg);

        void delay(unsigned int time);


    private: 
        SPI *spi;

    private:
        Gpio<TGPIOB, 3, GPIO_MODE_OUT>  pin_reset;
        Gpio<TGPIOB, 3, GPIO_MODE_OUT>  pin_ncs;
};

#endif