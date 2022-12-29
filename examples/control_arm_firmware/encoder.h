#ifndef _ENCODER_H_
#define _ENCODER_H_

#include <stdint.h>
#include <gpio.h>


/*
    pins : 
        PA0, PA1
*/
#define ENCODER_CH_A     (0)
#define ENCODER_CH_B     (1)
#define ENCODER_PORT      TGPIOA

#define PULSES_PER_ROTATION     ((int)140)
#define WHEEL_CIRCUMREFERENCE   ((int)100)


class Encoder
{
    public:
        Encoder();
        ~Encoder();

        void init();

        //input : two measurements time interval in [ms]
        //returns  traveled distance in mm
        int32_t update(int32_t dt_ms);

        //returns traveled distance in mm
        int32_t get_distance();

        //returns speed in mm/s
        int32_t get_velocity();
 
 
    private:
        int32_t distance, distance_prev, velocity;

        //Gpio<ENCODER_PORT, ENCODER_CH_A, GPIO_MODE_IN_PULLUP>  pa;
        //Gpio<ENCODER_PORT, ENCODER_CH_B, GPIO_MODE_IN_PULLUP>  pb;
};

#endif