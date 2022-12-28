#ifndef _ENCODER_H_
#define _ENCODER_H_

#include <stdint.h>

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
};

#endif