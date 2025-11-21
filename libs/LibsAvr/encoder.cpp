#include "Encoder.h"
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

/*
    pins : 
        PC2 (PCTIN10)
        PC3 (PCTIN11)
*/



#define ENC_CH_A            (1<<2)
#define ENC_CH_B            (1<<3) 

#define PULSES_PER_ROTATION     ((int)140)
#define WHEEL_CIRCUMREFERENCE   ((int)100)


volatile int32_t g_encoder = 0;


ISR(PCINT1_vect) 
{
    uint8_t tmp     = PINC;
    uint8_t ch_a    = tmp&ENC_CH_A;
    uint8_t ch_b    = tmp&ENC_CH_B;

    if (ch_a != 0)
    {
        if (ch_b == 0)
        {
            g_encoder+= 1;
        }
        else
        {
            g_encoder-= 1;
        }
    }
}
 
Encoder::Encoder()
{
    
} 

Encoder::~Encoder()
{

}

void Encoder::init()
{
    //pin change interrupt enable, PCINT1
    PCICR|=     (1<<1);
    PCMSK1|=    (1<<2); 

    this->distance_prev = 0;
    this->distance      = 0;
    this->velocity      = 0;
}

int32_t Encoder::update(int32_t dt_ms)
{
    volatile int32_t raw_steps;

    cli();
    raw_steps = g_encoder;
    sei();

    this->distance_prev = this->distance;
    this->distance      = (WHEEL_CIRCUMREFERENCE*raw_steps)/PULSES_PER_ROTATION;

    if (dt_ms == 0)
        dt_ms = 1;
        
    this->velocity      = ((this->distance - this->distance_prev)*1000)/dt_ms;

    return this->distance;
}

int32_t Encoder::get_distance()
{
    return this->distance;
}
 

int32_t Encoder::get_velocity()
{
    return this->velocity;
}
