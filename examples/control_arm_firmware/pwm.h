#ifndef _PWM_H_
#define _PWM_H_

#include "device.h"


class PWM
{
    public:
        PWM();
        void init();
        void set(unsigned int value);
        void set_motor(float speed);

};


#endif