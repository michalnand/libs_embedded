#ifndef _ADC_DRIVER_H_
#define _ADC_DRIVER_H_

#define ADC_CHANNELS_COUNT  ((uint32_t)8)

class ADC_driver
{
    public:
        ADC_driver();
        virtual ~ADC_driver();

        void init();
        uint16_t* read();
        uint32_t get_conversion();
};


#endif
