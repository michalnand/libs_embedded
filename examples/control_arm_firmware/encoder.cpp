#include "Encoder.h"
#include <stdint.h>
#include <device.h>


#ifdef __cplusplus
extern "C" {
#endif

volatile int32_t g_encoder;

void EXTI0_1_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line0) != RESET)
    { 
        uint8_t tmp     = GPIOA->IDR;
        uint8_t ch_a    = tmp&(1<<ENCODER_CH_A);
        uint8_t ch_b    = tmp&(1<<ENCODER_CH_B);

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

    EXTI_ClearITPendingBit(EXTI_Line0);
}

#ifdef __cplusplus
}
#endif
 
Encoder::Encoder()
{
    
}  

Encoder::~Encoder()
{

}

void Encoder::init()
{ 
    this->distance_prev = 0;
    this->distance      = 0;
    this->velocity      = 0;

    g_encoder           = 0;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_InitTypeDef EXTI_InitStruct;

    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
    GPIO_Init(GPIOA, &GPIO_InitStruct);


    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource4);
    
    EXTI_InitStruct.EXTI_Line    = EXTI_Line0;
    EXTI_InitStruct.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStruct);

    //NVIC interrupt for EXTI0 and EXTI1
    NVIC_SetPriority(EXTI0_1_IRQn, 0x0);
    NVIC_EnableIRQ(EXTI0_1_IRQn);
}

int32_t Encoder::update(int32_t dt_ms)
{
    volatile int32_t raw_steps;
 
    __disable_irq();
    raw_steps = g_encoder;
    __enable_irq();

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
