#include "pwm.h"


PWM::PWM()
{

}

void PWM::init()
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);


    TIM_TimeBaseInitTypeDef TIM_BaseStruct;

    //set timer to 8bit mode
    //running at a frequency of 48MHz/(255*(10 – 1)) = 18.8kHz
    TIM_BaseStruct.TIM_ClockDivision       = TIM_CKD_DIV1;
    TIM_BaseStruct.TIM_CounterMode         = TIM_CounterMode_Up;
    TIM_BaseStruct.TIM_Period              = 0xFF;
    TIM_BaseStruct.TIM_Prescaler           = 10; 
    TIM_BaseStruct.TIM_RepetitionCounter   = 0;
    TIM_TimeBaseInit(TIM1, &TIM_BaseStruct);
    TIM_Cmd(TIM1, ENABLE);

    TIM_CtrlPWMOutputs(TIM1, ENABLE);



    TIM_OCInitTypeDef TIM_OCStruct;

    /* PWM mode 2 = Clear on compare match */
	/* PWM mode 1 = Set on compare match */
	TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCStruct.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCStruct.TIM_Pulse = 0;
	TIM_OC1Init(TIM1, &TIM_OCStruct); 
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);


    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_2);
    
    GPIO_InitTypeDef GPIO_InitStruct;

    //PA8 pwm output
    GPIO_InitStruct.GPIO_Pin    = GPIO_Pin_8;
    GPIO_InitStruct.GPIO_OType  = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd   = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Mode   = GPIO_Mode_AF;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    //PA11 motor way control
    GPIO_InitStruct.GPIO_Pin    = GPIO_Pin_11;
    GPIO_InitStruct.GPIO_OType  = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd   = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Mode   = GPIO_Mode_OUT;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
}


void PWM::set(unsigned int value)
{
    TIM1->CCR1 = value;
}

void PWM::set_motor(float speed)
{   
    unsigned int pwm_value = 0;

    if (speed > 1.0)
    {
        speed = 1.0;
    }

    if (speed < -1.0)
    {
        speed = -1.0;
    } 

    if (speed < 0)
    {
        speed = -speed;
        GPIOA->ODR|= (1<<11);
        pwm_value  = 255*(1.0 - speed);
    }
    else
    {
        GPIOA->ODR&= ~(1<<11);
        pwm_value  = 255*speed;
    }

    
    TIM1->CCR1 = pwm_value;
}

