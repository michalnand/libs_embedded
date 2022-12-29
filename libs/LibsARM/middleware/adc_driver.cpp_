#include <stdint.h>
#include <stm32f7xx_hal.h>
#include <adc_driver.h>
#include <gpio.h>


volatile uint32_t g_adc_current_idx;
volatile uint16_t g_adc_channels[ADC_CHANNELS_COUNT];
volatile uint16_t g_adc_result[ADC_CHANNELS_COUNT];

#ifdef __cplusplus
extern "C" {
#endif
 
void ADC_IRQHandler(void)
{
    //read value
    g_adc_result[g_adc_current_idx] = ADC_GetConversionValue(ADC1);

    ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);

    //triger next conversion
    g_adc_current_idx = (g_adc_current_idx + 1)%ADC_CHANNELS_COUNT;

    uint32_t ch = g_adc_channels[g_adc_current_idx];

    ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_112Cycles);
    ADC_SoftwareStartConv(ADC1);
}

#ifdef __cplusplus
}
#endif

ADC_driver::ADC_driver()
{

}

ADC_driver::~ADC_driver()
{

}

void ADC_driver::init()
{
    g_adc_current_idx  = 0; 

    for (unsigned int i = 0; i < ADC_CHANNELS_COUNT; i++)
    {
        g_adc_result[i] = 0;
    }

    for (unsigned int i = 0; i < ADC_CHANNELS_COUNT; i++)
    {
        g_adc_channels[i] = i;
    }


    Gpio<TGPIOA, 0, GPIO_MODE_AN> adc_0;
    Gpio<TGPIOA, 1, GPIO_MODE_AN> adc_1;
    Gpio<TGPIOA, 2, GPIO_MODE_AN> adc_2;
    Gpio<TGPIOA, 3, GPIO_MODE_AN> adc_3;
    Gpio<TGPIOA, 4, GPIO_MODE_AN> adc_4;
    Gpio<TGPIOA, 5, GPIO_MODE_AN> adc_5;
    Gpio<TGPIOA, 6, GPIO_MODE_AN> adc_6;
    Gpio<TGPIOA, 7, GPIO_MODE_AN> adc_7;


    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    ADC_InitTypeDef ADC_InitStruct;
    ADC_InitStruct.ADC_ScanConvMode         =   DISABLE;
    ADC_InitStruct.ADC_Resolution           =   ADC_Resolution_12b;
    ADC_InitStruct.ADC_ContinuousConvMode   =   DISABLE;
    ADC_InitStruct.ADC_ExternalTrigConv     =   ADC_ExternalTrigConv_T1_CC1;
    ADC_InitStruct.ADC_ExternalTrigConvEdge =   ADC_ExternalTrigConvEdge_None;
    ADC_InitStruct.ADC_DataAlign            =   ADC_DataAlign_Right;
    ADC_InitStruct.ADC_NbrOfConversion      =   1;

    ADC_Init(ADC1, &ADC_InitStruct);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_15Cycles);
    ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);

    ADC_Cmd(ADC1, ENABLE);

    
    NVIC_InitTypeDef         NVIC_InitStructure;

    //configure and enable ADC1 interrupt
    NVIC_InitStructure.NVIC_IRQChannel                      = ADC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority    = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority           = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                   = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    //start first conversion
    ADC_SoftwareStartConv(ADC1);
}

uint16_t* ADC_driver::read()
{
    return (uint16_t*)g_adc_result;
}

/*
int ADC_driver::read_single(unsigned int channel)
{
    ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_15Cycles);
    ADC_SoftwareStartConv(ADC1);

    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC))
    { 

    }

    return ADC_GetConversionValue(ADC1);
}
*/
