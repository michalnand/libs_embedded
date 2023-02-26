#include "stm32h7xx_hal.h"
#include "system_init.h"


void delay_loops(uint32_t loops)
{
  while (loops--)
  {
    __asm("nop");
  }
}

#define PE3_Pin GPIO_PIN_3
#define PE3_GPIO_Port GPIOE


int main(void)
{  
  system_init();

  // switch on GPIOE module for 3 user LEDs (LD1: PB0, LD2: PB7, LD3: PB14
  RCC->AHB4ENR |= RCC_AHB4ENR_GPIOEEN_Msk;
 
  // important "=", not "|=" as you would possibly do in STM32F4 ... 
  // (the reset states are 11 for MODER in STM32H7 - they were 00 in STM32F4...)  
  GPIOE->MODER = GPIO_MODER_MODE3_0;

  uint32_t loops; 
   
  for(;;){ 
    GPIOE->ODR    = 0;

   
    delay_loops(100000000);
    
    GPIOE->ODR    = (1<<3);

    delay_loops(10000000);
  }
}

/*
int main(void)
{ 
  #ifdef W25Qxx
		SCB->VTOR = QSPI_BASE;
	#endif
	MPU_Config();
	CPU_CACHE_Enable();


  HAL_Init();
  SystemClock_Config();


  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  HAL_GPIO_WritePin(PE3_GPIO_Port, PE3_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = PE3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PE3_GPIO_Port, &GPIO_InitStruct);


  while (1)
  {
    HAL_GPIO_TogglePin(PE3_GPIO_Port,PE3_Pin);
    delay_loops(1000000);
  }
  return 0;
} 
*/