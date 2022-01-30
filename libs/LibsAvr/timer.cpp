#include "timer.h"
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define MCU_FREQ  ((uint32_t)16000000)

struct sTaskItem
{
    uint32_t    period;
    uint32_t    next_time_call;
    Thread      *callback_class;
};

volatile uint32_t g_time = 0;
volatile sTaskItem g_task_list[TIMER_MAX_TASKS];

 
ISR(TIMER0_COMPA_vect)
{
  for (unsigned int i = 0; i < TIMER_MAX_TASKS; i++)
  {
        if (g_task_list[i].callback_class != nullptr && g_time > g_task_list[i].next_time_call)
        {
            g_task_list[i].next_time_call = g_time + g_task_list[i].period;
            g_task_list[i].callback_class->main();
        }
   }

    g_time++;
}



Timer::Timer()
{
    
} 

Timer::~Timer()
{

}

void Timer::init(uint32_t frequency)
{
    current_ptr = 0;
    g_time      = 0;

    for (unsigned int i = 0; i < TIMER_MAX_TASKS; i++)
    {
        g_task_list[i].period           = 0;
        g_task_list[i].next_time_call   = 0;
        g_task_list[i].callback_class   = nullptr;
    }

    // Set the Timer Mode to CTC
    TCCR0A |= (1 << WGM01);

    // Set the value that you want to count to
    OCR0A = ((uint32_t)MCU_FREQ)/((uint32_t)64*frequency) - 1;
  
    // set prescaler to 64 and start the timer
    TCCR0B = (1 << CS01)|(1 << CS00);

    TIMSK0 |= (1 << OCIE0A);    //Set the ISR COMPA vect

    sei();
}

int Timer::add_task(Thread *callback_class, unsigned int period_ms)
{
    if (current_ptr < TIMER_MAX_TASKS)
    {
        cli();

        g_task_list[current_ptr].period           = period_ms;
        g_task_list[current_ptr].next_time_call   = g_time + period_ms;
        g_task_list[current_ptr].callback_class   = callback_class;
        current_ptr++;

        sei();

        return 0;
    }
    else
    {
        return -1;
    }
}


uint32_t Timer::get_time()
{
  volatile uint32_t tmp;

  cli();
  tmp = g_time;
  sei();

  return tmp;
}

void Timer::delay_ms(uint32_t time_ms)
{
    cli();
    time_ms = time_ms + g_time;
    sei();

    volatile uint32_t g_time_;

    do
    {
      cli();
      g_time_ = g_time;
      sei();

      __asm("nop");
    }
    while (time_ms > g_time_);
}