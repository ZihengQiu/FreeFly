#include "stm32f4xx.h"

uint32_t systick_cnt;


// ahb : 84MHz
// needed tick: 1 tick = 10ms = 100Hz -> reload val = 840000
void My_Systick_Config(uint32_t reload_value)
{
	
    SysTick->LOAD |= reload_value-1;	// set reload value
    SysTick->VAL |= SysTick->LOAD;	// clear current value
    SysTick->CTRL |= 1<<2;	// clock source : AHB
    SysTick->CTRL |= 1<<1;	// enable interrupt
    SysTick->CTRL |= 0x01;	// enable counter
}

