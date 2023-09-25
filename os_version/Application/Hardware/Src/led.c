#include "stm32f4xx.h" 

void led_init(void)
{
	RCC->AHB1ENR |= (1ul << 0);		   // GPIOA
	GPIOA->MODER &= ~((3ul << 2 * 5)); // 1100 0000 0000 -> 0011 1111 1111
	GPIOA->MODER |= ((1ul << 2 * 5));  // 0100 0000 0000
	GPIOA->OTYPER &= ~((1ul << 5));
	GPIOA->OSPEEDR &= ~((3ul << 2 * 5)); // 1100 0000 0000 -> 0011 1111 1111
	GPIOA->OSPEEDR |= ((1ul << 2 * 5));	 // 0100 0000 0000
	GPIOA->PUPDR &= ~((3ul << 2 * 5));	 // 1100 0000 0000 -> 0011 1111 1111
}

void led_on(void)
{
	GPIOA->ODR |= 1ul << 5; // LED_ON
}

void led_off(void)
{
	GPIOA->ODR &= 0ul << 5; // LED_OFF
}
