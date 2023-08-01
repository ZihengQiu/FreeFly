#include <stm32f4xx.h>
#include "MyDelay.h"

#define RCC_AHB1 ((unsigned int)0x40023800)
#define RCC_AHB1ENR *(unsigned int *)(RCC_AHB1 + 0x30)

#define GPIOA_ADDRESS ((unsigned int)0x40020000)
#define GPIOA_MODER *(unsigned int *)(GPIOA_ADDRESS + 0x00)
#define GPIOA_OTYPER *(unsigned int *)(GPIOA_ADDRESS + 0x04)
#define GPIOA_OSPEEDER *(unsigned int *)(GPIOA_ADDRESS + 0x08)
#define GPIOA_PUPDR *(unsigned int *)(GPIOA_ADDRESS + 0x0C)
#define GPIOA_ODR *(unsigned int *)(GPIOA_ADDRESS + 0x14)

void LED_ON(void);
void LED_OFF(void);
void Delay(unsigned int count);

int main() {

  RCC_AHB1ENR |= (1ul << 0);

  GPIOA_MODER &= ~(3ul << 2 * 5);
  GPIOA_MODER |= (1ul << 2 * 5);

  GPIOA_OTYPER |= ~(1ul << 5);

  GPIOA_OSPEEDER &= ~(3ul << 2 * 5);
  GPIOA_OSPEEDER |= (1ul << 2 * 5);

  GPIOA_PUPDR &= ~(3ul << 2 * 5);

  while (1) {
    LED_ON();
    Delay_ms(1000);
    LED_OFF();
    Delay_ms(500);
  }
}

void LED_ON(void) { GPIOA_ODR |= (1ul << 5); }

void LED_OFF(void) { GPIOA_ODR &= (0ul << 5); }
