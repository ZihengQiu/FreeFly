#include "stm32f4xx.h"
#include "esp-01s.h"
#include "stm32f4xx_usart.h"
#include <stdarg.h>
#include <stdio.h>

void ESP_GpioInit(void)
{
    RCC->AHB1ENR = (uint16_t)0x01<<0;   //GPIOAEN
    RCC->APB2ENR = (uint32_t)0x01<<17;  //USART2EN

    GPIOA->MODER &= ~((uint16_t)0x0003<<4);
    GPIOA->MODER &=  ((uint16_t)0x0002<<4); //PA2 AF mode
    GPIOA->MODER &= ~((uint16_t)0x0003<<6);
    GPIOA->MODER &=  ((uint16_t)0x0002<<6); //PA3 AF mode

    GPIOA->OTYPER |= ((uint16_t)0x0001<<2);
    GPIOA->OTYPER |= ((uint16_t)0x0001<<3);

    //set 25Mhz
    GPIOA->OSPEEDR &= ~((uint16_t)0x0003<<4);
    GPIOA->OSPEEDR &= ~((uint16_t)0x0003<<6);
    GPIOA->OSPEEDR |=  ((uint16_t)0x0001<<4);
    GPIOA->OSPEEDR |=  ((uint16_t)0x0001<<6);

    //no pull-up or -down
    GPIOA->PUPDR &= ~((uint16_t)0x0003<<4);
    GPIOA->PUPDR &= ~((uint16_t)0x0003<<6);

    //set AF (USART1-3: AF7)
    GPIOA->AFR[0] &= ~((uint16_t)0x000F<<8);
    GPIOA->AFR[0] &= ~((uint16_t)0x000F<<12);
    GPIOA->AFR[0] |=  ((uint16_t)0x0007<<8);
    GPIOA->AFR[0] |=  ((uint16_t)0x0007<<12);
}

void ESP_ConfigInit(void)
{
    //baudrate:115200
    USART2->BRR = 0x683;

    USART2->CR1 &= ~(0x01<<12); //word length: 1 start bit, 8 data bits, n stop bit
    USART2->CR1 &= ~(0x01<<10); //parity control disabled
    USART2->CR1 |=  (0x01<<5);  //RXNE interrupt enable
    USART2->CR1 |=  (0x01<<3);  //transmitter enable
    USART2->CR1 |=  (0x01<<2);  //receiver enable
    USART2->CR1 |=  (0x01<<13); //USART enable
}

void ESP_Init(void)
{
    ESP_GpioInit();
    ESP_ConfigInit();
}
