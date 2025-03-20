#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_usart.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "os_cpu.h"

#include "TP01.h"

#define IF_26 6

unsigned char rec;
uint16_t my_x = 0;
uint16_t my_y = 0;
uint16_t to_ground_distance = 0;
uint16_t time_cost = 0;
uint8_t if_valid = 0;
uint8_t confidence = 0;

int ret; 

void TP01_GPIOInit_2(void)
{
    RCC->AHB1ENR |= (uint16_t)0x01<<3; // GPIOD: PD5-TX PD6-RX
    RCC->APB1ENR |= (uint32_t)0x01<<17; // USART2
    // RCC->CFGR |= (uint16_t)0x0004 << 10; // APB1 = 42MHz

    //Alternate function mode : 10
    GPIOD->MODER &= ~((uint16_t)0x0003<<10);
    GPIOD->MODER &= ~((uint16_t)0x0003<<12);
    GPIOD->MODER |= (uint16_t)0x0002<<10;
    GPIOD->MODER |= (uint16_t)0x0002<<12;

    //Output push-pull (reset state) : 0
    GPIOD->OTYPER &= ~(uint16_t)0x0001<<5;
    GPIOD->OTYPER &= ~(uint16_t)0x0001<<6;

    // High speed : 25Mhz-01, 50Mhz-10
    GPIOD->OSPEEDR &= ~((uint16_t)0x0003<<10);
    GPIOD->OSPEEDR &= ~((uint16_t)0x0003<<12);
    GPIOD->OSPEEDR |= (uint16_t)0x0001<<10;
    GPIOD->OSPEEDR |= (uint16_t)0x0001<<12;

    //No pull-up, pull-down : 00
    GPIOD->PUPDR &= ~((uint16_t)0x0003<<10);
    GPIOD->PUPDR &= ~((uint16_t)0x0003<<12);

    //Alternate function selection AF7(USART2) : 0111
    GPIOD->AFR[0] &= ~((uint32_t)0xF<<20);
    GPIOD->AFR[0] &= ~((uint32_t)0xF<<24);
    GPIOD->AFR[0] |= (uint32_t)0x7<<20;
    GPIOD->AFR[0] |= (uint32_t)0x7<<24;
}

void TP01_GPIOInit_6(void)
{
    RCC->AHB1ENR |= (uint16_t)0x01<<0; // GPIOA: PA11-TX PA12-RX
    RCC->APB2ENR |= (uint16_t)0x01<<5; // USART6

    //Alternate function mode : 10
    GPIOA->MODER &= ~((uint16_t)0x0003<<22);
    GPIOA->MODER &= ~((uint16_t)0x0003<<24);
    GPIOA->MODER |= (uint16_t)0x0002<<22;
    GPIOA->MODER |= (uint16_t)0x0002<<24;

    //Output push-pull (reset state) : 0
    GPIOA->OTYPER &= ~(uint16_t)0x0001<<11;
    GPIOA->OTYPER &= ~(uint16_t)0x0001<<12;

    // High speed : 25Mhz-01, 50Mhz-10
    GPIOA->OSPEEDR &= ~((uint16_t)0x0003<<22);
    GPIOA->OSPEEDR &= ~((uint16_t)0x0003<<24);
    GPIOA->OSPEEDR |= (uint16_t)0x0003<<22;
    GPIOA->OSPEEDR |= (uint16_t)0x0003<<24;

    //No pull-up, pull-down : 00
    GPIOA->PUPDR &= ~((uint16_t)0x0003<<22);
    GPIOA->PUPDR &= ~((uint16_t)0x0003<<24);

    //Alternate function selection AF8(USART6) : 1000
    GPIOA->AFR[1] &= ~((uint32_t)0xF<<12);
    GPIOA->AFR[1] &= ~((uint32_t)0xF<<16);
    GPIOA->AFR[1] |= (uint32_t)0x8<<12;
    GPIOA->AFR[1] |= (uint32_t)0x8<<16;
}

void TP01_ConfigInit_2(void)
{
    //Baund rate : 9600 
	//USARTDIV = Mantissa+(Fraction/(8*2)) = 104.166666 = 0b110 1000 0011 = 0x683 -> 104.1875
	//Baund rate = fck / (8*2*USARTDIV)
	// USART1->BRR = 0x222E; // 683 AF0 2D9 AE0

	//Baudrate : 115200, Fpclk = 84MHz
	//USARTDIV = Fpclk/(16*baudrate) = 45.5625 = 0b101101.1001 = 0x2D9 (refer to reference manual)

    //baud rate: 115200
    //USARTDIV = 42M/(16*115200) = 22.7864 = 
    USART2->BRR = 0x16C;

    //Word length -> 1 Start bit, 8 Data bits, 1 Stop bit : 0
    USART2->CR1 &= ~(0x01<<12);

    //Parity control disable : 0
    USART2->CR1 &= ~(0x01<<10);

    //Transmitter enable
    USART2->CR1 |= 0x01<<3;

    //receiver enable
    USART2->CR1 |= 0x01<<2;

    //enable USART2
    USART2->CR1 |= 0x01<<13;

    //enable interrupt
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_Init(&NVIC_InitStructure);

    //RX interrupt enable
    USART2->CR1 |= 0x01<<5;
}

void TP01_ConfigInit_6(void)
{
    //baud rate: 115200
    USART6->BRR = 0x2D9;

    //Word length -> 1 Start bit, 8 Data bits, 1 Stop bit : 0
    USART6->CR1 &= ~(0x01<<12);

    //Parity control disable : 0
    USART6->CR1 &= ~(0x01<<10);

    //transmitter enable
    USART6->CR1 |= 0x01<<3;

    //receiver enable
    USART6->CR1 |= 0x01<<2;

    //enable USART6
    USART6->CR1 |= 0x01<<13;

    //enable interrupt
    USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_Init(&NVIC_InitStructure);

    //RX interrupt enable
    USART6->CR1 |= 0x01<<5;
}

void TP01Init(void)
{
    if(IF_26 == 2)
    {
        TP01_GPIOInit_2();
        TP01_ConfigInit_2();
    }else if(IF_26 == 6)
    {
        TP01_GPIOInit_6();
        TP01_ConfigInit_6();
    }
}

void TP01_SendByte(uint8_t data)
{
    if(IF_26 == 2)
    {
        while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
        USART_SendData(USART2, data);
    }else if(IF_26 == 6)
    {
        while(USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET);
        USART_SendData(USART6, data);
    }
}

void TP01_SendString(char data[])
{
    uint8_t i = 0;
    while(data[i] != '\0')
    {
        TP01_SendByte(data[i]);
        i++;
    }
}

void USART2_IRQHandler(void)
{
    if(USART_GetITStatus(USART2, USART_IT_RXNE))
    {
        while((USART2->SR & (1<<5)) == 0);
        rec = USART2->DR;
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}

void USART6_IRQHandler(void)
{
    if(USART_GetITStatus(USART6, USART_IT_RXNE))
    {
        while((USART6->SR & (1<<5)) == 0);
        rec = USART6->DR;
        ret = up_parse_char(rec);
        // USART_ClearITPendingBit(USART6, USART_IT_RXNE);
    }
}

void TP01_Parse()
{
    if(!ret)
    {
        my_x = up_data.flow_x_integral;
        my_y = up_data.flow_y_integral;
        to_ground_distance = up_data.ground_distance;
        time_cost = up_data.integration_timespan;
        if_valid = up_data.valid;
        confidence = up_data.tof_confidence;
    }
    // else
    // {
    //     my_x = 0;
    //     my_y = 0;
    //     to_ground_distance = 0;
    //     if_valid = 0;
    //     confidence = 0;
    // }
}