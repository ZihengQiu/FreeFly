#include "stm32f4xx.h"
#include "esp-01s.h"
#include "stm32f4xx_usart.h"
#include "ucos_ii.h"
#include "bluetooth.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define WIFI_NAME JNTM
#define WIFI_PASSWORD zz20030412

void ESP_GpioInit(void)
{
    RCC->AHB1ENR = (uint16_t)0x01<<0;   //GPIOA: PA2-TX PA3-RX
    RCC->APB1ENR = (uint32_t)0x01<<17;  //USART2EN

    GPIOA->MODER &= ~((uint16_t)0x0003<<4);
    GPIOA->MODER |=  ((uint16_t)0x0002<<4); //PA2 AF mode
    GPIOA->MODER &= ~((uint16_t)0x0003<<6);
    GPIOA->MODER |=  ((uint16_t)0x0002<<6); //PA3 AF mode

    GPIOA->OTYPER |= ((uint16_t)0x0001<<2);
    GPIOA->OTYPER |= ((uint16_t)0x0001<<3);

    //set 25Mhz
    GPIOA->OSPEEDR &= ~((uint16_t)0x0003<<4);
    GPIOA->OSPEEDR &= ~((uint16_t)0x0003<<6);
    GPIOA->OSPEEDR |=  ((uint16_t)0x0002<<4);
    GPIOA->OSPEEDR |=  ((uint16_t)0x0002<<6);

    //no pull-up or -down
    GPIOA->PUPDR &= ~((uint16_t)0x0003<<4);
    GPIOA->PUPDR &= ~((uint16_t)0x0003<<6);

    //set AF (USART1-3: AF7)
    GPIOA->AFR[0] &= ~((uint16_t)0x000E<<8);
    GPIOA->AFR[0] &= ~((uint16_t)0x000E<<12);
    GPIOA->AFR[0] |=  ((uint16_t)0x0007<<8);
    GPIOA->AFR[0] |=  ((uint16_t)0x0007<<12);
}

void ESP_ConfigInit(void)
{
    //baudrate:115200
    USART2->BRR = 0x2D9;

    USART2->CR1 &= ~(0x01<<12); //word length: 1 start bit, 8 data bits, n stop bit
    USART2->CR1 &= ~(0x01<<10); //parity control disabled
    USART2->CR1 |=  (0x01<<5);  //RXNE interrupt enable
    USART2->CR1 |=  (0x01<<3);  //transmitter enable
    USART2->CR1 |=  (0x01<<2);  //receiver enable
    USART2->CR1 |=  (0x01<<13); //USART enable
}

void ESP_NVICInit(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}

void ESP_Init(void)
{
    ESP_NVICInit();
    ESP_GpioInit();
    ESP_ConfigInit();
}

void Usart2_SendByte(uint8_t data)
{
    USART1->DR = (data & (uint16_t)0x01FF);
    while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}

uint16_t Usart2_ReceiveByte(void)
{
    uint16_t data = 0;
    while ((USART2->SR & (1<<5)) == 0);
    data = USART2->DR & (uint16_t)0x01FF;
    if(data != 0)
        return data;
    return 0;
}

void Usart2_SendString(char data[])
{
    uint8_t i = 0;
    while(data[i] != 0)
    {
        Usart2_SendByte(data[i]);
        i++;
    }
}

void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != SET)
    {
        Bluetooth_SendString("receive data!\r\n");
    
    }
    USART_ClearITPendingBit(USART2, USART_IT_RXNE);
}

void wifi_connect(void)
{
    Usart2_SendString("AT+CWMODE=1\r\n");
    Bluetooth_SendString("set station mode!\r\n");
    OSTimeDly(1000);

    Usart2_SendString("AT+CWJAP_DEF=\"JNTM\",\"zz20030412\"\r\n");
    Bluetooth_SendString("trying to connect!\r\n");
    while(1);
}
