#ifndef __ESP_01S_H
#define __ESP_01S_H

#include "stm32f4xx.h"

void Usart2_SendByte(uint8_t);
uint16_t Usart2_ReceiveByte(void);
void Usart2_SendString(char[]);

void ESP_Init(void);
void ESP_GpioInit(void);
void ESP_ConfigInit(void);
void ESP_NVICInit(void);

void USART2_IRQHandler(void);

void wifi_connect(void);

#endif
