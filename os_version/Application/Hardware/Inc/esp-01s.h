#ifndef __ESP_01S_H
#define __ESP_01S_H

#include "stm32f4xx.h"

void Usart2_SendByte(uint8_t);
uint8_t Usart2_ReceiveByte(void);

void ESP_Init(void);
void ESP_GpioInit(void);
void ESP_ConfigInit(void);

#endif __ESP_01S_H                    