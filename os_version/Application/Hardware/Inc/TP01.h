#ifndef __TP01_H
#define __TP01_H

#include "stm32f4xx.h"                  // Device header
#include "ucos_ii.h"
#include "flow_decode.h"

void TP01_GPIOInit_2(void);
void TP01_ConfigInit_2(void);
void TP01_GPIOInit_6(void);
void TP01_ConfigInit_6(void);
void TP01Init(void);
void TP01_SendByte(uint8_t data);
void TP01_SendString(char data[]);

uint8_t TP01_ReceiveByte(void);

void TP01_Parse();

#endif