#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#include "stm32f4xx.h"                  // Device header

void Bluetooth_GPIOInit(void);
void Bluetooth_ConfigInit(void);
void Bluetooth_SendByte(uint8_t data);
void Bluetooth_SendString(char data[]);
uint16_t Bluetooth_ReceiveByte(void);
void BluetoothInit(void);
void Bluetooth_SendSignedNum(int16_t num);
void BT_Printf(char *format, ...);
void BTReceivePackage(void);
// int fputc(int ch, FILE *f);

#endif
