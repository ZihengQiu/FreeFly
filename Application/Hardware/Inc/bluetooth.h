#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#include "stm32f4xx.h"                  // Device header
#include "ucos_ii.h"

void Bluetooth_GPIOInit(void);
void Bluetooth_ConfigInit(void);
void Bluetooth_SendByte(uint8_t data);
void Bluetooth_SendString(char data[]);
uint16_t Bluetooth_ReceiveByte(void);
void BluetoothInit(void);
void Bluetooth_SendSignedNum(int16_t num);
void BT_Printf(char *format, ...);
void BTCommandParser(void);
// int fputc(int ch, FILE *f);

extern char bt_receive_data[1005];
extern uint8_t it_rec_data, rec_cnt;
extern BOOLEAN bt_received_flag;

#endif
