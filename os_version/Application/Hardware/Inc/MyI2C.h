#ifndef __MYI2C_H
#define __MYI2C_H

#include "stm32f4xx.h"                  // Device header

#define	Direction_Transmitter		(uint8_t)0x00
#define Direction_Receiver			(uint8_t)0x01

void MyI2C_Init(void);
void MyI2C_GPIOInit(void);
void MyI2C_SetMasterMode(void);

void MyI2C_SetStart(FunctionalState);
void MyI2C_SetStop(FunctionalState);
void MyI2C_SetAck(FunctionalState);
void MyI2C_MasterReceiver(void);
void MyI2C_SendData(uint8_t);
uint8_t MyI2C_ReceiveData(void);
void MyI2C_WriteRegister(uint8_t,uint8_t,uint8_t);
void MyI2C_WriteRegisterHMC5883(uint8_t, uint8_t, uint8_t);

uint8_t MyI2C_CheckBusy(void);
uint8_t MyI2C_CheckMasterModeSelected(void);
uint8_t MyI2C_CheckMasterTransmitterModeSelected(void);
uint8_t MyI2C_CheckMasterByteTransmitted(void);
uint8_t MyI2C_CheckMasterReceiverModeSelected(void);
uint8_t MyI2C_CheckRXNE(void);

uint8_t MyI2C_ReadRegister_1Bytes(uint8_t, uint8_t);
uint16_t MyI2C_ReadRegister_2Bytes(uint8_t, uint8_t);

#endif
