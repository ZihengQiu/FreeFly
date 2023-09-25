#include "stm32f4xx.h"                  // Device header
#include "myI2C.h"
#include "MyDelay.h"
void MyI2C_GPIOInit(void)
{
	//GPIO_Init
	//Alternate function mode : 10
	GPIOB->MODER &= ~((uint16_t)0x0003<<16);
	GPIOB->MODER &= ~((uint16_t)0x0003<<18);
	GPIOB->MODER |= (uint16_t)0x0002<<16;
	GPIOB->MODER |= (uint16_t)0x0002<<18;
	
	
	GPIOB->OTYPER |= (uint16_t)0x0001<<8;
	GPIOB->OTYPER |= (uint16_t)0x0001<<9;
	
	GPIOB->OSPEEDR = (uint16_t)0x0003<<16;
	GPIOB->OSPEEDR = (uint16_t)0x0003<<18;
	
	//pull up
	GPIOB->PUPDR &= ~((uint16_t)0x0003<<16);
	GPIOB->PUPDR &= ~((uint16_t)0x0003<<18); 
	GPIOB->PUPDR |= (uint16_t)0x0001<<16;
	GPIOB->PUPDR |= (uint16_t)0x0001<<18; 
	
	GPIOB->AFR[1] |= (uint16_t)0x1<<2;
	GPIOB->AFR[1] |= (uint16_t)0x1<<6;
}

void MyI2C_SetMasterMode(void)
{
	//Set prec = 8
	I2C1->CR2 &= ~((uint16_t)(uint16_t)0x003F);
	I2C1->CR2 |= (uint16_t)0x0001<<2;
	
	//TRISE = 9    (P672
	I2C1->TRISE &= ~((uint16_t)0x003F);
	I2C1->TRISE |= (uint16_t)0x1001;
	
	//Enable peripheral 
	I2C1->CR1 |= (uint16_t)0x0001;
	
	//Disenable PEC
	I2C1->CR1 &= ~((uint16_t)0x0001<<5);
	
	//address
	I2C1->OAR1 &= ~((uint16_t)0x007F);
	I2C1->OAR1 |= (uint16_t)0x0001;
	
	//ccr=28 -> 100kHz SCL
	I2C1->CCR &= ~((uint16_t)0x0001<<15);
	I2C1->CCR &= ~((uint16_t)0x0FFF);
	I2C1->CCR |= 11000;
}

void MyI2C_SetStart(FunctionalState NewState)
{
	if(NewState == ENABLE)
	{
		I2C1->CR1 |= (uint16_t)0x0001<<8;
	}else{
		I2C1->CR1 &= ~((uint16_t)0x0001<<8);
	}
}

void MyI2C_SetStop(FunctionalState NewState)
{
	if(NewState == ENABLE)
	{
		I2C1->CR1 |= (uint16_t)0x0001<<9; //10 0000 0000
	}else{
		I2C1->CR1 &= ~((uint16_t)0x0001<<9);
	}
}

void MyI2C_SetAck(FunctionalState NewState)
{
	if(NewState == ENABLE)
	{
		I2C1->CR1 |= (uint16_t)0x0001<<10;
	}else{
		I2C1->CR1 &= ~((uint16_t)0x0001<<10);
	}
}

void MyI2C_MasterReceiver(void)
{
	MyI2C_SetStart(ENABLE);
}

void MyI2C_SendData(uint8_t Data)
{
	I2C1->DR = Data;
}

uint8_t MyI2C_ReceiveData()
{
  return (uint8_t)I2C1->DR;
}	

void MyI2C_Send7bitAddress(uint8_t Address, uint8_t Direction)
{
	if(Direction == Direction_Receiver)
	{
		// Set to Reveive (Set LSB = 1)
		Address |= ((uint16_t)0x0001);
	}else{
		// Set to Read/Write (Set LSB = 0)
		Address &= (uint8_t)~((uint8_t)I2C_OAR1_ADD0);
	}
	I2C1->DR = Address;
}

void MyI2C_MasterReceive()
{
	uint8_t Address = (uint8_t)0x00D1;  //1101000+1
	MyI2C_SetStart(ENABLE);
	MyI2C_Send7bitAddress(Address,Direction_Receiver);
}

void MyI2C_MasterRead(uint8_t Address)
{
	MyI2C_Send7bitAddress(Address,Direction_Transmitter);
}

uint8_t MyI2C_CheckBusy()
{
	// if(((I2C1->SR2>>1) & 1) == 1)// BUSY
	// {
	// 	return 0x0;
	// }
	// return 0x1; 
	uint16_t temp = I2C1->SR2;
	if(((temp>>1) & 1) == 1)// BUSY
	{
		return 0x0;
	}
	return 0x1;
}

uint8_t MyI2C_CheckMasterModeSelected()
{
	// BUSY, MSL and SB 0x00030001
	if(((I2C1->SR1 & 1) == 1) && ((I2C1->SR2 & 1) == 1) && ((I2C1->SR2>>1)&1) == 1 )
	{
		return 0x1;
	}
	return 0x0; 
}

uint8_t MyI2C_CheckMasterTransmitterModeSelected()
{
	//BUSY, MSL, ADDR, TXE and TRA 0x00070082
	// if(((I2C1->SR1>>1 & 1) == 1) && ((I2C1->SR1>>7 & 1) == 1) && ((I2C1->SR2)&1) == 1 && ((I2C1->SR2>>1)&1) == 1 && ((I2C1->SR2>>2)&1) == 1 )
	// {
	// 	return 0x1;
	// }
	// return 0x0;  
	
	uint16_t temp1 = I2C1->SR1, temp2 = I2C1->SR2;
	if(((temp1>>1 & 1) == 1) && ((temp1>>7 & 1) == 1) && ((temp2)&1) == 1 && ((temp2>>1)&1) == 1 && ((temp2>>2)&1) == 1 )
	{
		if(temp1>>9&1 == 1) I2C1->SR1 |= 0x400;
		return 0x1;
	}
	return 0x0;	//BUG:ÖÙ²Ã¶ªÊ§
}

uint8_t MyI2C_CheckMasterByteTransmitted()
{
	//TRA, BUSY, MSL, TXE and BTF  0x00070084
	if(((I2C1->SR1>>2 & 1) == 1) && ((I2C1->SR1>>7 & 1) == 1) && ((I2C1->SR2)&1) == 1 && ((I2C1->SR2>>1)&1) == 1 && ((I2C1->SR2>>2)&1) == 1 )
	{
		return 0x1;
	}
	return 0x0;  
}

uint8_t MyI2C_CheckMasterReceiverModeSelected()
{
	//BUSY, MSL and ADDR     00030002
	if (((I2C1->SR1>>1 & 1) == 1) && ((I2C1->SR2 & 1) == 1) && ((I2C1->SR2>>1)&1) == 1 )
	{
		return 0x1;
	}
	return 0x0;  
}

uint8_t MyI2C_CheckRXNE()
{
	//0x10000040
	if(((I2C1->SR1>>6 & 1) == 1)) //SET
	{
		return 0x1;
	}
	return 0x0;  
}

uint8_t MyI2C_CheckTXE()
{
	//0x10000040
	if(((I2C1->SR1>>7 & 1) == 1)) //SET
	{
		return 0x1;
	}
	return 0x0;  
}

uint8_t MyI2C_ReadRegister_1Bytes(uint8_t SlaveAddress, uint8_t RegisterAddress)
{
	uint8_t Data = 0;
	
	while(!MyI2C_CheckBusy());
	MyI2C_SetStart(ENABLE);
	while(!MyI2C_CheckMasterModeSelected());
	MyI2C_Send7bitAddress(SlaveAddress,Direction_Transmitter);
	while(!MyI2C_CheckMasterTransmitterModeSelected());
	MyI2C_SendData(RegisterAddress);
	while(!MyI2C_CheckMasterByteTransmitted());

	MyI2C_SetStart(ENABLE);
	while(!MyI2C_CheckMasterModeSelected());
	MyI2C_Send7bitAddress(SlaveAddress, Direction_Receiver); 
	while(!MyI2C_CheckMasterReceiverModeSelected());
	MyI2C_SetStop(DISABLE);
	MyI2C_SetAck(DISABLE);
	while(!MyI2C_CheckRXNE());
	Data = MyI2C_ReceiveData();
	MyI2C_SetStop(ENABLE);
	MyI2C_SetAck(ENABLE);
	
	return Data;
}
uint16_t MyI2C_ReadRegister_2Bytes(uint8_t SlaveAddress, uint8_t RegisterAddress)
{
	uint8_t dataH = 0, dataL = 0;
  	uint16_t data = 0;
	
	MyI2C_SetAck(ENABLE);
	MyI2C_SetStop(DISABLE);
	
  	while(!MyI2C_CheckBusy());  
  	MyI2C_SetStart(ENABLE);
  	while(!MyI2C_CheckMasterModeSelected());          
  	MyI2C_Send7bitAddress(SlaveAddress,Direction_Transmitter);
  	while(!MyI2C_CheckMasterTransmitterModeSelected()); 
  	MyI2C_SendData(RegisterAddress);
  	while(!MyI2C_CheckMasterByteTransmitted());         
	
  	MyI2C_SetStart(ENABLE);                                    
  	while(!MyI2C_CheckMasterModeSelected());
  	MyI2C_Send7bitAddress(SlaveAddress, Direction_Receiver);
  	while(!MyI2C_CheckMasterReceiverModeSelected());
  	MyI2C_SetStop(DISABLE);
	
  	while(!MyI2C_CheckRXNE());
  	dataH = MyI2C_ReceiveData();
	
  	MyI2C_SetAck(DISABLE);
  	MyI2C_SetStop(ENABLE);

  	while(!MyI2C_CheckRXNE());
  	dataL = MyI2C_ReceiveData();
	MyI2C_SetAck(DISABLE); 
  	MyI2C_SetStop(ENABLE);
  	MyI2C_SetAck(ENABLE);
	
  	data = (uint16_t)(dataH<<8) + dataL;
  	return data;
}

void MyI2C_BurstReadRegister(uint8_t SlaveAddress, uint8_t RegisterAddress, uint8_t *Buffer, uint8_t Length)
{
	while(!MyI2C_CheckBusy());
	MyI2C_SetStart(ENABLE);
	while(!MyI2C_CheckMasterModeSelected());
	MyI2C_Send7bitAddress(SlaveAddress,Direction_Transmitter);
	while(!MyI2C_CheckMasterTransmitterModeSelected());
	MyI2C_SendData(RegisterAddress);
	while(!MyI2C_CheckMasterByteTransmitted());
	
	MyI2C_SetStart(ENABLE);
	while(!MyI2C_CheckMasterModeSelected());
	MyI2C_Send7bitAddress(SlaveAddress, Direction_Receiver); 
	while(!MyI2C_CheckMasterReceiverModeSelected());
	MyI2C_SetStop(DISABLE);
	MyI2C_SetAck(ENABLE);
	
	for(uint8_t i = 0; i < Length; i++)
	{
		while(!MyI2C_CheckRXNE());
		Buffer[i] = MyI2C_ReceiveData();
	}
	
	MyI2C_SetAck(DISABLE);
	MyI2C_SetStop(ENABLE);
}

void MyI2C_WriteRegister(uint8_t SlaveAddress, uint8_t RegisterAddress, uint8_t Value)
{ 
	
	while(!MyI2C_CheckBusy());
	
	MyI2C_SetStart(ENABLE);
	while(!MyI2C_CheckMasterModeSelected());
	
	MyI2C_Send7bitAddress(SlaveAddress, Direction_Transmitter);
	while(!MyI2C_CheckMasterTransmitterModeSelected());
	
	MyI2C_SendData(RegisterAddress);
	while(!MyI2C_CheckMasterByteTransmitted());
	
	MyI2C_SendData(Value);
	while(!MyI2C_CheckMasterByteTransmitted());
	
	MyI2C_SetStop(ENABLE);
}

void MyI2C_WriteRegisterHMC5883(uint8_t SlaveAddress, uint8_t RegisterAddress, uint8_t Value)
{ 
	MyI2C_SetAck(ENABLE);
	while(!MyI2C_CheckBusy());
	
	MyI2C_SetStart(ENABLE);
	while(!MyI2C_CheckMasterModeSelected());
	
	MyI2C_Send7bitAddress(SlaveAddress, Direction_Transmitter);
	while(!MyI2C_CheckMasterTransmitterModeSelected());
	
	MyI2C_SendData(RegisterAddress);
	while(!MyI2C_CheckTXE());
	
	MyI2C_SendData(Value);
	while(!MyI2C_CheckMasterByteTransmitted());
	
	MyI2C_SetStop(ENABLE);
	MyI2C_SetAck(DISABLE);
}

void MyI2C_Init(void)
{
	RCC->AHB1ENR |= (uint16_t)0x0001<<1; //PB8 PB9
	RCC->APB1ENR |= (uint16_t)0x0001<<21; // I2C1
	
	MyI2C_GPIOInit();
	MyI2C_SetMasterMode();
}
