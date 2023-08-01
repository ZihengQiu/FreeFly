#include "stm32f4xx.h"                  // Device header
#include "Bluetooth.h"

char TransmitData[1005];

void Bluetooth_GPIOInit(void)
{
	RCC->AHB1ENR |= (uint16_t)0x01<<1; //GPIOB: PB6-TX PB7-RX
	RCC->APB2ENR |= (uint16_t)0x01<<4; // USART1
	
	//Alternate function mode : 10
	GPIOB->MODER &= ~((uint16_t)0x0003<<12);
	GPIOB->MODER &= ~((uint16_t)0x0003<<14);
	GPIOB->MODER |= (uint16_t)0x0002<<12;
	GPIOB->MODER |= (uint16_t)0x0002<<14;
	
	//Output push-pull (reset state) : 0
	GPIOB->OTYPER |= (uint16_t)0x0001<<6;
	GPIOB->OTYPER |= (uint16_t)0x0001<<7;
	
	// High speed : 10
	GPIOB->OSPEEDR &= ~((uint16_t)0x0003<<12);
	GPIOB->OSPEEDR &= ~((uint16_t)0x0003<<14);
	GPIOB->OSPEEDR |= (uint16_t)0x0002<<12;
	GPIOB->OSPEEDR |= (uint16_t)0x0002<<14;
	
	// medium speed
	
	//No pull-up, pull-down : 00
	GPIOB->PUPDR &= ~((uint16_t)0x0003<<12);
	GPIOB->PUPDR &= ~((uint16_t)0x0003<<14); 
	
	//Alternate function selection AF7(USART1) : 0111
	GPIOB->AFR[0] &= ~((uint32_t)(uint16_t)0xE<<24);
	GPIOB->AFR[0] &= ~((uint32_t)0xE<<28);
	GPIOB->AFR[0] |= (uint32_t)0x7<<24;
	GPIOB->AFR[0] |= (uint32_t)0x7<<28;
}

void Bluetooth_ConfigInit(void)
{
	// USART enable : 1
	USART1->CR1 |= 0x01<<13;
	
	// Baund rate : 38400 	USARTDIV £º 45.5625 2d9 		FCK £º27993600 REAL:26880000
	// Baund rate : 9600 	USARTDIV £º 182.25 b6.4 		FCK £º27993600 
	// Baund rate : 9600 	USARTDIV £º 273.4375 0x111.7		FCK £º42000000
	//			Fraction : .4375 * 16 = 7			Mantissa = 111
	// Baund rate : 9600 	USARTDIV £º 546.875		FCK: 84000000
	//			Fraction : E			Mantissa = 222
	
	
	//Baund rate : 9600 
	//USARTDIV = Mantissa+(Fraction/(8*2)) = 104.166666 = 0b110 1000 0011 = 0x683 -> 104.1875
	//Baund rate = fck / (8*2*USARTDIV)
	USART1->BRR = 0x222E; // 683 AF0 2D9 AE0
	
	//Word length -> 1 Start bit, 8 Data bits, n Stop bit : 0
	USART1->CR1 &= ~(0x01<<12);
	
	//Parity control disable : 0
	USART1->CR1 &= ~(0x01<<10);
	
	//RXNE interrupt enable
	USART1->CR1 |= 0x01<<5;
	
	//Transmitter enable : 1
	USART1->CR1 |= 0x01<<3;
	
	//Reciver enable : 1
	USART1->CR1 |= 0x01<<2;
}

void Bluetooth_SendByte(uint8_t data)
{
	USART1->DR = (data & (uint16_t)0x01FF);
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

uint16_t Bluetooth_ReceiveByte(void)
{
	uint8_t data = 0;
	while((USART1->SR&(1<<5))==0);
	data = USART1->DR & (uint16_t)0x01FF;
	if(data != 0)
		return  data;
	return 0;
}

/*void Bluetooth_SendString(uint8_t data[], uint8_t len)
{
	for(uint8_t i=0; i<len; i++)
	{
		Bluetooth_SendByte(data[i]);data[3]
	}
}*/

void Bluetooth_SendString(char data[])
{
	uint8_t i = 0;
	while(data[i] != 0)
	{
		Bluetooth_SendByte(data[i]);
		i++;
	}
}

void BluetoothInit()
{
	Bluetooth_GPIOInit();
	Bluetooth_ConfigInit();
}
