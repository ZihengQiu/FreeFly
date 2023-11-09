#include "stm32f4xx.h"                  // Device header
#include "bluetooth.h"
#include "stm32f4xx_usart.h"
#include <stdarg.h>
#include <stdio.h>

char TransmitData[1005];

void Bluetooth_GPIOInit(void)
{
	RCC->AHB1ENR |= (uint16_t)0x01<<1; // GPIOB: PB6-TX PB7-RX
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
	// Baund rate : 38400 	USARTDIV 锟斤�? 45.5625 2d9 		FCK 锟斤�?27993600 REAL:26880000
	// Baund rate : 9600 	USARTDIV 锟斤�? 182.25 b6.4 		FCK 锟斤�?27993600 
	// Baund rate : 9600 	USARTDIV 锟斤�? 273.4375 0x111.7		FCK 锟斤�?42000000
	//			Fraction : .4375 * 16 = 7			Mantissa = 111
	// Baund rate : 9600 	USARTDIV 锟斤�? 546.875		FCK: 84000000
	//			Fraction : E			Mantissa = 222
	
	
	//Baund rate : 9600 
	//USARTDIV = Mantissa+(Fraction/(8*2)) = 104.166666 = 0b110 1000 0011 = 0x683 -> 104.1875
	//Baund rate = fck / (8*2*USARTDIV)
	// USART1->BRR = 0x222E; // 683 AF0 2D9 AE0

	//Baudrate : 115200, Fpclk = 84MHz
	//USARTDIV = Fpclk/(16*baudrate) = 45.5625 = 0b101101.1001 = 0x2D9 (refer to reference manual)
	USART1->BRR = 0x2D9;
	
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
	
	// USART enable : 1
	USART1->CR1 |= 0x01<<13;
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

void Bluetooth_SendString(char data[])
{
	uint8_t i = 0;
	while(data[i] != 0)
	{
		Bluetooth_SendByte(data[i]);
		i++;
	}
}

void Bluetooth_SendSignedNum(int16_t num)
{
	if(num < 0)
	{
		Bluetooth_SendByte('-');
		num = -num;
	}
	sprintf(TransmitData, "%d", num);
	Bluetooth_SendString(TransmitData);
}

void BluetoothInit()
{
	Bluetooth_GPIOInit();
	Bluetooth_ConfigInit();
}

int fputc(int ch, FILE *f)
{
	Bluetooth_SendByte(ch);
	return ch;
}

void BT_Printf(char *format, ...)
{
	char string[100];
	va_list args;
	va_start(args, format); // receive arguments from "..."
	vsprintf(string, format, args);
	va_end(args); // clean memory
	Bluetooth_SendString(string);
}