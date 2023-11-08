#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_usart.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/_stdint.h>

#include "os_cpu.h"

#include "bluetooth.h"
#include "control.h"

char bt_transmit_data[1005], bt_receive_data[1005];
uint8_t it_rec_data, rec_cnt;
BOOLEAN bt_received_flag = 0;

#define BT_REC_STAT_START 	0
#define BT_REC_STAT_BODY  	1
#define BT_REC_STAT_END   	2
#define BT_REC_START_SYMBOL '$'
#define BT_REC_END_SYMBOL_1 '\r'
#define BT_REC_END_SYMBOL_2 '\n'

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


	// enable interrupt
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitTypeDef NVIC_InitStructure;	
	NVIC_InitStructure.NVIC_IRQChannel=USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=2;
	NVIC_Init(&NVIC_InitStructure);

	USART1->CR1 |= 0x01<<5;	
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
	sprintf(bt_transmit_data, "%d", num);
	Bluetooth_SendString(bt_transmit_data);
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

void BTReceivePackage(void)
{
	 // 0:waiting for start symbol, 1:receiving message body while waiting for end symbol
	static uint8_t rec_status = BT_REC_STAT_START;

	if(rec_status == BT_REC_STAT_START)
	{
		if(it_rec_data == BT_REC_START_SYMBOL)
		{
			rec_status = BT_REC_STAT_BODY;
			rec_cnt = 0;
		}
	}
	else if(rec_status == BT_REC_STAT_BODY)
	{
		if(it_rec_data == BT_REC_END_SYMBOL_1 || it_rec_data == BT_REC_END_SYMBOL_2)
		{
			bt_received_flag = 1;
			bt_receive_data[rec_cnt] = '\0';
			rec_status = BT_REC_STAT_START;
		}
		else 
		{
			bt_receive_data[rec_cnt++] = it_rec_data;
		}
	}
}

void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE))
	{
		while((USART1->SR&(1<<5))==0);
		it_rec_data= USART1->DR & (uint16_t)0x01FF;
		BTReceivePackage();
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
}

char command_res[5][100] = {
	"Invalid command!\r\n",
	"PID set successfully!\r\n"
};
void BTShowCommandResult(uint8_t res)
{
	Bluetooth_SendString(command_res[res]);
}
char *command[10][20];
void BTCommandParse(void)
{
	if(bt_received_flag == 1)
	{
		bt_received_flag = 0;

		uint8_t len = 0;
		command[0][0] = strtok(bt_receive_data, " ");
		while(command[len][0] != NULL)
		{
			command[++len][0] = strtok(NULL, " ");
		}

		if(0 == strcmp(command[0][0], "pid"))
		{
			if(len != 6)
			{
				BTShowCommandResult(0);
				return;
			}
			uint8_t pid_i = 0;
			if(0 == strcmp(command[1][0], "i"))
			{
				pid_i = 1;
			}
			if(0 == strcmp(command[2][0], "roll"))
			{
				pid_roll[pid_i].kp = atof(command[3][0]);
				pid_roll[pid_i].ki = atof(command[4][0]);
				pid_roll[pid_i].kd = atof(command[5][0]);
				BTShowCommandResult(1);
			}
			else if(0 == strcmp(command[2][0], "pitch"))
			{
				pid_pitch[pid_i].kp = atof(command[3][0]);
				pid_pitch[pid_i].ki = atof(command[4][0]);
				pid_pitch[pid_i].kd = atof(command[5][0]);
				BTShowCommandResult(1);
			}
			else if(0 == strcmp(command[2][0], "yaw"))
			{
				pid_yaw[pid_i].kp = atof(command[3][0]);
				pid_yaw[pid_i].ki = atof(command[4][0]);
				pid_yaw[pid_i].kd = atof(command[5][0]);
				BTShowCommandResult(1);
			}
			else
			{
				BTShowCommandResult(0);
			}
		}
		else
		{
			BTShowCommandResult(0);
		}
	}
}