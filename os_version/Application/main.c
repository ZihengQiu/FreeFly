#include "stm32f4xx.h"                  // Device header
#include "includes.h"

#include <stdio.h>
#include <stdlib.h>

#if (defined(OS_TRACE_EN) && (OS_TRACE_EN > 0u))
#include "SEGGER_SYSVIEW.h"                                   
#endif

#include "led.h"
#include "myI2C.h"
#include "gy86.h"
#include "motor.h"
#include "bluetooth.h"
#include "receiver.h"

#define TASK_STK_LEN 0x0800

OS_STK MainTaskStk[TASK_STK_LEN];
OS_STK Task1Stk[TASK_STK_LEN];
OS_STK Task2Stk[TASK_STK_LEN];
OS_STK Task3Stk[TASK_STK_LEN];
OS_STK Task4Stk[TASK_STK_LEN];
OS_STK Task5Stk[TASK_STK_LEN];

uint32_t states, task1_cnt, task2_cnt;

extern void My_Systick_Config(uint32_t reload_value);

uint16_t times;
uint8_t RxData;

void task_led_on(void *pdata)
{
	while(1)
	{
		led_on();
		task1_cnt++;
		OSTimeDly(1000);
	}
}

void task_led_off(void *pdata)
{
	while(1)
	{
		led_off();
		task2_cnt++;
		OSTimeDly(2000);
	}
}

void first_task(void *pdata) {
    // Initialization

    My_Systick_Config(840000); // AHB = 84MHz
	
    led_init();

	MyI2C_Init();
	times++;
	MPU6050Init();
	times++;
	// HMC5883Init();
	times++;
	
	BluetoothInit();
	
	TIM1_init();
	Motor_Init();
	
	OSTimeDly(1000);

//	TIM_SetCompare1(TIM3, 2000);
//	OSTimeDly(4000);
//	TIM_SetCompare1(TIM3, 1000);
//	OSTimeDly(4000);                                                
	
	Bluetooth_SendString("Initilization finished!\n");

    // create LED_ON task
    OSTaskCreate(task_led_on, (void *)0, &Task2Stk[TASK_STK_LEN - 1], 6);
    OSTaskNameSet(6, (INT8U *)"LED_ON", (INT8U *)"LED_ON_ERR");

    // create LED_OFF task
    OSTaskCreate(task_led_off, (void *)0, &Task3Stk[TASK_STK_LEN - 1], 7);
    OSTaskNameSet(7, (INT8U *)"LED_OFF", (INT8U *)"LED_OFF_ERR");

    OSTaskDel(OS_PRIO_SELF);
}

int main(void)
{
	
	OSInit();
	OS_TRACE_INIT(); //	SEGGER_SYSVIEW_Conf();
	OS_TRACE_START(); // SEGGER_SYSVIEW_Start();
	OSTaskCreate(first_task, (void *)0, &MainTaskStk[TASK_STK_LEN-1], 2);
	OSTaskNameSet(2, (INT8U *)"FIRST_TASK", (INT8U *)"FIRST_TASK_ERR");
	OSStart();
	return 0;
}
