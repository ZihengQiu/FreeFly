#include "stm32f4xx.h"                  // Device header
#include "includes.h"
#include "led.h"
#include "SEGGER_SYSVIEW.h"

#define TASK_STK_LEN 0x0800

OS_STK MainTaskStk[TASK_STK_LEN];
OS_STK Task1Stk[TASK_STK_LEN];
OS_STK Task2Stk[TASK_STK_LEN];
OS_STK Task3Stk[TASK_STK_LEN];

uint32_t states, task1_cnt, task2_cnt;

extern void My_Systick_Config(uint32_t reload_value);

void task_led_on(void *pdata)
{
	while(1)
	{
		led_on();
		task1_cnt++;
		OSTimeDly(100);
	}
}

void task_led_off(void *pdata)
{
	while(1)
	{
		led_off();
		task2_cnt++;
		OSTimeDly(200);
	}
}

void first_task(void *pdata)
{
	// Initialization
	
	// ahb : 84000000Hz 
	// needed tick: 100Hz = 10ms -> reload val = 840000

	// Create application task
	
	// OSTaskCreate(task1, (void *)0, &Task1Stk[TASK_STK_LEN - 1], 5);
	// delete task
	My_Systick_Config(840000);
	led_init();
	OSTaskCreate(task_led_on, (void *)0, &Task2Stk[TASK_STK_LEN - 1], 6);
	OSTaskCreate(task_led_off, (void *)0, &Task3Stk[TASK_STK_LEN - 1], 7);
	OSTaskNameSet(6, (INT8U *)"LED_ON", (INT8U *)"LED_ON_ERR");
	OSTaskNameSet(7, (INT8U *)"LED_OFF", (INT8U *)"LED_OFF_ERR");
	OSTaskDel(OS_PRIO_SELF);
}

int main(void)
{
	
	OSInit();
	OS_TRACE_INIT();//	SEGGER_SYSVIEW_Conf();
	OS_TRACE_START();//	SEGGER_SYSVIEW_Start();
	OSTaskCreate(first_task, (void *)0, &MainTaskStk[TASK_STK_LEN-1], 2);
	OSTaskNameSet(2, (INT8U *)"FIRST_TASK", (INT8U *)"FIRST_TASK_ERR");
	OSStart();
	
	return 0;
}
