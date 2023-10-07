#include "stm32f4xx.h"                  // Device header
#include "includes.h"
#include "ucos_ii.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#if (defined(OS_TRACE_EN) && (OS_TRACE_EN > 0u))
#include "SEGGER_SYSVIEW.h"                                   
#endif

#include "attitude.h"
#include "bluetooth.h"
#include "gy86.h"
#include "led.h"
#include "myI2C.h"
#include "mathkit.h"
#include "motor.h"
#include "receiver.h"

#define TASK_STK_LEN 0x0800

OS_STK MainTaskStk[TASK_STK_LEN];
OS_STK Task1Stk[TASK_STK_LEN];
OS_STK Task2Stk[TASK_STK_LEN];
OS_STK Task3Stk[TASK_STK_LEN];
OS_STK Task4Stk[TASK_STK_LEN];
OS_STK Task5Stk[TASK_STK_LEN];
OS_STK Task6Stk[TASK_STK_LEN];
OS_STK Task7Stk[TASK_STK_LEN];

uint16_t times;

extern Vec3d_t acc_offset, acc_scale, gyro_offset;

extern void My_Systick_Config(uint32_t reload_value);



void task_led_on(void *pdata)
{
	while(1)
	{
		led_on();
		OSTimeDly(1000);
	}
}

void task_led_off(void *pdata)
{
	while(1)
	{
		led_off();
		OSTimeDly(2000);
	}
}

void task_peripheral_init(void *pdata)
{
	BluetoothInit();
	printf("Bluetooth init finished!\n");
	times++;

	MyI2C_Init();
	printf("I2C init finished!\n");
	times++;
	
	GY86Init();
	printf("GY86 init finished!\n");
	times++;
	
	
	TIM1_init();
	Motor_Init();
	
	// OSTimeDly(1000);

//	TIM_SetCompare1(TIM3, 2000);
//	OSTimeDly(4000);
//	TIM_SetCompare1(TIM3, 1000);
//	OSTimeDly(4000);                                                
	
	printf("Initilization finished!\n");
	OSTaskDel(OS_PRIO_SELF);
}

void task_MPU6050(void *pdata)
{
	// AccCalibration(&acc_offset, &acc_scale);
	GyroCalibration(&gyro_offset);
	// MagCalibration(&mag_offset, &mag_scale);
	while(1)
	{
		Vec3d_t acc;
		// GetAccData(&acc);
		acc.x = (acc.x - acc_offset.x) * acc_scale.x;
		acc.y = (acc.y - acc_offset.y) * acc_scale.y;
		acc.z = (acc.z - acc_offset.z) * acc_scale.z;
		// printf("acc: %f, %f, %f\n", acc.x, acc.y, acc.z);

		Vec3d_t gyro;
		// GetGyroData(&gyro);
		gyro.x -= gyro_offset.x;
		gyro.y -= gyro_offset.y;
		gyro.z -= gyro_offset.z;
		// printf("gyro: %f, %f, %f\n", gyro.x, gyro.y, gyro.z);

		Vec3d_t mag;
		// GetMagData(&mag);
		printf("%f, %f, %f\n", mag.x, mag.y, mag.z);
		// printf("mag: %f, %f, %f\n", mag.x, mag.y, mag.z);
		
		OSTimeDly(100);
	}
}

void task_attitude_gyro(void *pdata)
{
	Vec4d_t q0 = {1, 0, 0, 0}, q1;
	Vec3d_t gyro0 = {0, 0, 0}, gyro1;
	while(1)
	{
		GetGyroData(&gyro1);
		GyroUpdateQuat(&q0, &q1, &gyro0, &gyro1, 0.1);
		q0 = q1;
		gyro0 = gyro1;
		OSTimeDly(100);
		printf("q: %f, %f, %f, %f\n", q0.w, q0.x, q0.y, q0.z);
	}
}

void task_attitude_acc(void *pdata)
{
	AccCalibration(&acc_offset, &acc_scale);
	printf("acc_offset: %f, %f, %f\n", acc_offset.x, acc_offset.y, acc_offset.z);
	Vec3d_t acc_data = {0, 0, 0}, euler = {0, 0, 0};
	while(1)
	{
		GetAccData(&acc_data);
		printf("acc: %f, %f, %f\n", acc_data.x, acc_data.y, acc_data.z);
		AccToEuler(&acc_data, &euler);
		printf("euler: %f, %f, %f\n", euler.x, euler.y, euler.z);
		OSTimeDly(100);
	}
}

void first_task(void *pdata) {
    // Initialization
    My_Systick_Config(840000); // AHB = 84MHz
	
    led_init();

    // create LED_ON task
    OSTaskCreate(task_led_on, (void *)0, &Task2Stk[TASK_STK_LEN - 1], 6);
    OSTaskNameSet(6, (INT8U *)"LED_ON", (INT8U *)"LED_ON_ERR");

    // create LED_OFF task
    OSTaskCreate(task_led_off, (void *)0, &Task3Stk[TASK_STK_LEN - 1], 7);
    OSTaskNameSet(7, (INT8U *)"LED_OFF", (INT8U *)"LED_OFF_ERR");

	// create peripheral init task
	OSTaskCreate(task_peripheral_init, (void *)0, &Task4Stk[TASK_STK_LEN - 1], 8);
	OSTaskNameSet(8, (INT8U *)"PERIPHERAL_INIT", (INT8U *)"PERIPHERAL_INIT_ERR");
	OSTimeDly(3000);

	// create MPU6050 task
	// OSTaskCreate(task_MPU6050, (void *)0, &Task5Stk[TASK_STK_LEN - 1], 9);
	// OSTaskNameSet(9, (INT8U *)"MPU6050", (INT8U *)"MPU6050_ERR");
	OSTimeDly(5000);

	// create attitude control task
	OSTaskCreate(task_attitude_gyro, (void *)0, &Task6Stk[TASK_STK_LEN - 1], 10);
	OSTaskNameSet(10, (INT8U *)"attitude", (INT8U *)"attitude_ERR");
	// OSTaskCreate(task_attitude_acc, (void *)0, &Task7Stk[TASK_STK_LEN - 1], 11);
	// OSTaskNameSet(11, (INT8U *)"attitude", (INT8U *)"attitude_ERR");
    OSTaskDel(OS_PRIO_SELF);
}

int main(void)
{
	
	OSInit();
	// OS_TRACE_INIT(); //	SEGGER_SYSVIEW_Conf();
	// OS_TRACE_START(); // SEGGER_SYSVIEW_Start();
	OSTaskCreate(first_task, (void *)0, &MainTaskStk[TASK_STK_LEN-1], 2);
	OSTaskNameSet(2, (INT8U *)"FIRST_TASK", (INT8U *)"FIRST_TASK_ERR");
	OSStart();
	return 0;
}
