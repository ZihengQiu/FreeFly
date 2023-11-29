#include "os_cpu.h"
#include "stm32f4xx.h"                  // Device header
#include "includes.h"
#include "ucos_ii.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/_stdint.h>

#if (defined(OS_TRACE_EN) && (OS_TRACE_EN > 0u))
#include "SEGGER_SYSVIEW.h"                                   
#endif

#include "attitude.h"
#include "bluetooth.h"
#include "control.h"
#include "ground_control.h"
#include "gy86.h"
#include "led.h"
#include "myI2C.h"
#include "mathkit.h"
#include "motor.h"
#include "receiver.h"

#define TASK_PRIO_PID		9
#define TASK_PRIO_ATTITUDE	10
#define TASK_PRIO_GC		11

#define TASK_STK_LEN 0x0800
#define TASK_STK_LEN_2 0x0600

OS_STK MainTaskStk[TASK_STK_LEN];	// each stack is 4B wide
OS_STK Task1Stk[TASK_STK_LEN];
OS_STK Task2Stk[TASK_STK_LEN];
OS_STK Task3Stk[TASK_STK_LEN];
OS_STK Task4Stk[TASK_STK_LEN];
OS_STK Task5Stk[TASK_STK_LEN_2];
OS_STK Task6Stk[TASK_STK_LEN];
OS_STK Task7Stk[TASK_STK_LEN];
OS_STK Task8Stk[TASK_STK_LEN];
OS_STK Task9Stk[TASK_STK_LEN];
OS_STK Task10Stk[TASK_STK_LEN];

uint16_t times;
extern vec3d_t acc_offset, acc_scale, gyro_offset, gyro_filter[2], mag_offset, mag_scale;
extern vec3d_t acc, mag, gyro, euler;
extern BOOLEAN motor_armed;
extern uint32_t ppm_val[10];

void task_led_on(void *pdata)
{
	while(1)
	{
		led_on();
		OSTimeDly(2000);
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
	printf("Bluetooth init finished!\r\n");
	times++;

	MyI2C_Init();
	printf("I2C init finished!\r\n");
	times++;
	
	GY86Init();
	printf("GY86 init finished!\r\n");
	times++;
	
	Receiver_Init();
	printf("Receiver init finished!\r\n");
	times++;

	Motor_Init();
	printf("Motor init finished!\r\n");
	times++;

	printf("Initilization finished!\r\n");

	OSTaskDel(OS_PRIO_SELF);
}

void task_MPU6050(void *pdata)
{
	// AccCalibration(&acc_offset, &acc_scale);
	// GyroCalibration(&gyro_offset, &gyro_filter[2]);
	// MagCalibration(&mag_offset, &mag_scale);
	while(1)
	{
		vec3d_t acc;
		GetAccData(&acc);
		printf("acc: %f, %f, %f\n", acc.x, acc.y, acc.z);

		vec3d_t gyro;
		GetGyroData(&gyro);
		printf("gyro: %f, %f, %f\n", gyro.x, gyro.y, gyro.z);

		vec3d_t mag;
		GetMagData(&mag);
		printf("%f,%f,%f\n", mag.x, mag.y, mag.z);
		
		OSTimeDly(100);
	}
}

void task_attitude_gyro(void *pdata)
{
	// GyroCalibration(&gyro_offset, &gyro_filter[2]);
	vec4d_t q0 = {1, 0, 0, 0}, q1;
	vec3d_t gyro0 = {0, 0, 0}, gyro1, acc, mag, euler = {0, 0, 0};
	while(1)
	{
		GetAccData(&acc);
		GetMagData(&mag);
		GetGyroData(&gyro1);

		GyroUpdateQuat(&q0, &q1, &gyro0, &gyro1, 0.001);
		q0 = q1;
		gyro0 = gyro1;
		QuaterToEuler(&q0, &euler);
		RadToDeg(&euler);
	}
}

void task_attitude_acc(void *pdata)
{
	vec3d_t acc = {0, 0, 0}, gyro, mag, euler = {0, 0, 0};
	vec4d_t q0 = {1, 0, 0, 0}, q1;
	while(1)
	{
		GetAccData(&acc);
		GetGyroData(&gyro);
		GetMagData(&mag);

		AccUpdateQuat(&q0, &q1, &acc, &gyro, 0.001);
		q0 = q1;
		QuaterToEuler(&q0, &euler);
		RadToDeg(&euler);
	}
}

void task_attitude_acc_mag(void *pdata)
{
	// AccCalibration(&acc_offset, &acc_scale);
	// GyroCalibration(&gyro_offset, &gyro_filter[2]);
	vec3d_t acc_data = {0, 0, 0}, mag_data, gyro_data, euler = {0, 0, 0};
	vec4d_t q0 = {1, 0, 0, 0}, q1;
	while(1)
	{
		GetAccData(&acc_data);
		Vec3Norm(&acc_data);
		GetMagData(&mag_data);
		Vec3Norm(&mag_data);
		GetGyroData(&gyro_data);
		AccMagUpdateQuat(&q0, &q1, &acc_data, &gyro_data, &mag_data, 0.001);
		q0 = q1;
		QuaterToEuler(&q0, &euler);
		OSTimeDly(10);
	}
}

void task_attitude_fusion(void *pdata)
{
	vec4d_t q0 = {1, 0, 0, 0};
	while(1)
	{
		GetGyroData(&gyro); // reading data takes about 0.6ms
		GetAccData(&acc);
		GetMagData(&mag);

		MadgwickAHRS(&q0, acc, gyro, mag);	// takes about 0.1ms
		
		QuaterToEuler(&q0, &euler);
		RadToDeg(&euler);

		OSTimeDly(3);
	}
}

void task_send_ground_control(void *pdata)
{
	while(1)
	{
		if(signal_blocked == 0)
		{
			SendAnotc();
			OSTimeDly(5);
		}
		BTCommandParser();
			OSTimeDly(5);
	}
}

void task_motor_control(void *pdata)
{
	
	while(1)
	{
		OSTimeDly(3);

		if(signal_blocked == 1 || motor_armed == 0)
		{
			motor_compare[0] = MOTOR_COMPARE_MIN_VAL;
			motor_compare[1] = MOTOR_COMPARE_MIN_VAL;
			motor_compare[2] = MOTOR_COMPARE_MIN_VAL;
			motor_compare[3] = MOTOR_COMPARE_MIN_VAL;
			MotorSetSpeed();
		}
		if(((~signal_blocked) & motor_armed) == 1)
		{
			MotorControl(euler, gyro);
		}
		if(ppm_error == 1)
		{
			Bluetooth_SendString("PPM signal error!\r\n");
		}
	}
}

void first_task(void *pdata) {
    // Initialization

    // My_Systick_Config(840000); // AHB = 84MHz
	OS_CPU_SysTickInitFreq(84000000);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
    led_init();

    // // create LED_ON task
    // OSTaskCreateExt(task_led_on, (void *)0, &Task2Stk[TASK_STK_LEN - 1], 6, 6, Task2Stk, TASK_STK_LEN, (void *)0, 0);
    // OSTaskNameSet(6, (INT8U *)"LED_ON", (INT8U *)"LED_ON_ERR");

    // // create LED_OFF task
    // OSTaskCreateExt(task_led_off, (void *)0, &Task3Stk[TASK_STK_LEN - 1], 7, 7, Task3Stk, TASK_STK_LEN, (void *)0, 0);
    // OSTaskNameSet(7, (INT8U *)"LED_OFF", (INT8U *)"LED_OFF_ERR");

	// create peripheral init task
	OSTaskCreateExt(task_peripheral_init, (void *)0, &Task4Stk[TASK_STK_LEN - 1], 8, 8, Task4Stk, TASK_STK_LEN, (void *)0, 0);
	OSTaskNameSet(8, (INT8U *)"PERIPHERAL_INIT", (INT8U *)"PERIPHERAL_INIT_ERR");
	// OSTimeDly(3000);

	// create MPU6050 task
	// OSTaskCreateExt(task_MPU6050, (void *)0, &Task5Stk[TASK_STK_LEN_2 - 1], 9, 9, Task5Stk, TASK_STK_LEN_2, (void *)0, 0);
	// OSTaskNameSet(9, (INT8U *)"MPU6050", (INT8U *)"MPU6050_ERR");
	// OSTimeDly(3000);

	// create attitude control task
	// OSTaskCreateExt(task_attitude_gyro, (void *)0, &Task6Stk[TASK_STK_LEN - 1], 10, 10, Task6Stk, TASK_STK_LEN, (void *)0, 0);
	// OSTaskNameSet(10, (INT8U *)"attitude", (INT8U *)"attitude_ERR");
	// OSTaskCreateExt(task_attitude_acc, (void *)0, &Task7Stk[TASK_STK_LEN - 1], 11, 11, Task7Stk, TASK_STK_LEN, (void *)0, 0);
	// OSTaskNameSet(11, (INT8U *)"attitude", (INT8U *)"attitude_ERR");
	// OSTaskCreateExt(task_attitude_acc_mag, (void *)0, &Task7Stk[TASK_STK_LEN - 1], 11, 11, Task7Stk, TASK_STK_LEN, (void *)0, 0);
	// OSTaskNameSet(11, (INT8U *)"attitude", (INT8U *)"attitude_ERR");

	OSTaskCreateExt(task_attitude_fusion, (void *)0, &Task8Stk[TASK_STK_LEN - 1], TASK_PRIO_ATTITUDE, TASK_PRIO_ATTITUDE, Task8Stk, TASK_STK_LEN, (void *)0, 0);
	OSTaskNameSet(12, (INT8U *)"attitude", (INT8U *)"attitude_ERR");

	OSTaskCreateExt(task_motor_control, (void *)0, &Task9Stk[TASK_STK_LEN - 1], TASK_PRIO_PID, TASK_PRIO_PID, Task9Stk, TASK_STK_LEN, (void *)0, 0);
	OSTaskNameSet(13, (INT8U *)"motor_control", (INT8U *)"motor_control_ERR");

	OSTaskCreateExt(task_send_ground_control, (void *)0, &Task10Stk[TASK_STK_LEN - 1], TASK_PRIO_GC, TASK_PRIO_GC, Task10Stk, TASK_STK_LEN, (void *)0, 0);
	OSTaskNameSet(14, (INT8U *)"GC", (INT8U *)"GC");
	
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
