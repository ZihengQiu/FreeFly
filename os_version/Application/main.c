#include "os_cpu.h"
#include "stm32f4xx.h"                  // Device header
#include "includes.h"
#include "ucos_ii.h"

#include <math.h>
#include <stdint.h>
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
#include "esp-01s.h"

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
RCC_ClocksTypeDef clockwatch;
extern vec3d_t acc_offset, acc_scale, gyro_offset, gyro_filter[2], mag_offset, mag_scale;
extern vec3d_t acc, mag, gyro, euler;
extern BOOLEAN motor_armed;
extern uint32_t ppm_val[10];

extern void My_Systick_Config(uint32_t reload_value);


void SendAnotc(vec3d_t acc, vec3d_t gyro, vec3d_t mag, vec3d_t angle)
{
	// send attitude data to anotc v2.6
	uint8_t sum;
	sum = 0;
	int8_t data[32] = {0x00};
	data[0] = 0x88;
	data[1] = 0xAF;
	data[2] = 0x1C;
	data[3] = (int16_t)(acc.x*100)>>8;
	data[4] = (int16_t)(acc.x*100)&0xFF;
	data[5] = (int16_t)(acc.y*100)>>8;
	data[6] = (int16_t)(acc.y*100)&0xFF;
	data[7] = (int16_t)(acc.z*100)>>8;
	data[8] = (int16_t)(acc.z*100)&0xFF;
	data[9] = (int16_t)(gyro.x*100)>>8;
	data[10] = (int16_t)(gyro.x*100)&0xFF;
	data[11] = (int16_t)(gyro.y*100)>>8;
	data[12] = (int16_t)(gyro.y*100)&0xFF;
	data[13] = (int16_t)(gyro.z*100)>>8;
	data[14] = (int16_t)(gyro.z*100)&0xFF;
	data[15] = (int16_t)(mag.x*100)>>8;
	data[16] = (int16_t)(mag.x*100)&0xFF;
	data[17] = (int16_t)(mag.y*100)>>8;
	data[18] = (int16_t)(mag.y*100)&0xFF;
	data[19] = (int16_t)(mag.z*100)>>8;
	data[20] = (int16_t)(mag.z*100)&0xFF;
	data[21] = (int16_t)(angle.x*100)>>8;
	data[22] = (int16_t)(angle.x*100)&0xFF;
	data[23] = (int16_t)(angle.y*100)>>8;
	data[24] = (int16_t)(angle.y*100)&0xFF;
	data[25] = (int16_t)(angle.z*10)>>8;
	data[26] = (int16_t)(angle.z*10)&0xFF;
	data[27] = 0x00;
	data[28] = 0x00;
	data[29] = 0x00;
	data[30] = 0x00;

	for(int i=0; i<31; i++)
	{
		sum += data[i];
	}
	
	data[31] = sum;
	for(int i=0; i<32; i++)
	{
		Bluetooth_SendByte(data[i]);
	}
}

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
	// RCC_GetClocksFreq(&clockwatch);

	BluetoothInit();
	Bluetooth_SendString("Bluetooth init finished!\r\n");
	times++;

	ESP_Init();
	Bluetooth_SendString("ESP init finished!\r\n");
	times++;

	MyI2C_Init();
	Bluetooth_SendString("I2C init finished!\r\n");
	times++;
	
	GY86Init();
	Bluetooth_SendString("GY86 init finished!\r\n");
	times++;
	
	Receiver_Init();
	// Bluetooth_SendString("Receiver init finished!\r\n");
	times++;

	Motor_Init();
	Bluetooth_SendString("Motor init finished!\r\n");
	times++;

	Bluetooth_SendString("Initilization finished!\r\n");
	Usart2_SendString("init\r\n");

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
		// GetAccData(&acc);
		// printf("acc: %f, %f, %f\n", acc.x, acc.y, acc.z);

		vec3d_t gyro;
		// GetGyroData(&gyro);
		// printf("gyro: %f, %f, %f\n", gyro.x, gyro.y, gyro.z);

		vec3d_t mag;
		GetMagData(&mag);
		float modulus = Vec3Modulus(mag);
		// printf("%f,%f,%f\n", mag.x, mag.y, mag.z);
		printf("mag: %f, %f, %f, M:%f\n", mag.x, mag.y, mag.z, modulus);
		
		OSTimeDly(100);
	}
}

void task_attitude_fusion(void *pdata)
{
	vec4d_t q0 = {1, 0, 0, 0}, q1;

	// estimate the attitude of the first frame by acc and mag
	for(uint8_t i=0; i<50; i++)	// takes 3s 
	{
		GetAccData(&acc);
		GetGyroData(&gyro); 
		GetMagData(&mag);
		AccMagUpdateQuat(&q0, &q1, &acc, &gyro, &mag, 0.001);
		q0 = q1;
		QuaterToEuler(&q0, &euler);
		RadToDeg(&euler);
		// SendAnotc(acc, gyro, mag, euler);
	}
	volatile uint32_t t[10];
	while(1)
	{
		GetGyroData(&gyro); // reading data takes about 0.6ms
		GetAccData(&acc);
		GetMagData(&mag);

		MadgwickAHRS(&q0, acc, gyro, mag);	// takes about 0.1ms
		
		QuaterToEuler(&q0, &euler);
		RadToDeg(&euler);
		
		// SendAnotc(acc, gyro, mag, euler);
		char str[100];
		sprintf(str, "%10f, %10f, %10f\r\n", euler.x, euler.y, euler.z);
		Bluetooth_SendString(str); // takes about 3ms
		// Usart2_SendString(str);
		// printf("euler: %10f, %10f, %10f\r\n", euler.x, euler.y, euler.z);
	}
}

void task_motor_control(void *pdata)
{
	while(1)
	{
		char str[100];
		// sprintf(str, "motor_armed%d\r\n", motor_armed);
		// Bluetooth_SendString(str);
		OSTimeDly(100);
		sprintf(str, "%d %d %d %d %d %d %d %d %d\r\n", ppm_val[0], ppm_val[1], ppm_val[2], ppm_val[3], ppm_val[4], ppm_val[5], ppm_val[6], ppm_val[7], ppm_val[8]);
		Bluetooth_SendString(str);
	}
}

void task_esp_test(void *pdata)
{
	while(1)
	{
	Bluetooth_SendString("now test begins!\r\n");
	OSTimeDly(1000);
	Usart2_SendString("AT\r\n");
	// wifi_connect();
	}

}

void first_task(void *pdata) {
    // Initialization
    // My_Systick_Config(840000); // AHB = 84MHz
	OS_CPU_SysTickInitFreq(84000000);
	
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
	OSTimeDly(3000);

	// create attitude control task
	// OSTaskCreateExt(task_attitude_gyro, (void *)0, &Task6Stk[TASK_STK_LEN - 1], 10, 10, Task6Stk, TASK_STK_LEN, (void *)0, 0);
	// OSTaskNameSet(10, (INT8U *)"attitude", (INT8U *)"attitude_ERR");
	// OSTaskCreateExt(task_attitude_acc_mag, (void *)0, &Task7Stk[TASK_STK_LEN - 1], 11, 11, Task7Stk, TASK_STK_LEN, (void *)0, 0);
	// OSTaskNameSet(11, (INT8U *)"attitude", (INT8U *)"attitude_ERR");
	// OSTaskCreateExt(task_attitude_fusion, (void *)0, &Task8Stk[TASK_STK_LEN - 1], 12, 12, Task8Stk, TASK_STK_LEN, (void *)0, 0);
	// OSTaskNameSet(12, (INT8U *)"attitude", (INT8U *)"attitude_ERR");

	//extra function 
	// OSTaskCreateExt(task_motor_control, (void *)0, &Task9Stk[TASK_STK_LEN - 1], 13, 13, Task8Stk, TASK_STK_LEN, (void *)0, 0);
	// OSTaskNameSet(13, (INT8U *)"motor_control", (INT8U *)"motor_control_ERR");
	OSTaskCreateExt(task_esp_test, (void *)0, &Task10Stk[TASK_STK_LEN - 1], 10, 10, Task10Stk, TASK_STK_LEN, (void *)0, 0);
	OSTaskNameSet(13, (INT8U *)"esp_test", (INT8U *)"esp_test_ERR");
	
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
