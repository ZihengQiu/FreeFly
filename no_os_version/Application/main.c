#include "stm32f4xx.h"                  // Device header
#include "MyDelay.h"
#include "MyI2C.h"
#include "GY86.h"
#include "Bluetooth.h"
#include "Motor.h"
#include "Receiver.h"
#include "math.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

MpuDataStruct MpuData;
extern Vec3d_t acc_offset, acc_scale;
extern uint32_t PulseWidth, Period, DutyCycle;
extern char TransmitData[1005];
uint16_t times;
uint8_t RxData;

RCC_ClocksTypeDef clockwatch;

int main(void)
{
//	RCC_GetClocksFreq(&clockwatch);
	Delay_ms(1000);
	
	MyI2C_Init();
	times++;
	MPU6050Init();
	times++;
	// HMC5883Init();
	times++;
	
	BluetoothInit();
	
	TIM1_init();
	Motor_Init();
	
	Delay_ms(1000);

//	TIM_SetCompare1(TIM3, 2000);
//	Delay_ms(4000);
//	TIM_SetCompare1(TIM3, 1000);
//	Delay_ms(4000);                                                
	
	Bluetooth_SendString("Initilization finished!\n");

	
	float a = 0.12345;
	printf("%f\n", a);

	

	Vec3d_t input[6] = { {-0.019556, 0.012036, 1.685950}, {1.018469, 0.069458, 0.720422},
							{-0.981909, 0.001819, 0.450183}, {0.063464, -0.957397, 0.515881},
							{-0.021716, 1.004285, 0.713989}, {0.032495, -0.022266, -0.387183}};
	Vec3d_t offset, scale;

	GaussNewton_LM(input, &offset, &scale);

	// update input
	for(uint8_t i=0; i<6; i++) 
	{
		input[i].x = (input[i].x - offset.x) * scale.x;
		input[i].y = (input[i].y - offset.y) * scale.y;
		input[i].z = (input[i].z - offset.z) * scale.z;
	}

	AccCalibration(&acc_offset, &acc_scale);


	Vec3d_t test;
	while(1)
	{
		// motor and receiver
		// Delay_ms(50);
		// Motor_SetDutyCycle(DutyCycle);
		
		// Bluetooth
		
		
		//MPU6050
		// uint8_t ID = MyI2C_ReadRegister_1Bytes(AddressMPU6050, WHO_AM_I);
		// sprintf(TransmitData, "ID : %d\n", ID);
		// Bluetooth_SendString(TransmitData);
		// Delay_ms(10);


		GetAccMPU6050(&test);

		test.x = (test.x - acc_offset.x) * acc_scale.x;
		test.y = (test.y - acc_offset.y) * acc_scale.y;
		test.z = (test.z - acc_offset.z) * acc_scale.z;

		uint8_t a = 1;

		// Bluetooth_SendString("ACC Data:\n");
		// Bluetooth_SendString("mpu_acc_x : ");
		// Bluetooth_SendSignedNum(MpuData.acc_x);
		// Bluetooth_SendByte('\n');
		// // Delay_ms(10);
		// Bluetooth_SendString("mpu_acc_y : ");
		// Bluetooth_SendSignedNum(MpuData.acc_y);
		// Bluetooth_SendByte('\n');
		// // Delay_ms(10);
		// Bluetooth_SendString("mpu_acc_z : ");
		// Bluetooth_SendSignedNum(MpuData.acc_z);
		// Bluetooth_SendByte('\n');
		// // Delay_ms(10);

//		Bluetooth_SendString("Temp Data:\n");
//		Bluetooth_SendSignedNum(MpuData.temp);
//		Bluetooth_SendByte('\n');
//		// Delay_ms(10);

//		Bluetooth_SendString("Gyro Data:\n");
//		Bluetooth_SendString("mpu_gyro_x : ");
//		Bluetooth_SendSignedNum(MpuData.gyro_x);
//		Bluetooth_SendByte('\n');
//		// Delay_ms(10);
//		Bluetooth_SendString("mpu_gyro_y : ");
//		Bluetooth_SendSignedNum(MpuData.gyro_y);
//		Bluetooth_SendByte('\n');
//		// Delay_ms(10);
//		Bluetooth_SendString("mpu_gyro_z : ");
//		Bluetooth_SendSignedNum(MpuData.gyro_z);
//		Bluetooth_SendByte('\n');
//		// Delay_ms(10);

		
//		Bluetooth_SendString("DutyCycle : ");
//		sprintf(TransmitData, "%d", DutyCycle);
//		Bluetooth_SendString(TransmitData);
//		Bluetooth_SendByte('\n');
//		Delay_ms(10);
	} 
}
