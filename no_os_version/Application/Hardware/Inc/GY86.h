#ifndef __GY86_H
#define __GY86_H

#include "stm32f4xx.h"                   // Device header
#include "MyDelay.h"
#include "math.h"
#include "Bluetooth.h"
#include "MyI2C.h"

/* MPU6050 */
#define AddressMPU6050				(uint8_t)0xD0
// 1101 0000
#define SMPRT_DIV              		(uint8_t)0x19
#define CONFIG              		(uint8_t)0x1A
#define GYRO_CONFIG              	(uint8_t)0x1B
#define ACCEL_CONFIG				(uint8_t)0x1C
#define INT_PIN_CFG					(uint8_t)0x37
#define USER_CTRL					(uint8_t)0x6A
#define PWR_MGMT_1              	(uint8_t)0x6B

#define ACCEL_XOUT_H            	(uint8_t)0x3B
#define ACCEL_XOUT_L            	(uint8_t)0x3C
#define ACCEL_YOUT_H            	(uint8_t)0x3D
#define ACCEL_YOUT_L            	(uint8_t)0x3E
#define ACCEL_ZOUT_H            	(uint8_t)0x3F
#define ACCEL_ZOUT_L            	(uint8_t)0x40

#define TEMP_OUT_H					(uint8_t)0x41
#define TEMP_OUT_L					(uint8_t)0x42

#define GYRO_XOUT_H					(uint8_t)0x43
#define GYRO_XOUT_L					(uint8_t)0x44
#define GYRO_YOUT_H					(uint8_t)0x45
#define GYRO_YOUT_L					(uint8_t)0x46
#define GYRO_ZOUT_H					(uint8_t)0x47
#define GYRO_ZOUT_L					(uint8_t)0x48

/* HMC5883 */ 
#define AddressHMC5883				(uint8_t)0x3C
// 7bit:		0x1E:0001 1110   
// 8bit:   read:0x3D:0011 1101 	    write:0x3C:0011 1100  direction receiver = 1
#define AddressHMC5883LWrite		(uint8_t)0x3C
#define AddressHMC5883LRead			(uint8_t)0x3D

#define ConfigA						(uint8_t)0x00
#define ConfigB						(uint8_t)0x01
#define ModeRegister				(uint8_t)0x02
#define OutputXMSB					(uint8_t)0x03
#define OutputXLSB					(uint8_t)0x04
#define OutputYMSB					(uint8_t)0x05
#define OutputYLSB					(uint8_t)0x06
#define OutputZMSB					(uint8_t)0x07
#define OutputZLSB					(uint8_t)0x08
#define StatusRegister				(uint8_t)0x09

/* MS5611 */ 
#define AddressMS5611				(uint8_t)0x111011C0 //0x111011

#define WHO_AM_I					(uint8_t)0x75

typedef struct
{
  double acc_x, acc_y, acc_z;
  double gyro_x, gyro_y, gyro_z;
  double temp;
  double hmc5883_x, hmc5883_y, hmc5883_z;
}MpuDataStruct;

extern Vec3d_t acc_offset, acc_scale;

void MPU6050Init(void);

uint16_t GetACCXMPU6050(void);
uint16_t GetACCYMPU6050(void);
uint16_t GetACCZMPU6050(void);
uint8_t AccCalibration(Vec3d_t *offset, Vec3d_t *scale);

uint16_t GetGYROXMPU6050(void);
uint16_t GetGYROYMPU6050(void);
uint16_t GetGYROZMPU6050(void);

void HMC5883Init(void);
uint16_t GetXHMC5883(void);
uint16_t GetYHMC5883(void);
uint16_t GetZHMC5883(void);

void MS5611Init(void);

void GetMpuData(MpuDataStruct *mpuData);
void GetAccMPU6050(Vec3d_t *acc);



#endif
