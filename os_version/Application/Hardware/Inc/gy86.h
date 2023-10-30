#ifndef __GY86_H
#define __GY86_H

#include "stm32f4xx.h"                   // Device header

#include "mathkit.h"
#include "bluetooth.h"
#include "myI2C.h"

#include "ucos_ii.h"

// MPU6050
#define AddressMPU6050				(uint8_t)0xD0

#define SMPRT_DIV              		(uint8_t)0x19
#define CONFIG              		(uint8_t)0x1A
#define GYRO_CONFIG              	(uint8_t)0x1B
#define ACCEL_CONFIG				(uint8_t)0x1C
#define INT_PIN_CFG					(uint8_t)0x37
#define USER_CTRL					(uint8_t)0x6A
#define PWR_MGMT_1              	(uint8_t)0x6B

#define ACCEL_X_OUT_H            	(uint8_t)0x3B
#define ACCEL_X_OUT_L            	(uint8_t)0x3C
#define ACCEL_Y_OUT_H            	(uint8_t)0x3D
#define ACCEL_Y_OUT_L            	(uint8_t)0x3E
#define ACCEL_Z_OUT_H            	(uint8_t)0x3F
#define ACCEL_Z_OUT_L            	(uint8_t)0x40

#define TEMP_OUT_H					(uint8_t)0x41
#define TEMP_OUT_L					(uint8_t)0x42

#define GYRO_X_OUT_H					(uint8_t)0x43
#define GYRO_X_OUT_L					(uint8_t)0x44
#define GYRO_Y_OUT_H					(uint8_t)0x45
#define GYRO_Y_OUT_L					(uint8_t)0x46
#define GYRO_Z_OUT_H					(uint8_t)0x47
#define GYRO_Z_OUT_L					(uint8_t)0x48

// HMC5883
#define AddressHMC5883				(uint8_t)0x3C

#define ConfigA						(uint8_t)0x00
#define ConfigB						(uint8_t)0x01
#define ModeRegister				(uint8_t)0x02
#define MAG_X_OUT_H					(uint8_t)0x03
#define MAG_X_OUT_L					(uint8_t)0x04
#define MAG_Y_OUT_H					(uint8_t)0x05
#define MAG_Y_OUT_L					(uint8_t)0x06
#define MAG_Z_OUT_H					(uint8_t)0x07
#define MAG_Z_OUT_L					(uint8_t)0x08
#define StatusRegister				(uint8_t)0x09

// MS5611
#define AddressMS5611				(uint8_t)0x111011C0

#define WHO_AM_I					(uint8_t)0x75

typedef struct
{
  float acc_x, acc_y, acc_z;
  float gyro_x, gyro_y, gyro_z;
  float temp;
  float hmc5883_x, hmc5883_y, hmc5883_z;
}MpuDataStruct;

extern vec3d_t acc_offset, acc_scale;
extern vec3d_t gyro_offset, gyro_filter[2];
extern vec3d_t mag_offset, mag_scale;

void GY86Init(void);

void MPU6050Init(void);
void GetMpuData(MpuDataStruct *mpuData);

void GetAccData(vec3d_t *acc);
void AccCalibration(vec3d_t *offset, vec3d_t *scale);

void GetGyroData(vec3d_t *gyro);
void GyroCalibration(vec3d_t *offset, vec3d_t *gyro_filter[2]);

void HMC5883Init(void);
void GetMagData(vec3d_t *mag);
void MagCalibration(vec3d_t *offset, vec3d_t *scale);

void MS5611Init(void);

#endif
