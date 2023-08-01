#ifndef __GY86_H
#define __GY86_H

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


void MPU6050Init(void);
uint16_t GetACCXMPU6050(void);
uint16_t GetACCYMPU6050(void);
uint16_t GetACCZMPU6050(void);

void HMC5883Init(void);
uint16_t GetXHMC5883(void);
uint16_t GetYHMC5883(void);
uint16_t GetZHMC5883(void);


void MS5611Init(void);


#endif
