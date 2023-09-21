#include "GY86.h"

Vec3d_t acc_offset, acc_scale;

void MPU6050Init(void)
{
	MyI2C_WriteRegister(AddressMPU6050, PWR_MGMT_1, 0x01);		//clock source: PLL with X axis gyroscope reference (10KHz with DLPF on and 1 KHz with DLPF off)
	MyI2C_WriteRegister(AddressMPU6050, SMPRT_DIV, 0x09);		//SMPLRT_DIV:10 Sample Rate = 1KHz/(1+9) = 100Hz
	MyI2C_WriteRegister(AddressMPU6050, CONFIG, 0x06);          //DLPF_CFG = 3
	MyI2C_WriteRegister(AddressMPU6050, GYRO_CONFIG, 0x18);     //FS_SEL = 3, Full Scale Range of gyroscope = +- 2000 degree/s
	MyI2C_WriteRegister(AddressMPU6050, ACCEL_CONFIG, 0x18);    //AFS_SEL = 3, Full scale range of accelerometer = +- 16g
	
//	MyI2C_WriteRegister(AddressMPU6050, INT_PIN_CFG, 0x02);		//I2C_bypass_en
//	MyI2C_WriteRegister(AddressMPU6050, USER_CTRL, 0x01);		// reset
}

void GetMpuData(MpuDataStruct *data)
{
	// read separately
	// uint8_t tempH, tempL;
	// tempH = MyI2C_ReadRegister_1Bytes(AddressMPU6050, ACCEL_XOUT_H);
	// tempL = MyI2C_ReadRegister_1Bytes(AddressMPU6050, ACCEL_XOUT_L);
	// data->acc_x = (tempH<<8) + tempL;

	// burst read
	uint8_t Buffer[20];
	float acc_full = 16.0, gyro_full = 2000.0;
	MyI2C_BurstReadRegister(AddressMPU6050, ACCEL_XOUT_H, Buffer, 14);
	data->acc_x = ((double)(int16_t)((Buffer[0]<<8) + Buffer[1]))*acc_full/32768.0;
	data->acc_y = (double)(int16_t)((Buffer[2]<<8) + Buffer[3])*acc_full/32768.0;
	data->acc_z = (double)(int16_t)((Buffer[4]<<8) + Buffer[5])*acc_full/32768.0;
	data->temp = (double)(int16_t)((Buffer[6]<<8) + Buffer[7]) / 340.0 + 36.53;
	data->gyro_x = (double)(int16_t)((Buffer[8]<<8) + Buffer[9])*gyro_full/32768.0;
	data->gyro_y = (double)(int16_t)((Buffer[10]<<8) + Buffer[11])*gyro_full/32768.0;
	data->gyro_z = (double)(int16_t)((Buffer[12]<<8) + Buffer[13])*gyro_full/32768.0;
	
}

void GetAccMPU6050(Vec3d_t *acc)
{
	uint8_t Buffer[6];
	float acc_full = 16.0;
	MyI2C_BurstReadRegister(AddressMPU6050, ACCEL_XOUT_H, Buffer, 6);
	acc->x = (double)(int16_t)((Buffer[0]<<8) + Buffer[1])*acc_full/32768.0;
	acc->y = (double)(int16_t)((Buffer[2]<<8) + Buffer[3])*acc_full/32768.0;
	acc->z = (double)(int16_t)((Buffer[4]<<8) + Buffer[5])*acc_full/32768.0;
}

uint8_t AccCalibration(Vec3d_t *offset, Vec3d_t *scale)
{
	Bluetooth_SendString("Acc Calibration start.\n");
	Bluetooth_SendString("Please put the sensor on a flat surface.\n");
	Delay_ms(3000);

	Vec3d_t acc_data[6];
	for(uint8_t i=0; i<6; i++)
	{
		acc_data[i].x = 0;
		acc_data[i].y = 0;
		acc_data[i].z = 0;
	}
	char direction[6][10] = {"front", "back", "left", "right", "up", "down"};
	
	for(uint8_t i=0; i<6; i++)
	{
		Bluetooth_SendString(direction[i]);
		Delay_ms(2000);
		Bluetooth_SendString("measuring...");
		for(uint8_t j=0; j<100; j++)
		{
			GetAccMPU6050(&acc_data[i]);
			acc_data[i].x += acc_data[i].x;
			acc_data[i].y += acc_data[i].y;
			acc_data[i].z += acc_data[i].z;
			Delay_ms(10);
		}
		acc_data[i].x /= 100.0;
		acc_data[i].y /= 100.0;
		acc_data[i].z /= 100.0;
		Bluetooth_SendString("done.\n");
	}

	GaussNewton(acc_data, offset, scale);
}



uint16_t GetGYROXMPU6050(void)
{
	return MyI2C_ReadRegister_2Bytes(AddressMPU6050, GYRO_XOUT_H); 
}

uint16_t GetGYROYMPU6050(void)
{
	return MyI2C_ReadRegister_2Bytes(AddressMPU6050, GYRO_YOUT_H); 
}

uint16_t GetGYROZMPU6050(void)
{
	return MyI2C_ReadRegister_2Bytes(AddressMPU6050, GYRO_ZOUT_H); 
}

void HMC5883Init(void)
{
	MyI2C_WriteRegisterHMC5883(AddressHMC5883, ConfigA, 0x70);			//Outout rate : 15Hz
	MyI2C_WriteRegisterHMC5883(AddressHMC5883, ConfigB, 0xA0);		
	MyI2C_WriteRegisterHMC5883(AddressHMC5883, ModeRegister, 0x00);	//Continuous-Measurement Mode
	// StatusRegister = 0x01 when data prepared
}

uint16_t GetXHMC5883(void)
{
	return MyI2C_ReadRegister_1Bytes(AddressHMC5883, OutputXMSB)<<8 + MyI2C_ReadRegister_1Bytes(AddressHMC5883, OutputXLSB);
}
uint16_t GetYHMC5883(void)
{
	return MyI2C_ReadRegister_1Bytes(AddressHMC5883, OutputYMSB)<<8 + MyI2C_ReadRegister_1Bytes(AddressHMC5883, OutputYLSB);
}
uint16_t GetZHMC5883(void)
{
	return MyI2C_ReadRegister_1Bytes(AddressHMC5883, OutputZMSB)<<8 + MyI2C_ReadRegister_1Bytes(AddressHMC5883, OutputZLSB);
}

void MS5611Init(void)
{
	
}
