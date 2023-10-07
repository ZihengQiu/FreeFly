#include "gy86.h"

Vec3d_t acc_offset = {0, 0, 0}, acc_scale = {1, 1, 1};
Vec3d_t gyro_offset = {0, 0, 0};
Vec3d_t mag_offset = {0, 0, 0}, mag_scale = {1, 1, 1};

void GY86Init(void)
{
	MPU6050Init();
	HMC5883Init();
}

void MPU6050Init(void)
{
	MyI2C_WriteRegister(AddressMPU6050, PWR_MGMT_1, 0x01);		// clock source: PLL with X axis gyroscope reference (10KHz with DLPF on and 1 KHz with DLPF off)
	MyI2C_WriteRegister(AddressMPU6050, SMPRT_DIV, 0x09);		// SMPLRT_DIV:10 Sample Rate = 1KHz/(1+9) = 100Hz
	MyI2C_WriteRegister(AddressMPU6050, CONFIG, 0x06);          // DLPF_CFG = 3
	MyI2C_WriteRegister(AddressMPU6050, GYRO_CONFIG, 0x18);     // FS_SEL = 3, Full Scale Range of gyroscope = +- 2000 degree/s
	MyI2C_WriteRegister(AddressMPU6050, ACCEL_CONFIG, 0x18);    // AFS_SEL = 3, Full scale range of accelerometer = +- 16g
	MyI2C_WriteRegister(AddressMPU6050, INT_PIN_CFG, 0x02);		// I2C_bypass_en
	MyI2C_WriteRegister(AddressMPU6050, USER_CTRL, 0x00);		// the auxiliary I2C bus lines are logically driven by the primary I2C bus (SDA and SCL).

}

void GetMpuData(MpuDataStruct *data)
{
	// read separately
	// uint8_t tempH, tempL;
	// tempH = MyI2C_ReadRegister_1Bytes(AddressMPU6050, ACCEL_X_OUT_H);
	// tempL = MyI2C_ReadRegister_1Bytes(AddressMPU6050, ACCEL_X_OUT_L);
	// data->acc_x = (tempH<<8) + tempL;

	// burst read
	uint8_t Buffer[20];
	float acc_full = 16.0, gyro_full = 2000.0;
	MyI2C_BurstReadRegister(AddressMPU6050, ACCEL_X_OUT_H, Buffer, 14);
	data->acc_x = ((double)(int16_t)((Buffer[0]<<8) + Buffer[1]))*acc_full/32768.0;
	data->acc_y = (double)(int16_t)((Buffer[2]<<8) + Buffer[3])*acc_full/32768.0;
	data->acc_z = (double)(int16_t)((Buffer[4]<<8) + Buffer[5])*acc_full/32768.0;
	data->temp = (double)(int16_t)((Buffer[6]<<8) + Buffer[7]) / 340.0 + 36.53;
	data->gyro_x = (double)(int16_t)((Buffer[8]<<8) + Buffer[9])*gyro_full/32768.0;
	data->gyro_y = (double)(int16_t)((Buffer[10]<<8) + Buffer[11])*gyro_full/32768.0;
	data->gyro_z = (double)(int16_t)((Buffer[12]<<8) + Buffer[13])*gyro_full/32768.0;
	
}

// Accelerometer
void GetAccData(Vec3d_t *acc)
{
	uint8_t Buffer[6];
	float acc_full = 16.0;
	MyI2C_BurstReadRegister(AddressMPU6050, ACCEL_X_OUT_H, Buffer, 6);
	acc->x = (double)(int16_t)((Buffer[0]<<8) + Buffer[1])*acc_full/32768.0;
	acc->y = (double)(int16_t)((Buffer[2]<<8) + Buffer[3])*acc_full/32768.0;
	acc->z = (double)(int16_t)((Buffer[4]<<8) + Buffer[5])*acc_full/32768.0;

	// acc calibration
	acc->x = (acc->x - acc_offset.x) * acc_scale.x;
	acc->y = (acc->y - acc_offset.y) * acc_scale.y;
	acc->z = (acc->z - acc_offset.z) * acc_scale.z;
}

void AccCalibration(Vec3d_t *offset, Vec3d_t *scale)
{
	Bluetooth_SendString("Acc Calibration start.\n");
	Bluetooth_SendString("Please put the sensor on a flat surface.\n");
	OSTimeDly(3000);

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
		OSTimeDly(3000);
		Bluetooth_SendString(" measuring...");
		Vec3d_t temp= {0, 0, 0};
		for(uint8_t j=0; j<100; j++)
		{
			GetAccData(&temp);
			acc_data[i].x += temp.x;
			acc_data[i].y += temp.y;
			acc_data[i].z += temp.z;
			OSTimeDly(10);
		}
		acc_data[i].x /= 100.0;
		acc_data[i].y /= 100.0;
		acc_data[i].z /= 100.0;
		Bluetooth_SendString("done.\n");
	}

	GaussNewton_LM(acc_data, offset, scale);
}

// Gyroscope
void GetGyroData(Vec3d_t *gyro)
{
	uint8_t Buffer[6];
	float gyro_full = 2000.0;
	MyI2C_BurstReadRegister(AddressMPU6050, GYRO_X_OUT_H, Buffer, 6);
	gyro->x = (double)(int16_t)((Buffer[0]<<8) + Buffer[1])*gyro_full/32768.0;
	gyro->y = (double)(int16_t)((Buffer[2]<<8) + Buffer[3])*gyro_full/32768.0;
	gyro->z = (double)(int16_t)((Buffer[4]<<8) + Buffer[5])*gyro_full/32768.0;

	// gyro calibration
	gyro->x -= gyro_offset.x;
	gyro->y -= gyro_offset.y;
	gyro->z -= gyro_offset.z;
}

void GyroCalibration(Vec3d_t *offset)
{
	Bluetooth_SendString("Gyro Calibration start.\n");
	Bluetooth_SendString("Please keep the sensor still.\n");
	OSTimeDly(3000);

	Vec3d_t gyro_data = {0, 0, 0}, temp;

	for(uint8_t i=0; i<100; i++)
	{
		GetGyroData(&temp);
		gyro_data.x += temp.x;
		gyro_data.y += temp.y;
		gyro_data.z += temp.z;
		OSTimeDly(10);
	}
	gyro_data.x /= 100.0;
	gyro_data.y /= 100.0;
	gyro_data.z /= 100.0;
	offset->x = gyro_data.x;
	offset->y = gyro_data.y;
	offset->z = gyro_data.z;
	Bluetooth_SendString("done.\n");
}

void HMC5883Init(void)
{
	MyI2C_WriteRegisterHMC5883(AddressHMC5883, ConfigA, 0x70); //Outout rate : 15Hz
	MyI2C_WriteRegisterHMC5883(AddressHMC5883, ConfigB, 0x20);		
	MyI2C_WriteRegisterHMC5883(AddressHMC5883, ModeRegister, 0x00);	//Continuous-Measurement Mode
	// StatusRegister = 0x01 when data prepared
}

void GetMagData(Vec3d_t *mag)
{
	uint8_t Buffer[6];
	float mag_full = 1.0, mag_gain = 1090;
	MyI2C_BurstReadRegister(AddressHMC5883, MAG_X_OUT_H, Buffer, 6);
	mag->x = (double)(int16_t)((Buffer[0]<<8) + Buffer[1])*mag_full/mag_gain;
	mag->y = (double)(int16_t)((Buffer[2]<<8) + Buffer[3])*mag_full/mag_gain;
	mag->z = (double)(int16_t)((Buffer[4]<<8) + Buffer[5])*mag_full/mag_gain;

	// mag calibration
	mag->x = (mag->x - mag_offset.x) * mag_scale.x;
	mag->y = (mag->y - mag_offset.y) * mag_scale.y;
	mag->z = (mag->z - mag_offset.z) * mag_scale.z;
}

void MagCalibration(Vec3d_t *offset, Vec3d_t *scale)
{
	Bluetooth_SendString("Mag Calibration start.\n");
	Bluetooth_SendString("Please rotate the sensor around all axes.\n");
	// OSTimeDly(3000);

	Vec3d_t mag_data[6];
	for(uint8_t i=0; i<6; i++)
	{
		mag_data[i].x = 0;
		mag_data[i].y = 0;
		mag_data[i].z = 0;
	}
	char direction[6][10] = {"front", "back", "left", "right", "up", "down"};
	
	for(uint8_t i=0; i<6; i++)
	{
		Bluetooth_SendString(direction[i]);
		OSTimeDly(500);
		Bluetooth_SendString(" measuring...");
		Vec3d_t temp= {0, 0, 0};
		for(uint8_t j=0; j<100; j++)
		{
			GetMagData(&temp);
			mag_data[i].x += temp.x;
			mag_data[i].y += temp.y;
			mag_data[i].z += temp.z;
			OSTimeDly(10);
		}
		mag_data[i].x /= 100.0;
		mag_data[i].y /= 100.0;
		mag_data[i].z /= 100.0;
		Bluetooth_SendString("done.\n");
	}

	GaussNewton_LM(mag_data, offset, scale);

	scale->x = 1.0/scale->x;
	scale->y = 1.0/scale->y;
	scale->z = 1.0/scale->z;
}

void MS5611Init(void)
{
	
}
