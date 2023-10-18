#include "gy86.h"
#include "mathkit.h"
#include "ucos_ii.h"
#include <stdint.h>

#define MAX_DOUBLE 10000
#define ACC_CALIBRATED

// Vec3d_t acc_offset = {0, 0, 0}, acc_scale = {1, 1, 1};
// Vec3d_t gyro_offset = {0, 0, 0},
// 		gyro_filter[2] = {{MAX_DOUBLE, MAX_DOUBLE, MAX_DOUBLE}, {-1*MAX_DOUBLE, -1*MAX_DOUBLE, -1*MAX_DOUBLE}};
Vec3d_t acc_offset = {0.037417782323451894, -0.013271198993009256, 0.0010460136242150379},
		acc_scale = {0.99615152668035645, 0.99960933237994376, 1.0040191999055366};
uint8_t acc_calibrated = 1, gyro_calibrated = 1, mag_calibrated = 1;
Vec3d_t gyro_offset = {-2.86273193359375, -0.65972900390625, -1.403076171875}, 
		gyro_filter[2] = {{-0.00592041015625, -0.07269287109375, -0.061767578125},
						{0.05511474609375, 0.04937744140625, 0.060302734375}};
Vec3d_t mag_offset = {0.4479153454629381, -0.04967758735144858, 0.13242835683955598}, 
		mag_scale = {0.4675627581500121, 0.4639468446544417, 0.4310428650240345};
		
void GY86Init(void)
{
	MPU6050Init();
	HMC5883Init();
}

void MPU6050Init(void)
{
	MyI2C_WriteRegister(AddressMPU6050, PWR_MGMT_1, 0x01);		// clock source: PLL with X axis gyroscope reference (10KHz with DLPF on and 1 KHz with DLPF off)
	MyI2C_WriteRegister(AddressMPU6050, SMPRT_DIV, 0x09);		// SMPLRT_DIV:10 Sample Rate = 1KHz/(1+9) = 100Hz (acc output rate=1kHz)
	MyI2C_WriteRegister(AddressMPU6050, CONFIG, 0x06);          // DLPF_CFG = 6
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

	if(acc_calibrated == 0)	return;

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

	acc_calibrated = 1;
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

	// convert to rad/s
	// gyro->x *= PI/180.0;
	// gyro->y *= PI/180.0;
	// gyro->z *= PI/180.0;

	if(gyro_calibrated == 0)	return;

	// gyro calibration
	gyro->x -= gyro_offset.x;
	gyro->y -= gyro_offset.y;
	gyro->z -= gyro_offset.z;
	if(gyro->x >= gyro_filter[0].x && gyro->x <= gyro_filter[1].x)
	{
		gyro->x = 0;
	}
	if(gyro->y >= gyro_filter[0].y && gyro->y <= gyro_filter[1].y)
	{
		gyro->y = 0;
	}
	if(gyro->z >= gyro_filter[0].z && gyro->z <= gyro_filter[1].z)
	{
		gyro->z = 0;
	}
}

// TODO : optimize the way to pass parameter filter
void GyroCalibration(Vec3d_t *offset, Vec3d_t *filter[2])
{
	Bluetooth_SendString("Gyro Calibration start.\n");
	Bluetooth_SendString("Please keep the sensor still.\n");
	OSTimeDly(1000);

	// TODO : optimize the type of sample_cnt
	int sample_cnt = 1000;
	Vec3d_t data = {0, 0, 0}, data_min = {100, 100, 100}, data_max = {-100, -100, -100},
			temp;

	for(int i=0; i<sample_cnt; i++)
	{
		GetGyroData(&temp);
		data.x += temp.x;
		data.y += temp.y;
		data.z += temp.z;
		data_min.x = data_min.x > temp.x ? temp.x : data_min.x;
		data_min.y = data_min.y > temp.y ? temp.y : data_min.y;
		data_min.z = data_min.z > temp.z ? temp.z : data_min.z;
		data_max.x = data_max.x < temp.x ? temp.x : data_max.x;
		data_max.y = data_max.y < temp.y ? temp.y : data_max.y;
		data_max.z = data_max.z < temp.z ? temp.z : data_max.z;
		OSTimeDly(10);
	}
	data.x /= sample_cnt;
	data.y /= sample_cnt;
	data.z /= sample_cnt;
	offset->x = data.x;
	offset->y = data.y;
	offset->z = data.z;
	gyro_filter[0].x = data_min.x - offset->x;
	gyro_filter[0].y = data_min.y - offset->y;
	gyro_filter[0].z = data_min.z - offset->z;
	gyro_filter[1].x = data_max.x - offset->x;
	gyro_filter[1].y = data_max.y - offset->y;
	gyro_filter[1].z = data_max.z - offset->z;
	// filter[0]->x = data_min.x - offset->x;
	// filter[0]->y = data_min.y - offset->y;
	// filter[0]->z = data_min.z - offset->z;
	// filter[1]->x = data_max.x - offset->x;
	// filter[1]->y = data_max.y - offset->y;
	// filter[1]->z = data_max.z - offset->z;
	Bluetooth_SendString("done.\n");

	gyro_calibrated = 1;
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
	// sequence : x-z-y
	mag->x = (double)(int16_t)((Buffer[0]<<8) + Buffer[1])*mag_full/mag_gain;
	mag->z = (double)(int16_t)((Buffer[2]<<8) + Buffer[3])*mag_full/mag_gain;
	mag->y = (double)(int16_t)((Buffer[4]<<8) + Buffer[5])*mag_full/mag_gain;

	// mag calibration (ellipsoid fitting)
	mag->x = (mag->x - mag_offset.x) / mag_scale.x;
	mag->y = (mag->y - mag_offset.y) / mag_scale.y;
	mag->z = (mag->z - mag_offset.z) / mag_scale.z;

	Vec3Norm(mag);
}

void MagCalibration(Vec3d_t *offset, Vec3d_t *scale)
{
	Bluetooth_SendString("Mag Calibration start.\n");
	Bluetooth_SendString("Please rotate the sensor around all axes.\n");
	// OSTimeDly(3000);

	uint8_t sample_cnt = 100;

	Vec3d_t mag_data[sample_cnt];
	double matrix_k[sample_cnt][6], matrix_y[sample_cnt][1];
	for(uint8_t i=0; i<sample_cnt; i++)
	{
		GetMagData(&mag_data[i]);
		matrix_k[i][0] = mag_data[i].y*mag_data[i].y;
		matrix_k[i][1] = mag_data[i].z*mag_data[i].z;
		matrix_k[i][2] = mag_data[i].x;
		matrix_k[i][3] = mag_data[i].y;
		matrix_k[i][4] = mag_data[i].z;
		matrix_k[i][5] = 1;
		matrix_y[i][0] = -1*mag_data[i].x*mag_data[i].x;
		OSTimeDly(10);
	}

	// LeastSquare
	// X = (K^T * K)^-1 * K^T * Y
	double matrix_kt[6][sample_cnt], matrix_ktk[6][6], matrix_ktki[6][6], tmp4[6][sample_cnt], tmp5[6][1];
	MatrixTranspose((double *)matrix_kt, (double *)matrix_k, sample_cnt, 6);
	MatrixesMultiply((double *)matrix_ktk, (double *)matrix_kt, (double *)matrix_k, sample_cnt, 6, 6);
	MatrixInverse((double *)matrix_ktki, (double *)matrix_ktk, 6);
	MatrixesMultiply((double *)tmp4, (double *)matrix_ktki, (double *)matrix_kt, 6, 6, sample_cnt);
	MatrixesMultiply((double *)tmp5, (double *)tmp4, (double *)matrix_y, 6, sample_cnt, 1);

	// return offset and scale
	offset->x = -1*tmp5[2][0]/2;
	offset->y = -1*tmp5[3][0]/2/tmp5[0][0];
	offset->z = -1*tmp5[4][0]/2/tmp5[1][0];
	scale->x = sqrt(offset->x*offset->x+tmp5[0][0]*offset->y*offset->y+tmp5[1][0]*offset->z*offset->z-tmp5[5][0]);
	scale->y = sqrt(scale->x*scale->x/tmp5[0][0]);
	scale->z = sqrt(scale->x*scale->x/tmp5[1][0]);

	Bluetooth_SendString("done.\n");
	mag_calibrated = 1;
}

void MS5611Init(void)
{
	
}
