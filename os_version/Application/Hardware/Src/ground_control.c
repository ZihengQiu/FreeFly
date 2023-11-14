#include "ground_control.h"
#include "attitude.h"
#include "bluetooth.h"
#include "control.h"
#include "mathkit.h"
#include "receiver.h"

extern vec3d_t euler;
// extern pid_t pid_roll[2], pid_pitch[2], pid_yaw[2];

void SendAnotcSensor()
{
	// send sensor data to anotc v7
	//0xAA 0xFF 0x03 LEN=13 DATA SC AC
	uint8_t sum_check = 0, add_check = 0;
	uint8_t data[32] = {0x00};
	data[0] = 0xAA;
	data[1] = 0xFF;
	data[2] = 0x03;
	data[3] = 0x0D;
	int16_t acc_x = (int16_t)(acc.x),
			acc_y = (int16_t)(acc.y),
			acc_z = (int16_t)(acc.z),
			gyro_x = (int16_t)(gyro.x),
			gyro_y = (int16_t)(gyro.y),
			gyro_z = (int16_t)(gyro.z);
	data[4] = (acc_x)&0xFF;
	data[5] = (acc_x)>>8;
	data[6] = (acc_y)&0xFF;
	data[7] = (acc_y)>>8;
	data[8] = (acc_z)&0xFF;
	data[9] = (acc_z)>>8;
	data[10] = (gyro_x)&0xFF;
	data[11] = (gyro_x)>>8;
	data[12] = (gyro_y)&0xFF;
	data[13] = (gyro_y)>>8;
	data[14] = (gyro_z)&0xFF;
	data[15] = (gyro_z)>>8;
	data[16] = 1;

	for(uint8_t i=0; i<data[3]+4; i++)
	{
		sum_check += data[i];
		add_check += sum_check;
	}
	data[data[3]+4] = sum_check;
	data[data[3]+5] = add_check;
	for(uint8_t i=0; i<6+data[3]; i++)
	{
		Bluetooth_SendByte(data[i]);
	}
}

void SendAnotcEuler()
{
	// send euler angle data to anotc v7
	//0xAA 0xFF 0x03 LEN=7 DATA SC AC
	uint8_t sum_check = 0, add_check = 0;
	uint8_t data[32] = {0x00};
	data[0] = 0xAA;
	data[1] = 0xFF;
	data[2] = 0x03;
	data[3] = 0x07;
	int16_t roll = (int16_t)(euler.x*100),
			pitch = (int16_t)(euler.y*100),
			yaw = (int16_t)(euler.z*100);
	data[4] = (roll)&0xFF;
	data[5] = (roll)>>8;
	data[6] = (pitch)&0xFF;
	data[7] = (pitch)>>8;
	data[8] = (yaw)&0xFF;
	data[9] = (yaw)>>8;
	data[10] = 1;

	for(uint8_t i=0; i<data[3]+4; i++)
	{
		sum_check += data[i];
		add_check += sum_check;
	}
	data[data[3]+4] = sum_check;
	data[data[3]+5] = add_check;
	for(uint8_t i=0; i<6+data[3]; i++)
	{
		Bluetooth_SendByte(data[i]);
	}
}

void SendAnotcPID()
{
	// send pid data to anotc v7 using flexible frame transmission
	//0xAA 0xFF 0xF1 LEN DATA SC AC
	uint8_t sum_check = 0, add_check = 0;
	uint8_t data[32] = {0x00};
	data[0] = 0xAA;
	data[1] = 0xFF;
	data[2] = 0xF1;
	data[3] = 0x12;
	int16_t roll_target = (int16_t)(pid_roll[0].target*100),
			roll_out_o = (int16_t)(pid_roll[0].out*100),
			roll_out_i = (int16_t)(pid_roll[1].out*100),
			pitch_target = (int16_t)(pid_pitch[0].target*100),
			pitch_out_o = (int16_t)(pid_pitch[0].out*100),
			pitch_out_i = (int16_t)(pid_pitch[1].out*100),
			yaw_target = (int16_t)(pid_yaw[0].target*100),
			yaw_out_o = (int16_t)(pid_yaw[0].out*100),
			yaw_out_i = (int16_t)(pid_yaw[1].out*100);
			
	data[4] = (roll_target)&0xFF;
	data[5] = (roll_target)>>8;
	data[6] = (roll_out_o)&0xFF;
	data[7] = (roll_out_o)>>8;
	data[8] = (roll_out_i)&0xFF;
	data[9] = (roll_out_i)>>8;
	data[10] = (pitch_target)&0xFF;
	data[11] = (pitch_target)>>8;
	data[12] = (pitch_out_o)&0xFF;
	data[13] = (pitch_out_o)>>8;
	data[14] = (pitch_out_i)&0xFF;
	data[15] = (pitch_out_i)>>8;
	data[16] = (yaw_target)&0xFF;
	data[17] = (yaw_target)>>8;
	data[18] = (yaw_out_o)&0xFF;
	data[19] = (yaw_out_o)>>8;
	data[20] = (yaw_out_i)&0xFF;
	data[21] = (yaw_out_i)>>8;

	for(uint8_t i=0; i<data[3]+4; i++)
	{
		sum_check += data[i];
		add_check += sum_check;
	}
	data[data[3]+4] = sum_check;
	data[data[3]+5] = add_check;
	for(uint8_t i=0; i<6+data[3]; i++)
	{
		Bluetooth_SendByte(data[i]);
	}
}

void SendAnotcController()
{
	// send controller data to anotc v7 using flexible frame transmission
	//0xAA 0xFF 0xF1 LEN DATA SC AC
	uint8_t sum_check = 0, add_check = 0;
	uint8_t data[32] = {0x00};
	data[0] = 0xAA;
	data[1] = 0xFF;
	data[2] = 0xF1;
	data[3] = 0x08;
	int16_t thr = (int16_t)(ppm_val[THR]),
			ail = (int16_t)(ppm_val[AIL]),
			ele = (int16_t)(ppm_val[ELE]),
			rud = (int16_t)(ppm_val[RUD]);
			
	data[4] = (thr)&0xFF;
	data[5] = (thr)>>8;
	data[6] = (ail)&0xFF;
	data[7] = (ail)>>8;
	data[8] = (ele)&0xFF;
	data[9] = (ele)>>8;
	data[10] = (rud)&0xFF;
	data[11] = (rud)>>8;

	for(uint8_t i=0; i<data[3]+4; i++)
	{
		sum_check += data[i];
		add_check += sum_check;
	}
	data[data[3]+4] = sum_check;
	data[data[3]+5] = add_check;
	for(uint8_t i=0; i<6+data[3]; i++)
	{
		Bluetooth_SendByte(data[i]);
	}
}

void SendAnotc(void)
{
	SendAnotcEuler();
	SendAnotcSensor();
	SendAnotcController();
	SendAnotcPID();
}