#include "dev_data.h"

//extern fl2u8 GyroscopeData[10];
//extern u162u8 CameraData[3];

Remote_t RemoteData;
Gyro_t GyroData;

void AnalysisRemoteData(serial_frame_t *pFrame)
{

	RemoteData.Joystick_LX = (int8_t)pFrame->pdata[0];
	RemoteData.Joystick_LY = (int8_t)pFrame->pdata[1];
	RemoteData.Joystick_RX = (int8_t)pFrame->pdata[2];
	RemoteData.Joystick_RY = (int8_t)pFrame->pdata[3];
	RemoteData.Gait = pFrame->pdata[4];
	RemoteData.Coordinate = pFrame->pdata[5];
	RemoteData.Dial = pFrame->pdata[6];
}

void AnalysisGyroData(serial_frame_t *pFrame)
{
	float_uint8_t gyro_temp_arr[6];

	for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			gyro_temp_arr[i].u8[j] = pFrame->pdata[i * 4 + j];
		}
	}

	GyroData.Pitch = gyro_temp_arr[0].fl;
	GyroData.Roll = gyro_temp_arr[1].fl;
	GyroData.Yaw = gyro_temp_arr[2].fl;
	GyroData.Gyro_X = gyro_temp_arr[3].fl;
	GyroData.Gyro_Y = gyro_temp_arr[4].fl;
	GyroData.Gyro_Z = gyro_temp_arr[5].fl;
}

void DispRemoteData(void)
{
	printf("Remote Data:\n");
	printf("LX:%d\t", RemoteData.Joystick_LX);
	printf("LY:%d\t", RemoteData.Joystick_LY);
	printf("RX:%d\t", RemoteData.Joystick_RX);
	printf("RY:%d\t", RemoteData.Joystick_RY);
	printf("\n");
	printf("LS:%d\t", RemoteData.Gait);
	printf("RS:%d\t", RemoteData.Coordinate);
	printf("VA:%d\t", RemoteData.Dial);
	printf("\n");
}

void DispGyroData(void)
{
	printf("Gyro Data:\n");
	printf("Pitch:%f\t", GyroData.Pitch);
	printf("Roll:%f\t", GyroData.Roll);
	printf("Yaw:%f\t", GyroData.Yaw);
	printf("\n");
	printf("Gyro_X:%f\t", GyroData.Gyro_X);
	printf("Gyro_Y:%f\t", GyroData.Gyro_Y);
	printf("Gyro_Z:%f\t", GyroData.Gyro_Z);
	printf("\n");
}

static FILE *fd_gyro_log;
char GyroLogFileName[128];

void CreateGyroLogFile(void)
{
	char str_data[64];
	char str_cwd[64];
	time_t timer = time(NULL);

	strftime(str_data, sizeof(str_data), "%Y-%m-%d[%H:%M:%S][GYRO]", localtime(&timer));
	strcat(GyroLogFileName, str_data);
	strcat(GyroLogFileName, ".txt");
	fd_gyro_log = fopen(GyroLogFileName, "w");

	if (fd_gyro_log == NULL)
	{
		printf("Failed to create gyro log file.\n");
	}
}

void WriteGyroLogFile(void)
{
	fprintf(fd_gyro_log, "%f\t%f\t%f\t%f\t%f\t%f\t%f\n", sRobot_MotionPara.Time_S + sRobot_MotionPara.Time_MS / 1000.0, GyroData.Pitch, GyroData.Roll, GyroData.Yaw, GyroData.Gyro_X, GyroData.Gyro_Y, GyroData.Gyro_Z);
}

void SaveGyroLogFile(void)
{
	if (!fclose(fd_gyro_log))
	{
		printf("Gyro log file saved. Filename is %s.\n", GyroLogFileName);
	}
}
