#include "dev_data.h"

//extern fl2u8 GyroscopeData[10];
//extern u162u8 CameraData[3];

Remote_t RemoteData;

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
