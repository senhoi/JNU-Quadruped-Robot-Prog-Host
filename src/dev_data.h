#ifndef _DEV_DATA_H
#define _DEV_DATA_H

#include "dev/uart.h"
#include "ctrl/para_ctrl.h"
#include "usr_lib/filter.h"

enum Gait_t
{
	STANDING = 0,
	TROTTING,
	WALKING
};

enum Coordinate_t
{
	UNIVERSE_XYPY,
	POSITION_YZPY,
	POSITION_XZRY
};

typedef struct Remote_t
{
	int Joystick_LX;
	int Joystick_LY;
	int Joystick_RX;
	int Joystick_RY;
	unsigned char Dial;
	enum Gait_t Gait;
	enum Coordinate_t Coordinate;
} Remote_t;

typedef struct Gyro_t
{
	float Pitch;
	float Roll;
	float Yaw;
	float Gyro_X;
	float Gyro_Y;
	float Gyro_Z;
} Gyro_t;

typedef struct GyroFilter_t
{
	float CutFreq;
	float SampleFreq;
	Gyro_t Data;
	Gyro_t PrevData;
	Gyro_t FreshData;
} GyroFilter_t;

extern Remote_t RemoteData;
extern Gyro_t GyroData;
extern GyroFilter_t sGyroData_LowFreq;
extern GyroFilter_t sGyroData_HighFreq;

void AnalysisRemoteData(serial_frame_t *pFrame);
void AnalysisGyroData(serial_frame_t *pFrame, int filter);
void DispRemoteData(void);
void DispGyroData(void);
void CreateGyroLogFile(void);
void WriteGyroLogFile(void);
void SaveGyroLogFile(void);

#endif
