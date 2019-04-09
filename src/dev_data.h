#ifndef _DEV_DATA_H
#define _DEV_DATA_H

#include "dev/uart.h"

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

extern Remote_t RemoteData;

void AnalysisRemoteData(serial_frame_t *pFrame);
void DispRemoteData(void);
void UpdateRemoteParameter(void);

#endif
