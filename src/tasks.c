#include "tasks.h"

#define FRAME_HEAD 0xAA55AA55
#define FRAMETYPE_REMOTE 0x11
#define FRAMETYPE_CAMERA 0x21
#define FRAMETYPE_GYROSCOPE 0x22

Matrix_t sFootEndingPos;

float JointAngle[12];

int fd_serialport;

enum DevType_t
{
	RPI = 0,
	PC
} DevType = PC;

void InitTask(void)
{
	if (setProgPri(100) != 0)
		PRINTF_WARNING("Set priority unsuccessfully.\n");

	switch (DevType)
	{
	case RPI:
		fd_serialport = serialOpen("/dev/ttyAMA0", 115200);
		PRINTF_TIPS("Open /dev/ttyAMA0");
		break;
	case PC:
		fd_serialport = serialOpen("/dev/ttyUSB0", 115200);
		PRINTF_TIPS("Open /dev/ttyUSB0");
		break;
	}
	if (fd_serialport < 0)
	{
		PRINTF_WARNING("SerialPort failed to initialize. -err:%d", fd_serialport);
		PRINTF_TIPS("Please check whether there is a serial device plugged in.");
		PRINTF_TIPS("Press ENTER to continue without connection.");
		getchar();
	}
	else
	{
		PRINTF_TIPS("SerialPort initialized successfully. fd:%d", fd_serialport);
	}

//	serialTest(fd_serialport, FRAME_HEAD);

	Init_AllPara(GAIT_TROT);
	SCurveCtrlInit();
	Calc_Position2Zero();
	Calc_Body2Leg();
	Modify_Posture();
	switch (sRobot_MotionPara.Gait)
	{
	case 1:
		Calc_GaitTrajPolyCoeffi_Trot();
		break;
	case 2:
		Calc_GaitTrajPolyCoeffi_Walk();
		break;
	}
}

void InterruptTask(void)
{
	Matrix_t buffer;

	TimeKeeping();
	ParaUpdate(0);

	Modify_COG();
	Modify_Posture();

	switch (sRobot_MotionPara.Gait)
	{
	case GAIT_STAND:
		sFootEndingPos = Calc_GaitTrajPoint_Trot();
		break;
	case GAIT_TROT:
		//Calc_Position2Zero();
		Calc_GaitTrajPolyCoeffi_Trot();
		sFootEndingPos = Calc_GaitTrajPoint_Trot();
		break;
	case GAIT_WALK:
		Calc_GaitTrajPolyCoeffi_Walk();
		sFootEndingPos = Calc_GaitTrajPoint_Walk();
		break;
	}

	MatrixDisplay(sFootEndingPos);
	for (int i = 0; i < 4; i++)
	{
		buffer = InvKineCalc(i, sFootEndingPos.pMatrix[i * 3 + 0], sFootEndingPos.pMatrix[i * 3 + 1], sFootEndingPos.pMatrix[i * 3 + 2]);
		JointAngle[i * 3 + 0] = buffer.pMatrix[0];
		JointAngle[i * 3 + 1] = buffer.pMatrix[1];
		JointAngle[i * 3 + 2] = buffer.pMatrix[2];
		free(buffer.pMatrix);
	}
	free(sFootEndingPos.pMatrix);

	serialSendFloatArr(fd_serialport, 12, JointAngle, 1);
}

void DisplayTask(void)
{
	DispRemoteData();
}

void RevTask(void)
{
	serial_frame_t sFrame;
	if (!serialRevFrame(&sFrame, fd_serialport, FRAME_HEAD))
	{
		switch (sFrame.type)
		{
		case FRAMETYPE_REMOTE:
			AnalysisRemoteData(&sFrame);
			break;

		default:
			PRINTF_WARNING("Unknown frame type.");
			break;
		}
		free(sFrame.pdata);
	}
}

void LowPriorityTask(void)
{
	DisplayTask();
	RevTask();
}
