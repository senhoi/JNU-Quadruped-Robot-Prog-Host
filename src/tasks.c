#include "tasks.h"

#define FRAME_HEAD 0x55AA
#define FRAMETYPE_REMOTE 0x11
#define FRAMETYPE_CAMERA 0x21
#define FRAMETYPE_GYROSCOPE 0x22
#define FRAMETYPE_FOOTGROUNDING 0x23

Matrix_t sFootEndingPos;

float JointAngle[12];

int fd_serialport;
FILE *fd_log;

char LogFileName[128];

enum DevType_t
{
	RPI = 0,
	PC
} DevType = PC;

void CreateLogFile(void)
{
	char str_data[64];
	char str_cwd[64];
	time_t timer = time(NULL);

	strftime(str_data, sizeof(str_data), "%Y-%m-%d[%H:%M:%S]", localtime(&timer));
	strcat(LogFileName, str_data);
	strcat(LogFileName, ".txt");
	fd_log = fopen(LogFileName, "w");

	if (fd_log == NULL)
	{
		printf("Failed to create log file.\n");
	}
	else
	{
		LOG("%s", LogFileName);
	}

#ifdef GYRO_LOG
	CreateGyroLogFile();
#endif
}

void ProgStop(int signo)
{
	printf("\nProgram aborted.\n");
	serialClose(fd_serialport);
	if (!fclose(fd_log))
	{
		printf("Log file saved. Filename is %s.\n", LogFileName);
	}
	else
	{
		printf("Failed to closed log file. \n");
	}

#ifdef GYRO_LOG
	SaveGyroLogFile();
#endif

	_exit(0);
}

void InitTask(void)
{
	signal(SIGINT, ProgStop);
	CreateLogFile();

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

	//serialTest(fd_serialport, FRAME_HEAD);

	Init_AllPara(GAIT_WALK);
	SCurveCtrl_Init();
	PID_Init();
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
	//Modify_Pitch(); //若陀螺仪数据未经滤波，一定不要调用此函数！！！
	Modify_Posture();

	switch (sRobot_MotionPara.Gait)
	{
	case GAIT_STAND:
		sFootEndingPos = Calc_GaitTrajPoint_Trot();
		break;
	case GAIT_TROT:
		Calc_Position2Zero();
		Calc_GaitTrajPolyCoeffi_Trot();
		sFootEndingPos = Calc_GaitTrajPoint_Trot();
		break;
	case GAIT_WALK:
		Calc_Position2Zero();
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

	serialSendFrameHead(fd_serialport, FRAME_HEAD);
	serialSendFloatArr(fd_serialport, 12, JointAngle, 1);

#ifdef GYRO_LOG
	WriteGyroLogFile();
#endif
}

void DisplayTask(void)
{
	DispRemoteData();
	DispGyroData();
	DispFootGroundingData(FootGrounding);
}

void RevTask(void)
{
	serial_frame_t sFrame;
	if (!serialRevFrame(&sFrame, fd_serialport, FRAME_HEAD))
	{
		switch (sFrame.type)
		{
		case FRAMETYPE_FOOTGROUNDING:
			AnalysisFootGroundingData(&sFrame);
			break;

		case FRAMETYPE_REMOTE:
			AnalysisRemoteData(&sFrame);
			break;

		case FRAMETYPE_GYROSCOPE:
			AnalysisGyroData(&sFrame, 1);
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
	//DisplayTask();
	RevTask();
}
