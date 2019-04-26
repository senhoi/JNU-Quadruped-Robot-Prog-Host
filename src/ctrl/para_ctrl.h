#ifndef __PARAMETERCONTROL_H
#define __PARAMETERCONTROL_H

#include "../usr_lib/matrix.h"
#include "../usr_lib/scurve.h"
#include "../usr_lib/pid.h"
#include "../main.h"

#define GAIT_STAND 0
#define GAIT_TROT 1
#define GAIT_WALK 2

#define ELBOW_ELBOW 0
#define ELBOW_KNEE 1

#define FOOT_STATUS_LF_UP 0x0E
#define FOOT_STATUS_RH_UP 0x0D
#define FOOT_STATUS_RF_UP 0x0B
#define FOOT_STATUS_LH_UP 0x07
#define FOOT_STATUS_ALL_DOWN 0x0F

#define ABOVE_PLAIN 0x00 //接触点在理想平面上方
#define DOWN_PLAIN 0x01  //接触点在理想平面下方
#define OFF_PLAIN 0x02   //未接触地面
#define FULL_PLAIN 0x03  //四足完全接触地面

typedef struct Robot_MatrixGroup_t
{
	Matrix_t LF;
	Matrix_t LH;
	Matrix_t RF;
	Matrix_t RH;
} Robot_MatrixGroup_t;

//机器人机械结构参数
typedef struct Robot_MechanicalPara_t
{
	float BodyWidth;
	float BodyLength;
	float BodyHeight;
	float Leg_a1; //大腿长度
	float Leg_a2; //小腿长度
	float Leg_d1; //股关节偏距
	float Leg_d2; //髋关节偏距
	float Leg_d3; //膝关节偏距
} Robot_MechanicalPara_t;

//机器人机身姿态参数（非机器人整体姿态）
typedef struct Robot_BodyPosturePara_t
{
	float X;
	float Y;
	float Z;
	float Roll;
	float Pitch;
	float Yaw;
} Robot_BodyPosturePara_t;

//机器人运动参数
typedef struct Robot_MotionPara_t
{
	int Structure;	//机器构型
	int Gait;		  //步态选择 1-Trot 2-Walk
	float Cycle;	  //步态周期	[s]
	float DutyRatio;  //占空比 取值 0~1
	float PhaseTotal; //当前步态对应总相位 2-Trot 4-Walk

	float Phase_A;  //当前A副相位
	float Phase_B;  //当前B副相位
	float Phase_LF; //当前LF副相位
	float Phase_LH; //当前LH副相位
	float Phase_RF; //当前RF副相位
	float Phase_RH; //当前RH副相位

	float Phase_Shift;
	int Phase_Shift_Flag;
	int Phase_Shift_FlagOld;
	int Phase_Shift_Dir;

	unsigned int Interval;			//同步信号周期	[ms]
	volatile unsigned long Time_S;  //运行时间	[s]
	volatile unsigned long Time_MS; //运行时间	[ms]

	int Span_Z;		 //前进方向跨距	[mm]
	int Span_X;		 //侧向跨距		[mm]
	int Span_Y;		 //抬腿高度		[mm]
	float Span_W;	//自转角度		[rad]
	float COG_Shift; //Walk步态平移量
	float Zero_Y;	//Walk步态抬腿量
} Robot_MotionPara_t;

//机器人足底平面参数
typedef struct Robot_PlanePosturePara_t
{
	float Roll;
	float Pitch;
} Robot_PlanePosturePara_t;

typedef struct Robot_ZeroShift_t
{
	int LF[3];
	int LH[3];
	int RF[3];
	int RH[3];
} Robot_ZeroShift_t;

typedef struct Robot_InitialZeroShift_t
{
	int LF[3]; //0-X 1-Y 2-Z （机身XYZ）
	int LH[3];
	int RF[3];
	int RH[3];
} Robot_InitialZeroShift_t;

extern Robot_MotionPara_t sRobot_MotionPara;
extern Robot_MechanicalPara_t sRobot_MechanicalPara;
extern Robot_BodyPosturePara_t sRobot_BodyPosturePara;
extern Robot_PlanePosturePara_t sRobot_PlanePosturePara;
extern Robot_InitialZeroShift_t sInitialZeroShift;
extern Robot_ZeroShift_t sRobot_ZeroShift;

extern Robot_MatrixGroup_t Body2Leg;
extern Robot_MatrixGroup_t Position2Zero;

extern Matrix_t Position2Body;
extern Matrix_t Plane2Body;
extern Matrix_t Plane2Position;

extern SCurveSpdCtrl_t sSCurveSpdCtrlSpan_Z;
extern SCurveSpdCtrl_t sSCurveSpdCtrlSpan_X;

void Init_AllPara(int gait);
void Init_MotionPara(int gait);
void Init_MechanicalPara(void);
void Init_BodyPosturePara(void);
void Init_PlanePosturePara(void);
void Init_InitialZeroShift(void);
void Init_ZeroShift(void);

void Calc_Body2Leg(void);
void Calc_Position2Zero(void);
void Calc_Plane2Position(void);
void Calc_Plane2Body(void);

void Reset_WalkPhase(void);
void Revise_WalkPhase(void);
void Modify_COG(void);
void Modify_Pitch(void); //若陀螺仪数据未经滤波，一定不要调用此函数！！！
void Modify_Posture(void);
void SCurveCtrl_Init(void);
void PID_Init(void);
void TimeKeeping(void);

#endif
