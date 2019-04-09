#ifndef __PARAMETERCONTROL_H
#define __PARAMETERCONTROL_H

#include "../usr_lib/matrix.h"
#include "../usr_lib/scurve.h"
#include "../main.h"

#define GAIT_STAND 0
#define GAIT_TROT 1
#define GAIT_WALK 2

#define ELBOW_ELBOW 0
#define ELBOW_KNEE 1

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
} Robot_MotionPara_t;

//机器人足底平面参数
typedef struct Robot_PlanePosturePara_t
{
	float Roll;
	float Pitch;
} Robot_PlanePosturePara_t;

typedef struct Robot_ZeroShift_t
{
	int LF;
	int LH;
	int RF;
	int RH;
} Robot_ZeroShift_t;

typedef struct Robot_InitialZeroShift_t
{
	int LF;
	int LH;
	int RF;
	int RH;
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
void Modify_Posture(void);
void SCurveCtrlInit(void);
void TimeKeeping(void);

#endif
