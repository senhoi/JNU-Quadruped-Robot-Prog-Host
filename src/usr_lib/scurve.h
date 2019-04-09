#ifndef _SCURVE_H
#define _SCURVE_H

#include "stdio.h"
#include "math.h"

#define SCURVE_EQUAL_ERROR 0.0001f

typedef struct SCurvePosCtrl_t
{
	/*设定参数*/
	char Flag;				//S曲线位置控制执行标志位
	unsigned int Precision; //求解精度
	float Time;				//当前时间
	float TimeInc;			//每次执行的离散时间增量
	float Cycle;			//每次执行的总时长（若Cycle>0则为CycleLimited模式，否则为MaxSpdLimited模式）
	float AccStep;			//自动求解加速度时步长（绝对值）
	float Acc;				//变速段加速度（绝对值）
	float Dec;				//不可修改，绝对值与Acc绝对值保持一致
	float MaxSpd;			//最大速度（绝对值）
	float OriginPos;		//起点位置
	float FinishPos;		//终点位置

	/*中间变量参数*/
	float AccTime; //加速段时间
	float MidTime; //匀速段时间
	float DecTime; //减速段时间
	float Spd;	 //当前控制速度
	float Pos;	 //当前控制位置

	/*过程变量参数*/
	float DeltaPos;
	float DeltaPos_MaxSpdLimitedCondition0;
	float DeltaPos_MaxSpdLimitedCondition1;
	float DeltaPos_MaxSpdLimitedCondition2;
	float DeltaPos_CycleLimitedCondition0;
	float DeltaPos_CycleLimitedCondition1;
	float DeltaPos_CycleLimitedCondition2;
	float DeltaPos_CycleLimitedCondition3;

	/*输出参数*/
	float PosOutput; //位置输出
} SCurvePosCtrl_t;

typedef struct SCurveSpdCtrl_t
{
	/*设定参数*/
	char Flag;		 //S曲线位置控制执行标志位
	float TimeInc;   //每次执行的离散时间增量
	float Acc;		 //加速段加速度（绝对值）
	float Dec;		 //减速段加速度（绝对值）
	float TargetSpd; //目标速度

	/*中间变量参数*/
	float Spd; //当前控制速度

	/*输出参数*/
	float SpdOutput; //速度输出
} SCurveSpdCtrl_t;

void SCurvePosCtrl_New(SCurvePosCtrl_t *pSCurvePosCtrl, unsigned int precision_level, float time_inc, float cycle, float acc_step, float acc, float max_spd, float origin_pos, float finish_pos);
void SCurvePosCtrl_SetNewPos(SCurvePosCtrl_t *pSCurvePosCtrl, float cycle, float max_spd, float acc, float finish_pos);
int SCurvePosCtrl_Calc(SCurvePosCtrl_t *pSCurvePosCtrl);

void SCurveSpdCtrl_New(SCurveSpdCtrl_t *pSCurveSpdCtrl, float time_inc, float acc, float dec, float target_spd);
void SCurveCtrl_SetNewSpd(SCurveSpdCtrl_t *pSCurveSpdCtrl, float target_spd);
int SCurveSpdCtrl_Calc(SCurveSpdCtrl_t *pSCurveSpdCtrl);

#endif
