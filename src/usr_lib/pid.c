#include "pid.h"

static void absLimit(float *num, float Limit)
{
	if (*num > Limit)
	{
		*num = Limit;
	}
	else if (*num < -Limit)
	{
		*num = -Limit;
	}
}

void PID_Regular_Reset(PID_Regular_t *PID_Regular, float Kp, float Ki, float Kd, float Deadband, float MaxInt, float MaxOutput)
{
	/*设定PID系数及最大输出*/
	PID_Regular->Kp = Kp;
	PID_Regular->Ki = Ki;
	PID_Regular->Kd = Kd;
	PID_Regular->MaxInt = MaxInt;
	PID_Regular->MaxOutput = MaxOutput;

	/*清除误差记录*/
	PID_Regular->Error = 0.0f;
	PID_Regular->LastError = 0.0f;
	PID_Regular->PrevError = 0.0f;
	PID_Regular->IntError = 0.0f;

	/*清除输入输出*/
	PID_Regular->Feedback = 0.0f;
	PID_Regular->Ref = 0.0f;
	PID_Regular->Output = 0.0f;
}
void PID_Regular_Cacl(PID_Regular_t *PID_Regular)
{
	static float Error = 0.0f, Integrate = 0.0f, Delta = 0.0f, Output = 0.0f;
	Error = PID_Regular->Ref - PID_Regular->Feedback;
	Delta = Error - PID_Regular->Error;
	PID_Regular->PrevError = PID_Regular->LastError;
	PID_Regular->LastError = PID_Regular->Error;
	PID_Regular->Error = Error;
	Integrate = Error + PID_Regular->IntError;
	absLimit(&Integrate, PID_Regular->MaxInt);
	PID_Regular->IntError = Integrate;
	Output = PID_Regular->Kp * Error + PID_Regular->Ki * Integrate - PID_Regular->Kd * Delta;
	absLimit(&Output, PID_Regular->MaxOutput);
	PID_Regular->Output = Output;
}

void PID_Increment_Reset(
	PID_Increment_t *PID_Increment, float Kp, float Ki, float Kd,
	float Deadband, float MaxIncrease, float MaxInt, float MaxOutput)
{
	/*设定PID系数及最大输出*/
	PID_Increment->Kp = Kp;
	PID_Increment->Ki = Ki;
	PID_Increment->Kd = Kd;
	PID_Increment->Deadband = Deadband;
	PID_Increment->MaxIncrease = MaxIncrease;
	PID_Increment->MaxInt = MaxInt;
	PID_Increment->MaxOutput = MaxOutput;

	/*清除误差记录*/
	PID_Increment->Error = 0.0f;
	PID_Increment->LastError = 0.0f;
	PID_Increment->PrevError = 0.0f;
	PID_Increment->IntError = 0.0f;

	/*清除输入输出*/
	PID_Increment->Feedback = 0.0f;
	PID_Increment->Ref = 0.0f;
	PID_Increment->Output = 0.0f;
}

void PID_Increment_Calc(PID_Increment_t *PID_Increment)
{
	static float Error = 0.0f, DeltaOutput = 0.0f, Output = 0.0f;
	Error = PID_Increment->Ref - PID_Increment->Feedback;
	PID_Increment->PrevError = PID_Increment->LastError;
	PID_Increment->LastError = PID_Increment->Error;
	PID_Increment->Error = Error;
	DeltaOutput = PID_Increment->Kp * (PID_Increment->Error - PID_Increment->LastError) + PID_Increment->Ki * PID_Increment->Error + PID_Increment->Kd * (PID_Increment->Error - 2 * PID_Increment->LastError + PID_Increment->PrevError);
	absLimit(&DeltaOutput, PID_Increment->MaxIncrease);
	Output = PID_Increment->Output + DeltaOutput;
	absLimit(&Output, PID_Increment->MaxOutput);
	PID_Increment->Output = Output;
}
