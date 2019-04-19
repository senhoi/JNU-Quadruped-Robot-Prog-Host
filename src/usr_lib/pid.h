#ifndef _PID_H
#define _PID_H

typedef struct PID_Regular_t
{
	float Kp;
	float Ki;
	float Kd;
	float Deadband;
	float Feedback;
	float Ref;
	float Output;
	float MaxOutput;
	float IntError;
	float MaxInt;
	float Error;
	float LastError;
	float PrevError;
} PID_Regular_t;

typedef struct PID_Increment_t
{
	float Kp;
	float Ki;
	float Kd;
	float Deadband;
	float Feedback;
	float Ref;
	float Output;
	float MaxIncrease;
	float MaxOutput;
	float IntError;
	float MaxInt;
	float Error;
	float LastError;
	float PrevError;
} PID_Increment_t;

void PID_Regular_Reset(PID_Regular_t *PID_Regular, float Kp, float Ki, float Kd, float Deadband, float MaxInt, float MaxOutput);
void PID_Regular_Cacl(PID_Regular_t *PID_Regular);
void PID_Increment_Reset(PID_Increment_t *PID_Increment, float Kp, float Ki, float Kd, float Deadband, float MaxIncrease, float MaxInt, float MaxOutput);
void PID_Increment_Calc(PID_Increment_t *PID_Increment);

#endif
