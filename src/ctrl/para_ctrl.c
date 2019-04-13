#include "para_ctrl.h"

Robot_MotionPara_t sRobot_MotionPara;
Robot_MechanicalPara_t sRobot_MechanicalPara;
Robot_BodyPosturePara_t sRobot_BodyPosturePara;
Robot_PlanePosturePara_t sRobot_PlanePosturePara;
Robot_InitialZeroShift_t sRobot_InitialZeroShift;
Robot_ZeroShift_t sRobot_ZeroShift;

Robot_MatrixGroup_t Body2Leg;
Robot_MatrixGroup_t Position2Zero;

Matrix_t Plane2Body;
Matrix_t Plane2Position;

void Init_AllPara(int gait)
{
	/*Motion parameters must be initialized first.*/
	Init_MotionPara(gait);
	Init_MechanicalPara();
	Init_BodyPosturePara();
	Init_PlanePosturePara();
	Init_InitialZeroShift();
	Init_ZeroShift();
}

void Init_MotionPara(int gait)
{
	sRobot_MotionPara.Gait = gait;
	switch (gait)
	{
	case GAIT_STAND:
		sRobot_MotionPara.Span_Z = 0;
		sRobot_MotionPara.Span_X = 0;
		sRobot_MotionPara.Span_Y = 0;
		sRobot_MotionPara.Span_W = 0.0f;
		break;

	case GAIT_TROT:
		sRobot_MotionPara.Cycle = 1.0f;
		sRobot_MotionPara.DutyRatio = 0.5f;
		sRobot_MotionPara.PhaseTotal = 2.0f;
		sRobot_MotionPara.Phase_A = 0.0f;
		sRobot_MotionPara.Phase_B = 1.0f;
		sRobot_MotionPara.Interval = 10;
		sRobot_MotionPara.Span_Z = 100;
		sRobot_MotionPara.Span_X = 0;
		sRobot_MotionPara.Span_Y = 30;
		sRobot_MotionPara.Span_W = 0.0f;
		break;

	case GAIT_WALK:
		sRobot_MotionPara.Cycle = 3.6f;
		sRobot_MotionPara.DutyRatio = 0.82f;
		sRobot_MotionPara.PhaseTotal = 4.0f;
		sRobot_MotionPara.Phase_Shift = 0.0f;
		sRobot_MotionPara.Phase_Shift_Flag = 1;
		sRobot_MotionPara.Phase_Shift_FlagOld = 1;
		sRobot_MotionPara.Phase_Shift_Dir = 1;
		sRobot_MotionPara.Interval = 10;
		sRobot_MotionPara.Span_Z = 100;
		sRobot_MotionPara.Span_X = 0;
		sRobot_MotionPara.Span_Y = 30;
		sRobot_MotionPara.Span_W = 0.0f;
		sRobot_MotionPara.COG_Shift = 30;
		Reset_WalkPhase();
		break;

	default:
		break;
	}
}

void Init_MechanicalPara(void)
{
#ifndef __SERVO_ROBOT
	sRobot_MechanicalPara.BodyLength = 600;
	sRobot_MechanicalPara.BodyWidth = 250;
	sRobot_MechanicalPara.BodyHeight = 550;
	sRobot_MechanicalPara.Leg_a1 = 350;
	sRobot_MechanicalPara.Leg_a2 = 325;
	sRobot_MechanicalPara.Leg_d1 = 0;
	sRobot_MechanicalPara.Leg_d2 = 0;
	sRobot_MechanicalPara.Leg_d3 = 0;

#else
	sRobot_MechanicalPara.BodyLength = 337.4;
	sRobot_MechanicalPara.BodyWidth = 113;
	sRobot_MechanicalPara.BodyHeight = 180;
	sRobot_MechanicalPara.Leg_a1 = 130;
	sRobot_MechanicalPara.Leg_a2 = 130;
	sRobot_MechanicalPara.Leg_d1 = 0;
	sRobot_MechanicalPara.Leg_d2 = 40;
	sRobot_MechanicalPara.Leg_d3 = 32;

#endif
}

void Init_BodyPosturePara(void)
{
#ifndef __SERVO_ROBOT
	sRobot_BodyPosturePara.X = 0;
	sRobot_BodyPosturePara.Y = 0;
	sRobot_BodyPosturePara.Z = 550;
	sRobot_BodyPosturePara.Roll = 0;
	sRobot_BodyPosturePara.Pitch = 0;
	sRobot_BodyPosturePara.Yaw = 0;

#else
	sRobot_BodyPosturePara.X = 0;
	sRobot_BodyPosturePara.Y = 0;
	sRobot_BodyPosturePara.Z = 180;
	sRobot_BodyPosturePara.Roll = 0;
	sRobot_BodyPosturePara.Pitch = 0;
	sRobot_BodyPosturePara.Yaw = 0;

#endif
}

void Init_PlanePosturePara(void)
{
	sRobot_PlanePosturePara.Roll = 0;
	sRobot_PlanePosturePara.Pitch = 0;
}

void Init_InitialZeroShift(void)
{
	sRobot_InitialZeroShift.LF = 0;
	sRobot_InitialZeroShift.LH = 0;
	sRobot_InitialZeroShift.RF = 0;
	sRobot_InitialZeroShift.RH = 0;
}

void Init_ZeroShift(void)
{
	sRobot_ZeroShift.LF = 0;
	sRobot_ZeroShift.LH = 0;
	sRobot_ZeroShift.RF = 0;
	sRobot_ZeroShift.RH = 0;
}

void Reset_WalkPhase(void)
{
	sRobot_MotionPara.Phase_LH = 0;
	sRobot_MotionPara.Phase_RF = sRobot_MotionPara.PhaseTotal - (1 - sRobot_MotionPara.DutyRatio) * sRobot_MotionPara.PhaseTotal - sRobot_MotionPara.PhaseTotal / 2;
	sRobot_MotionPara.Phase_RH = 2;
	sRobot_MotionPara.Phase_LF = sRobot_MotionPara.PhaseTotal - (1 - sRobot_MotionPara.DutyRatio) * sRobot_MotionPara.PhaseTotal;
}

void Revise_WalkPhase(void)
{
	sRobot_MotionPara.Phase_LH = sRobot_MotionPara.Phase_LH;
	sRobot_MotionPara.Phase_RF = sRobot_MotionPara.Phase_LH + sRobot_MotionPara.PhaseTotal - (1 - sRobot_MotionPara.DutyRatio) * sRobot_MotionPara.PhaseTotal - sRobot_MotionPara.PhaseTotal / 2;
	sRobot_MotionPara.Phase_RH = sRobot_MotionPara.Phase_LH + 2;
	sRobot_MotionPara.Phase_LF = sRobot_MotionPara.Phase_LH + sRobot_MotionPara.PhaseTotal - (1 - sRobot_MotionPara.DutyRatio) * sRobot_MotionPara.PhaseTotal;
	if (sRobot_MotionPara.Phase_LF > sRobot_MotionPara.PhaseTotal)
		sRobot_MotionPara.Phase_LF -= sRobot_MotionPara.PhaseTotal;
	if (sRobot_MotionPara.Phase_LH > sRobot_MotionPara.PhaseTotal)
		sRobot_MotionPara.Phase_LH -= sRobot_MotionPara.PhaseTotal;
	if (sRobot_MotionPara.Phase_RF > sRobot_MotionPara.PhaseTotal)
		sRobot_MotionPara.Phase_RF -= sRobot_MotionPara.PhaseTotal;
	if (sRobot_MotionPara.Phase_RH > sRobot_MotionPara.PhaseTotal)
		sRobot_MotionPara.Phase_RH -= sRobot_MotionPara.PhaseTotal;
}

void Calc_Body2Leg(void)
{
	Matrix_t MatrixTranslation, MatrixRotation_X, MatrixRotation_Z;

	MatrixTranslation = SE3_T(sRobot_MechanicalPara.BodyLength / 2, sRobot_MechanicalPara.BodyWidth / 2, 0);
	MatrixRotation_Z = SE3_Rz(90);
	MatrixRotation_X = SE3_Rx(90);
	Body2Leg.LF = MatrixMultiply(3, MatrixTranslation, MatrixRotation_Z, MatrixRotation_X);
	free(MatrixTranslation.pMatrix);
	free(MatrixRotation_X.pMatrix);
	free(MatrixRotation_Z.pMatrix);

	MatrixTranslation = SE3_T(-sRobot_MechanicalPara.BodyLength / 2, sRobot_MechanicalPara.BodyWidth / 2, 0);
	MatrixRotation_Z = SE3_Rz(90);
	MatrixRotation_X = SE3_Rx(-90);
	Body2Leg.LH = MatrixMultiply(3, MatrixTranslation, MatrixRotation_Z, MatrixRotation_X);
	free(MatrixTranslation.pMatrix);
	free(MatrixRotation_X.pMatrix);
	free(MatrixRotation_Z.pMatrix);

	MatrixTranslation = SE3_T(sRobot_MechanicalPara.BodyLength / 2, -sRobot_MechanicalPara.BodyWidth / 2, 0);
	MatrixRotation_Z = SE3_Rz(-90);
	MatrixRotation_X = SE3_Rx(-90);
	Body2Leg.RF = MatrixMultiply(3, MatrixTranslation, MatrixRotation_Z, MatrixRotation_X);
	free(MatrixTranslation.pMatrix);
	free(MatrixRotation_X.pMatrix);
	free(MatrixRotation_Z.pMatrix);

	MatrixTranslation = SE3_T(-sRobot_MechanicalPara.BodyLength / 2, -sRobot_MechanicalPara.BodyWidth / 2, 0);
	MatrixRotation_Z = SE3_Rz(-90);
	MatrixRotation_X = SE3_Rx(90);
	Body2Leg.RH = MatrixMultiply(3, MatrixTranslation, MatrixRotation_Z, MatrixRotation_X);
	free(MatrixTranslation.pMatrix);
	free(MatrixRotation_X.pMatrix);
	free(MatrixRotation_Z.pMatrix);
}

void Calc_Position2Zero(void)
{
	Matrix_t MatrixTranslation, MatrixRotation_X, MatrixRotation_Z;

	sRobot_ZeroShift.LF = sRobot_InitialZeroShift.LF - sRobot_MotionPara.Span_Z / 2 + 3.0 / 10 * sRobot_MotionPara.Span_Z;
	sRobot_ZeroShift.RF = sRobot_InitialZeroShift.RF - sRobot_MotionPara.Span_Z / 2 + 3.0 / 10 * sRobot_MotionPara.Span_Z;
	sRobot_ZeroShift.LH = sRobot_InitialZeroShift.LH + sRobot_MotionPara.Span_Z / 2 - 3.0 / 10 * sRobot_MotionPara.Span_Z;
	sRobot_ZeroShift.RH = sRobot_InitialZeroShift.RH + sRobot_MotionPara.Span_Z / 2 - 3.0 / 10 * sRobot_MotionPara.Span_Z;

	//Position2Zero.XX
	MatrixTranslation = SE3_T(sRobot_ZeroShift.LF + sRobot_MechanicalPara.BodyLength / 2, sRobot_MechanicalPara.BodyWidth / 2 + sRobot_MechanicalPara.Leg_d2 + sRobot_MechanicalPara.Leg_d3, 0);
	MatrixRotation_Z = SE3_Rz(90);
	MatrixRotation_X = SE3_Rx(90);
	Position2Zero.LF = MatrixMultiply(3, MatrixTranslation, MatrixRotation_Z, MatrixRotation_X);
	free(MatrixTranslation.pMatrix);
	free(MatrixRotation_X.pMatrix);
	free(MatrixRotation_Z.pMatrix);

	MatrixTranslation = SE3_T(-(sRobot_ZeroShift.LH + sRobot_MechanicalPara.BodyLength / 2), sRobot_MechanicalPara.BodyWidth / 2 + sRobot_MechanicalPara.Leg_d2 + sRobot_MechanicalPara.Leg_d3, 0);
	MatrixRotation_Z = SE3_Rz(90);
	MatrixRotation_X = SE3_Rx(-90);
	Position2Zero.LH = MatrixMultiply(3, MatrixTranslation, MatrixRotation_Z, MatrixRotation_X);
	free(MatrixTranslation.pMatrix);
	free(MatrixRotation_X.pMatrix);
	free(MatrixRotation_Z.pMatrix);

	MatrixTranslation = SE3_T(sRobot_ZeroShift.RF + sRobot_MechanicalPara.BodyLength / 2, -sRobot_MechanicalPara.BodyWidth / 2 - sRobot_MechanicalPara.Leg_d2 - sRobot_MechanicalPara.Leg_d3, 0);
	MatrixRotation_Z = SE3_Rz(-90);
	MatrixRotation_X = SE3_Rx(-90);
	Position2Zero.RF = MatrixMultiply(3, MatrixTranslation, MatrixRotation_Z, MatrixRotation_X);
	free(MatrixTranslation.pMatrix);
	free(MatrixRotation_X.pMatrix);
	free(MatrixRotation_Z.pMatrix);

	MatrixTranslation = SE3_T(-(sRobot_ZeroShift.RH + sRobot_MechanicalPara.BodyLength / 2), -sRobot_MechanicalPara.BodyWidth / 2 - sRobot_MechanicalPara.Leg_d2 - sRobot_MechanicalPara.Leg_d3, 0);
	MatrixRotation_Z = SE3_Rz(-90);
	MatrixRotation_X = SE3_Rx(90);
	Position2Zero.RH = MatrixMultiply(3, MatrixTranslation, MatrixRotation_Z, MatrixRotation_X);
	free(MatrixTranslation.pMatrix);
	free(MatrixRotation_X.pMatrix);
	free(MatrixRotation_Z.pMatrix);
}

void Calc_Plane2Position(void)
{
	Matrix_t Matrix_Rx, Matrix_Ry;

	Matrix_Rx = SE3_Rx(sRobot_PlanePosturePara.Roll);
	Matrix_Ry = SE3_Ry(sRobot_PlanePosturePara.Pitch);
	Plane2Position = MatrixMultiply(2, Matrix_Rx, Matrix_Ry);
	free(Matrix_Rx.pMatrix);
	free(Matrix_Ry.pMatrix);
}

void Calc_Plane2Body(void)
{
	Matrix_t Matrix_Rx, Matrix_Ry, Matrix_Rz, Matrix_T;

	Matrix_T = SE3_T(sRobot_BodyPosturePara.X, sRobot_BodyPosturePara.Y, sRobot_BodyPosturePara.Z);
	Matrix_Rx = SE3_Rx(sRobot_BodyPosturePara.Roll);
	Matrix_Ry = SE3_Ry(sRobot_BodyPosturePara.Pitch);
	Matrix_Rz = SE3_Rz(sRobot_BodyPosturePara.Yaw);
	Plane2Body = MatrixMultiply(4, Matrix_T, Matrix_Rx, Matrix_Ry, Matrix_Rz);

	free(Matrix_T.pMatrix);
	free(Matrix_Rx.pMatrix);
	free(Matrix_Ry.pMatrix);
	free(Matrix_Rz.pMatrix);
}

void Modify_Posture(void)
{
	Calc_Plane2Position();
	Calc_Plane2Body();
}

SCurveSpdCtrl_t sSCurveSpdCtrlSpan_Z;
SCurveSpdCtrl_t sSCurveSpdCtrlSpan_X;
SCurvePosCtrl_t sScurvePosCtrlBody_Y;

void Modify_COG(void)
{
	switch (sRobot_MotionPara.Gait)
	{
	case 1:
		sRobot_BodyPosturePara.Y = 0;
		break;
	case 2:
		sRobot_BodyPosturePara.Y = sScurvePosCtrlBody_Y.PosOutput;

		break;
	}
}

void SCurveCtrlInit(void)
{
	SCurveSpdCtrl_New(&sSCurveSpdCtrlSpan_Z, 0.01, 20, 20, 0);
	sSCurveSpdCtrlSpan_Z.Flag = 1;
	SCurveSpdCtrl_New(&sSCurveSpdCtrlSpan_X, 0.01, 20, 20, 0);
	sSCurveSpdCtrlSpan_X.Flag = 1;
	SCurvePosCtrl_New(&sScurvePosCtrlBody_Y, 100, 0.01, sRobot_MotionPara.Cycle * sRobot_MotionPara.DutyRatio, 30, 2000, 500, 0, 80);
}

void TimeKeeping(void)
{
	static float shadow_cycle, shadow_dutyratio; //Save the copy of the value of the global varities, - sRobot_MotionPara.Cycle & sRobot_MotionPara.DutyRatio,
												 //to avoid useless accelerate calculation of SCurve when 'cycle' and 'dutyratio' aren't revised.
	static float shadow_acc;
	if (fabs(shadow_cycle - sRobot_MotionPara.Cycle) < 0.01f && fabs(shadow_dutyratio - sRobot_MotionPara.DutyRatio) < 0.01f)
	{
		/*When 'Cycle' and 'DutyRatio' aren't revised, use the old value to calc.*/
		shadow_acc = fabs(sScurvePosCtrlBody_Y.Acc);
	}
	else
	{
		/*Or use the default.*/
		shadow_acc = 10000;
	}
	shadow_cycle = sRobot_MotionPara.Cycle;
	shadow_dutyratio = sRobot_MotionPara.DutyRatio;

	sRobot_MotionPara.Time_MS = sRobot_MotionPara.Time_MS + sRobot_MotionPara.Interval;
	if (sRobot_MotionPara.Time_MS == 1000)
	{
		sRobot_MotionPara.Time_S = sRobot_MotionPara.Time_S + 1;
		sRobot_MotionPara.Time_MS = 0;
	}
	printf("T:%F\n", sRobot_MotionPara.Time_S + sRobot_MotionPara.Time_MS / 1000.0f);
	switch (sRobot_MotionPara.Gait)
	{
	case 1:

		sRobot_MotionPara.Phase_A = sRobot_MotionPara.Phase_A + sRobot_MotionPara.Interval / (1000 * sRobot_MotionPara.Cycle) * sRobot_MotionPara.PhaseTotal;
		sRobot_MotionPara.Phase_B = sRobot_MotionPara.Phase_B + sRobot_MotionPara.Interval / (1000 * sRobot_MotionPara.Cycle) * sRobot_MotionPara.PhaseTotal;

		if (sRobot_MotionPara.Phase_A > sRobot_MotionPara.PhaseTotal)
			sRobot_MotionPara.Phase_A = 0;
		if (sRobot_MotionPara.Phase_B > sRobot_MotionPara.PhaseTotal)
			sRobot_MotionPara.Phase_B = 0;

		break;
	case 2:

		sRobot_MotionPara.Phase_LF = sRobot_MotionPara.Phase_LF + sRobot_MotionPara.Interval / (1000 * sRobot_MotionPara.Cycle) * sRobot_MotionPara.PhaseTotal;
		sRobot_MotionPara.Phase_LH = sRobot_MotionPara.Phase_LH + sRobot_MotionPara.Interval / (1000 * sRobot_MotionPara.Cycle) * sRobot_MotionPara.PhaseTotal;
		sRobot_MotionPara.Phase_RF = sRobot_MotionPara.Phase_RF + sRobot_MotionPara.Interval / (1000 * sRobot_MotionPara.Cycle) * sRobot_MotionPara.PhaseTotal;
		sRobot_MotionPara.Phase_RH = sRobot_MotionPara.Phase_RH + sRobot_MotionPara.Interval / (1000 * sRobot_MotionPara.Cycle) * sRobot_MotionPara.PhaseTotal;

		if (sRobot_MotionPara.Phase_LF < sRobot_MotionPara.PhaseTotal * sRobot_MotionPara.DutyRatio && sRobot_MotionPara.Phase_RH < sRobot_MotionPara.PhaseTotal * sRobot_MotionPara.DutyRatio && sRobot_MotionPara.Phase_LH < sRobot_MotionPara.PhaseTotal * sRobot_MotionPara.DutyRatio && sRobot_MotionPara.Phase_RF < sRobot_MotionPara.PhaseTotal * sRobot_MotionPara.DutyRatio)
		{
			sRobot_MotionPara.Phase_Shift_Flag = 1;
			if ((sRobot_MotionPara.Phase_Shift_FlagOld != sRobot_MotionPara.Phase_Shift_Flag) && sRobot_MotionPara.Phase_Shift_FlagOld == 0)
			{
				if (sRobot_MotionPara.Phase_Shift_Dir == 1) //FIXME
				{
					SCurvePosCtrl_SetNewPos(&sScurvePosCtrlBody_Y, sRobot_MotionPara.Cycle * (1 - 4 * (1 - sRobot_MotionPara.DutyRatio)) / 2.0f, 200, shadow_acc, -80);
					sRobot_MotionPara.Phase_Shift_Dir = -1;
				}
				else if (sRobot_MotionPara.Phase_Shift_Dir == -1)
				{
					SCurvePosCtrl_SetNewPos(&sScurvePosCtrlBody_Y, sRobot_MotionPara.Cycle * (1 - 4 * (1 - sRobot_MotionPara.DutyRatio)) / 2.0f, 200, shadow_acc, 80);
					sRobot_MotionPara.Phase_Shift_Dir = 1;
				}
				sScurvePosCtrlBody_Y.Flag = 1;
			}
		}
		else
		{
			sScurvePosCtrlBody_Y.Flag = 0;
			sRobot_MotionPara.Phase_Shift_Flag = 0;
		}
		SCurvePosCtrl_Calc(&sScurvePosCtrlBody_Y);
		sRobot_MotionPara.Phase_Shift_FlagOld = sRobot_MotionPara.Phase_Shift_Flag;

		if (sRobot_MotionPara.Phase_LF > sRobot_MotionPara.PhaseTotal)
			sRobot_MotionPara.Phase_LF = 0;
		if (sRobot_MotionPara.Phase_LH > sRobot_MotionPara.PhaseTotal)
			sRobot_MotionPara.Phase_LH = 0;
		if (sRobot_MotionPara.Phase_RF > sRobot_MotionPara.PhaseTotal)
			sRobot_MotionPara.Phase_RF = 0;
		if (sRobot_MotionPara.Phase_RH > sRobot_MotionPara.PhaseTotal)
			sRobot_MotionPara.Phase_RH = 0;

		break;
	}
	SCurveSpdCtrl_Calc(&sSCurveSpdCtrlSpan_Z);
	SCurveSpdCtrl_Calc(&sSCurveSpdCtrlSpan_X);
}

void ParaUpdate(int mode)
{
	if (mode == 0)
	{
		if (sRobot_MotionPara.Gait != RemoteData.Gait)
		{
			sRobot_MotionPara.Gait = RemoteData.Gait;
			Init_AllPara(RemoteData.Gait);
		}
		switch (RemoteData.Gait)
		{
		case GAIT_STAND:
			switch (RemoteData.Coordinate)
			{
			case POSITION_YZPY:
				PRINTF_TIPS("STANDING. -Y -Z -Pitch -Yaw");
				sRobot_BodyPosturePara.Y = RemoteData.Joystick_LX;
				sRobot_BodyPosturePara.Z = RemoteData.Joystick_LY + sRobot_MechanicalPara.BodyHeight;
				sRobot_BodyPosturePara.Pitch = RemoteData.Joystick_RY;
				sRobot_BodyPosturePara.Yaw = RemoteData.Joystick_RX;
				break;
			case POSITION_XZRY:
				PRINTF_TIPS("STANDING. -X -Z -Roll -Yaw");
				sRobot_BodyPosturePara.X = RemoteData.Joystick_LX;
				sRobot_BodyPosturePara.Z = RemoteData.Joystick_LY + sRobot_MechanicalPara.BodyHeight;
				sRobot_BodyPosturePara.Roll = RemoteData.Joystick_RY;
				sRobot_BodyPosturePara.Yaw = RemoteData.Joystick_RX;
				break;
			case UNIVERSE_XYPY:
				PRINTF_TIPS("STANDING. Auto -Roll -Pitch");
				//sRobot_BodyPosturePara.Roll = -GyroscopeData[5].fl;
				//sRobot_BodyPosturePara.Pitch = -GyroscopeData[6].fl;
				//printf("\tRoll:%f\n", GyroscopeData[5].fl);
				//printf("\tPitch:%f\n", GyroscopeData[6].fl);
				break;
			}
			break;

		case GAIT_WALK:
			switch (RemoteData.Coordinate)
			{
			case UNIVERSE_XYPY:
				SCurveCtrl_SetNewSpd(&sSCurveSpdCtrlSpan_Z, RemoteData.Joystick_LY);
				SCurveCtrl_SetNewSpd(&sSCurveSpdCtrlSpan_X, RemoteData.Joystick_LX);
				sRobot_MotionPara.Span_Z = sSCurveSpdCtrlSpan_Z.SpdOutput;
				sRobot_MotionPara.Span_X = -sSCurveSpdCtrlSpan_X.SpdOutput;
				sRobot_MotionPara.Span_W = -RemoteData.Joystick_RX / 180.0 * PI / 8;

				//	sRobot_PlanePosturePara.Roll = GyroscopeData[5].fl;
				//	sRobot_PlanePosturePara.Pitch = GyroscopeData[6].fl;
				sRobot_BodyPosturePara.Roll = sRobot_PlanePosturePara.Roll;
				sRobot_BodyPosturePara.Pitch = sRobot_PlanePosturePara.Pitch;

				if ((sRobot_MotionPara.Interval / (1000 * sRobot_MotionPara.Cycle * (1 - 4 * (1 - (0.75 + ((float)RemoteData.Dial / 1024)))) / 2) * PI) < 0.4)
					sRobot_MotionPara.Cycle = 0.4 + ((float)RemoteData.Dial / 128);
				PRINTF_TIPS("WALKING. Cycle:%f\n", sRobot_MotionPara.Cycle);
				Revise_WalkPhase();
				break;
			case POSITION_XZRY:
				if ((sRobot_MotionPara.Interval / (1000 * sRobot_MotionPara.Cycle * (1 - 4 * (1 - (0.75 + ((float)RemoteData.Dial / 1024)))) / 2) * PI) < 0.4)
					sRobot_MotionPara.DutyRatio = 0.75 + ((float)RemoteData.Dial / 1024);
				PRINTF_TIPS("WALKING. DutyRatio:%f\n", sRobot_MotionPara.DutyRatio);
				Revise_WalkPhase();
				break;
			case POSITION_YZPY:
				sRobot_MotionPara.Span_Y = 0 + 150 * ((float)RemoteData.Dial / 512);
				PRINTF_TIPS("WALKING. Span_Y:%d\n", sRobot_MotionPara.Span_Y);
				break;
			}
			break;

		case GAIT_TROT:
			switch (RemoteData.Coordinate)
			{
			case UNIVERSE_XYPY:
				SCurveCtrl_SetNewSpd(&sSCurveSpdCtrlSpan_Z, RemoteData.Joystick_LY);
				sRobot_MotionPara.Span_Z = sSCurveSpdCtrlSpan_Z.SpdOutput;
				SCurveCtrl_SetNewSpd(&sSCurveSpdCtrlSpan_X, RemoteData.Joystick_LX);
				sRobot_MotionPara.Span_X = -sSCurveSpdCtrlSpan_X.SpdOutput;
				sRobot_MotionPara.Span_W = -RemoteData.Joystick_RX / 180.0 * PI / 8;

				//sRobot_PlanePosturePara.Roll = GyroscopeData[5].fl;
				//sRobot_PlanePosturePara.Pitch = GyroscopeData[6].fl;
				sRobot_BodyPosturePara.Roll = sRobot_PlanePosturePara.Roll;
				sRobot_BodyPosturePara.Pitch = sRobot_PlanePosturePara.Pitch;

				sRobot_MotionPara.Cycle = 0.4 + ((float)RemoteData.Dial / 512);
				PRINTF_TIPS("TROTTING. Cycle:%f\n", sRobot_MotionPara.Cycle);
				break;
			case POSITION_XZRY:
				sRobot_MotionPara.DutyRatio = 0.5 + ((float)RemoteData.Dial / 512);
				PRINTF_TIPS("TROTTING. DutyRatio:%f\n", sRobot_MotionPara.DutyRatio);
				break;
			case POSITION_YZPY:
				sRobot_MotionPara.Span_Y = 0 + 150 * ((float)RemoteData.Dial / 512);
				PRINTF_TIPS("TROTTING. Span_Y:%d\n", sRobot_MotionPara.Span_Y);
				break;
			}
			break;
		}
	}
}
