#include "gait_plan.h"

//机器人结构矩阵
extern Robot_MatrixGroup_t Body2Leg;
extern Robot_MatrixGroup_t Position2Zero;

//机器人变换矩阵
extern Matrix_t Position2Body;
extern Matrix_t Plane2Body;
extern Matrix_t Plane2Position;

Matrix_t Zero2Supporting_Ending;
Matrix_t Zero2Swing_Ending;
Robot_MatrixGroup_t Body2Supporting_Ending;
Robot_MatrixGroup_t Body2Swing_Ending;
Robot_MatrixGroup_t Position2Supporting_Ending;
Robot_MatrixGroup_t Position2Swing_Ending;
Robot_MatrixGroup_t Plane2Supporting_Ending;
Robot_MatrixGroup_t Plane2Swing_Ending;
Robot_MatrixGroup_t Leg2Supporting_Ending;
Robot_MatrixGroup_t Leg2Swing_Ending;

//足末端点轨迹规划系数矩阵
Matrix_t Coeff_X, Coeff_Y, Coeff_W;

//Trot用全局变量
float SwingTime_Trot, SupportingTime_Trot;
float SwingTime_Walk, SupportingTime_Walk;

void Calc_GaitTrajPolyCoeffi_Trot(void)
{
	Matrix_t InverseMatrix, TrajectoryPlanningMatrix, TrajectoryPlanningParameterMatrix;

	SupportingTime_Trot = sRobot_MotionPara.Cycle * sRobot_MotionPara.DutyRatio;
	SwingTime_Trot = sRobot_MotionPara.Cycle * (1 - sRobot_MotionPara.DutyRatio);

	TrajectoryPlanningMatrix = Traj3_Mat(SwingTime_Trot);
	InverseMatrix = Inv4(TrajectoryPlanningMatrix);
	TrajectoryPlanningParameterMatrix = Traj3_Para_Mat(1 - 4 * sRobot_MotionPara.DutyRatio, -4, 1, -4);

	if (Coeff_X.pMatrix != NULL && Coeff_Y.pMatrix != NULL && Coeff_W.pMatrix != NULL)
	{
		free(Coeff_X.pMatrix);
		free(Coeff_Y.pMatrix);
		free(Coeff_W.pMatrix);
	}

	Coeff_X = MatrixMultiply(2, InverseMatrix, TrajectoryPlanningParameterMatrix);
	Coeff_Y = MatrixMultiply(2, InverseMatrix, TrajectoryPlanningParameterMatrix);
	Coeff_W = MatrixMultiply(2, InverseMatrix, TrajectoryPlanningParameterMatrix);

	free(TrajectoryPlanningMatrix.pMatrix);
	free(InverseMatrix.pMatrix);
	free(TrajectoryPlanningParameterMatrix.pMatrix);
}

void Calc_GaitTrajPolyCoeffi_Walk(void)
{
	Matrix_t InverseMatrix, TrajectoryPlanningMatrix, TrajectoryPlanningParameterMatrix;

	SupportingTime_Walk = sRobot_MotionPara.Cycle * sRobot_MotionPara.DutyRatio;
	SwingTime_Walk = sRobot_MotionPara.Cycle * (1 - sRobot_MotionPara.DutyRatio);
	TrajectoryPlanningMatrix = Traj3_Mat(SwingTime_Walk);
	InverseMatrix = Inv4(TrajectoryPlanningMatrix);
	TrajectoryPlanningParameterMatrix = Traj3_Para_Mat(1 - 4 * sRobot_MotionPara.DutyRatio, -4, 1, -4);

	if (Coeff_X.pMatrix != NULL && Coeff_Y.pMatrix != NULL && Coeff_W.pMatrix != NULL)
	{
		free(Coeff_X.pMatrix);
		free(Coeff_Y.pMatrix);
		free(Coeff_W.pMatrix);
	}

	Coeff_X = MatrixMultiply(2, InverseMatrix, TrajectoryPlanningParameterMatrix);
	Coeff_Y = MatrixMultiply(2, InverseMatrix, TrajectoryPlanningParameterMatrix);
	Coeff_W = MatrixMultiply(2, InverseMatrix, TrajectoryPlanningParameterMatrix);

	free(TrajectoryPlanningMatrix.pMatrix);
	free(InverseMatrix.pMatrix);
	free(TrajectoryPlanningParameterMatrix.pMatrix);
}

Matrix_t Calc_GaitTrajPoint_Trot(void)
{
	float t;
	float *ResultArray = (float *)malloc(sizeof(float) * 4 * 3);
	float Zero2Supporting_Ending_PosX, Zero2Supporting_Ending_PosZ, Zero2Supporting_Ending_PosY;
	float Zero2Swing_Ending_PosX, Zero2Swing_Ending_PosZ, Zero2Swing_Ending_PosY;
	float Theta_Supporting, Theta_Swing;
	Matrix_t MatrixRotation_Z, InverseMatrix;

	t = sRobot_MotionPara.Phase_A / sRobot_MotionPara.PhaseTotal * sRobot_MotionPara.Cycle;
	if (sRobot_MotionPara.Phase_A < sRobot_MotionPara.PhaseTotal * sRobot_MotionPara.DutyRatio)
	{
		Zero2Supporting_Ending_PosZ = sRobot_MotionPara.Span_Z / 2 * (1 - 4 * t / sRobot_MotionPara.Cycle);
		Zero2Supporting_Ending_PosX = sRobot_MotionPara.Span_X / 2 * (1 - 4 * t / sRobot_MotionPara.Cycle);
		Zero2Supporting_Ending_PosY = 0 * t;
		Theta_Supporting = sRobot_MotionPara.Span_W * (1 - 4 * t / sRobot_MotionPara.Cycle);

		MatrixRotation_Z = SE3_Rz(Theta_Supporting / PI * 180);
		InverseMatrix = Inv4(Plane2Body);

		Zero2Supporting_Ending = SE3_T(Zero2Supporting_Ending_PosX, Zero2Supporting_Ending_PosY, Zero2Supporting_Ending_PosZ);
		Position2Supporting_Ending.LF = MatrixMultiply(2, Position2Zero.LF, Zero2Supporting_Ending);
		Plane2Supporting_Ending.LF = MatrixMultiply(2, Plane2Position, Position2Supporting_Ending.LF);
		Body2Supporting_Ending.LF = MatrixMultiply(3, MatrixRotation_Z, InverseMatrix, Plane2Supporting_Ending.LF);
		free(Zero2Supporting_Ending.pMatrix);
		free(Position2Supporting_Ending.LF.pMatrix);
		free(Plane2Supporting_Ending.LF.pMatrix);

		Zero2Supporting_Ending = SE3_T(-Zero2Supporting_Ending_PosX, Zero2Supporting_Ending_PosY, -Zero2Supporting_Ending_PosZ);
		Position2Supporting_Ending.RH = MatrixMultiply(2, Position2Zero.RH, Zero2Supporting_Ending);
		Plane2Supporting_Ending.RH = MatrixMultiply(2, Plane2Position, Position2Supporting_Ending.RH);
		Body2Supporting_Ending.RH = MatrixMultiply(3, MatrixRotation_Z, InverseMatrix, Plane2Supporting_Ending.RH);
		free(Zero2Supporting_Ending.pMatrix);
		free(Position2Supporting_Ending.RH.pMatrix);
		free(Plane2Supporting_Ending.RH.pMatrix);

		free(MatrixRotation_Z.pMatrix);
		free(InverseMatrix.pMatrix);

		InverseMatrix = Inv4(Body2Leg.LF);
		Leg2Supporting_Ending.LF = MatrixMultiply(2, InverseMatrix, Body2Supporting_Ending.LF);
		free(InverseMatrix.pMatrix);
		free(Body2Supporting_Ending.LF.pMatrix);
		InverseMatrix = Inv4(Body2Leg.RH);
		Leg2Supporting_Ending.RH = MatrixMultiply(2, InverseMatrix, Body2Supporting_Ending.RH);
		free(InverseMatrix.pMatrix);
		free(Body2Supporting_Ending.RH.pMatrix);

		*(ResultArray + 0 * 3 + 0) = *(Leg2Supporting_Ending.LF.pMatrix + 0 * 4 + 3);
		*(ResultArray + 0 * 3 + 1) = *(Leg2Supporting_Ending.LF.pMatrix + 1 * 4 + 3);
		*(ResultArray + 0 * 3 + 2) = *(Leg2Supporting_Ending.LF.pMatrix + 2 * 4 + 3);
		*(ResultArray + 1 * 3 + 0) = *(Leg2Supporting_Ending.RH.pMatrix + 0 * 4 + 3);
		*(ResultArray + 1 * 3 + 1) = *(Leg2Supporting_Ending.RH.pMatrix + 1 * 4 + 3);
		*(ResultArray + 1 * 3 + 2) = *(Leg2Supporting_Ending.RH.pMatrix + 2 * 4 + 3);

		free(Leg2Supporting_Ending.LF.pMatrix);
		free(Leg2Supporting_Ending.RH.pMatrix);
	}
	else
	{
		Zero2Swing_Ending_PosZ = sRobot_MotionPara.Span_Z / 2 * polyval(Coeff_X.pMatrix, 4, (t - SupportingTime_Trot));
		Zero2Swing_Ending_PosX = sRobot_MotionPara.Span_X / 2 * polyval(Coeff_Y.pMatrix, 4, (t - SupportingTime_Trot));
		Zero2Swing_Ending_PosY = sRobot_MotionPara.Span_Y * (1 - cos(2 * PI * (t - SupportingTime_Trot) / SwingTime_Trot)) / 2;
		Theta_Swing = sRobot_MotionPara.Span_W * polyval(Coeff_W.pMatrix, 4, (t - SupportingTime_Trot));

		MatrixRotation_Z = SE3_Rz(Theta_Swing / PI * 180);
		InverseMatrix = Inv4(Plane2Body);

		Zero2Swing_Ending = SE3_T(Zero2Swing_Ending_PosX, Zero2Swing_Ending_PosY, Zero2Swing_Ending_PosZ);
		Position2Swing_Ending.LF = MatrixMultiply(2, Position2Zero.LF, Zero2Swing_Ending);
		Plane2Swing_Ending.LF = MatrixMultiply(2, Plane2Position, Position2Swing_Ending.LF);
		Body2Swing_Ending.LF = MatrixMultiply(3, MatrixRotation_Z, InverseMatrix, Plane2Swing_Ending.LF);
		free(Zero2Swing_Ending.pMatrix);
		free(Position2Swing_Ending.LF.pMatrix);
		free(Plane2Swing_Ending.LF.pMatrix);

		Zero2Swing_Ending = SE3_T(-Zero2Swing_Ending_PosX, Zero2Swing_Ending_PosY, -Zero2Swing_Ending_PosZ);
		Position2Swing_Ending.RH = MatrixMultiply(2, Position2Zero.RH, Zero2Swing_Ending);
		Plane2Swing_Ending.RH = MatrixMultiply(2, Plane2Position, Position2Swing_Ending.RH);
		Body2Swing_Ending.RH = MatrixMultiply(3, MatrixRotation_Z, InverseMatrix, Plane2Swing_Ending.RH);
		free(Zero2Swing_Ending.pMatrix);
		free(Position2Swing_Ending.RH.pMatrix);
		free(Plane2Swing_Ending.RH.pMatrix);

		free(MatrixRotation_Z.pMatrix);
		free(InverseMatrix.pMatrix);

		InverseMatrix = Inv4(Body2Leg.LF);
		Leg2Swing_Ending.LF = MatrixMultiply(2, InverseMatrix, Body2Swing_Ending.LF);
		free(InverseMatrix.pMatrix);
		free(Body2Swing_Ending.LF.pMatrix);
		InverseMatrix = Inv4(Body2Leg.RH);
		Leg2Swing_Ending.RH = MatrixMultiply(2, InverseMatrix, Body2Swing_Ending.RH);
		free(InverseMatrix.pMatrix);
		free(Body2Swing_Ending.RH.pMatrix);

		*(ResultArray + 0 * 3 + 0) = *(Leg2Swing_Ending.LF.pMatrix + 0 * 4 + 3);
		*(ResultArray + 0 * 3 + 1) = *(Leg2Swing_Ending.LF.pMatrix + 1 * 4 + 3);
		*(ResultArray + 0 * 3 + 2) = *(Leg2Swing_Ending.LF.pMatrix + 2 * 4 + 3);
		*(ResultArray + 1 * 3 + 0) = *(Leg2Swing_Ending.RH.pMatrix + 0 * 4 + 3);
		*(ResultArray + 1 * 3 + 1) = *(Leg2Swing_Ending.RH.pMatrix + 1 * 4 + 3);
		*(ResultArray + 1 * 3 + 2) = *(Leg2Swing_Ending.RH.pMatrix + 2 * 4 + 3);

		free(Leg2Swing_Ending.LF.pMatrix);
		free(Leg2Swing_Ending.RH.pMatrix);
	}

	t = sRobot_MotionPara.Phase_B / sRobot_MotionPara.PhaseTotal * sRobot_MotionPara.Cycle;
	if (sRobot_MotionPara.Phase_B < sRobot_MotionPara.PhaseTotal * sRobot_MotionPara.DutyRatio)
	{
		Zero2Supporting_Ending_PosZ = sRobot_MotionPara.Span_Z / 2 * (1 - 4 * t / sRobot_MotionPara.Cycle);
		Zero2Supporting_Ending_PosX = sRobot_MotionPara.Span_X / 2 * (1 - 4 * t / sRobot_MotionPara.Cycle);
		Zero2Supporting_Ending_PosY = 0 * t;
		Theta_Supporting = sRobot_MotionPara.Span_W * (1 - 4 * t / sRobot_MotionPara.Cycle);

		MatrixRotation_Z = SE3_Rz(Theta_Supporting / PI * 180);
		InverseMatrix = Inv4(Plane2Body);

		Zero2Supporting_Ending = SE3_T(Zero2Supporting_Ending_PosX, -Zero2Supporting_Ending_PosY, -Zero2Supporting_Ending_PosZ);
		Position2Supporting_Ending.LH = MatrixMultiply(2, Position2Zero.LH, Zero2Supporting_Ending);
		Plane2Supporting_Ending.LH = MatrixMultiply(2, Plane2Position, Position2Supporting_Ending.LH);
		Body2Supporting_Ending.LH = MatrixMultiply(3, MatrixRotation_Z, InverseMatrix, Plane2Supporting_Ending.LH);
		free(Zero2Supporting_Ending.pMatrix);
		free(Position2Supporting_Ending.LH.pMatrix);
		free(Plane2Supporting_Ending.LH.pMatrix);

		Zero2Supporting_Ending = SE3_T(-Zero2Supporting_Ending_PosX, -Zero2Supporting_Ending_PosY, Zero2Supporting_Ending_PosZ);
		Position2Supporting_Ending.RF = MatrixMultiply(2, Position2Zero.RF, Zero2Supporting_Ending);
		Plane2Supporting_Ending.RF = MatrixMultiply(2, Plane2Position, Position2Supporting_Ending.RF);
		Body2Supporting_Ending.RF = MatrixMultiply(3, MatrixRotation_Z, InverseMatrix, Plane2Supporting_Ending.RF);
		free(Zero2Supporting_Ending.pMatrix);
		free(Position2Supporting_Ending.RF.pMatrix);
		free(Plane2Supporting_Ending.RF.pMatrix);

		free(MatrixRotation_Z.pMatrix);
		free(InverseMatrix.pMatrix);

		InverseMatrix = Inv4(Body2Leg.LH);
		Leg2Supporting_Ending.LH = MatrixMultiply(2, InverseMatrix, Body2Supporting_Ending.LH);
		free(InverseMatrix.pMatrix);
		free(Body2Supporting_Ending.LH.pMatrix);
		InverseMatrix = Inv4(Body2Leg.RF);
		Leg2Supporting_Ending.RF = MatrixMultiply(2, InverseMatrix, Body2Supporting_Ending.RF);
		free(InverseMatrix.pMatrix);
		free(Body2Supporting_Ending.RF.pMatrix);

		*(ResultArray + 2 * 3 + 0) = *(Leg2Supporting_Ending.RF.pMatrix + 0 * 4 + 3);
		*(ResultArray + 2 * 3 + 1) = *(Leg2Supporting_Ending.RF.pMatrix + 1 * 4 + 3);
		*(ResultArray + 2 * 3 + 2) = *(Leg2Supporting_Ending.RF.pMatrix + 2 * 4 + 3);
		*(ResultArray + 3 * 3 + 0) = *(Leg2Supporting_Ending.LH.pMatrix + 0 * 4 + 3);
		*(ResultArray + 3 * 3 + 1) = *(Leg2Supporting_Ending.LH.pMatrix + 1 * 4 + 3);
		*(ResultArray + 3 * 3 + 2) = *(Leg2Supporting_Ending.LH.pMatrix + 2 * 4 + 3);

		free(Leg2Supporting_Ending.RF.pMatrix);
		free(Leg2Supporting_Ending.LH.pMatrix);
	}
	else
	{
		Zero2Swing_Ending_PosZ = sRobot_MotionPara.Span_Z / 2 * polyval(Coeff_X.pMatrix, 4, (t - SupportingTime_Trot));
		Zero2Swing_Ending_PosX = sRobot_MotionPara.Span_X / 2 * polyval(Coeff_Y.pMatrix, 4, (t - SupportingTime_Trot));
		Zero2Swing_Ending_PosY = sRobot_MotionPara.Span_Y * (1 - cos(2 * PI * (t - SupportingTime_Trot) / SwingTime_Trot)) / 2;
		Theta_Swing = sRobot_MotionPara.Span_W * polyval(Coeff_W.pMatrix, 4, (t - SupportingTime_Trot));

		MatrixRotation_Z = SE3_Rz(Theta_Swing / PI * 180);
		InverseMatrix = Inv4(Plane2Body);

		Zero2Swing_Ending = SE3_T(Zero2Swing_Ending_PosX, -Zero2Swing_Ending_PosY, -Zero2Swing_Ending_PosZ);
		Position2Swing_Ending.LH = MatrixMultiply(2, Position2Zero.LH, Zero2Swing_Ending);
		Plane2Swing_Ending.LH = MatrixMultiply(2, Plane2Position, Position2Swing_Ending.LH);
		Body2Swing_Ending.LH = MatrixMultiply(3, MatrixRotation_Z, InverseMatrix, Plane2Swing_Ending.LH);
		free(Zero2Swing_Ending.pMatrix);
		free(Position2Swing_Ending.LH.pMatrix);
		free(Plane2Swing_Ending.LH.pMatrix);

		Zero2Swing_Ending = SE3_T(-Zero2Swing_Ending_PosX, -Zero2Swing_Ending_PosY, Zero2Swing_Ending_PosZ);
		Position2Swing_Ending.RF = MatrixMultiply(2, Position2Zero.RF, Zero2Swing_Ending);
		Plane2Swing_Ending.RF = MatrixMultiply(2, Plane2Position, Position2Swing_Ending.RF);
		Body2Swing_Ending.RF = MatrixMultiply(3, MatrixRotation_Z, InverseMatrix, Plane2Swing_Ending.RF);
		free(Zero2Swing_Ending.pMatrix);
		free(Position2Swing_Ending.RF.pMatrix);
		free(Plane2Swing_Ending.RF.pMatrix);

		free(MatrixRotation_Z.pMatrix);
		free(InverseMatrix.pMatrix);

		InverseMatrix = Inv4(Body2Leg.LH);
		Leg2Swing_Ending.LH = MatrixMultiply(2, InverseMatrix, Body2Swing_Ending.LH);
		free(InverseMatrix.pMatrix);
		free(Body2Swing_Ending.LH.pMatrix);
		InverseMatrix = Inv4(Body2Leg.RF);
		Leg2Swing_Ending.RF = MatrixMultiply(2, InverseMatrix, Body2Swing_Ending.RF);
		free(InverseMatrix.pMatrix);
		free(Body2Swing_Ending.RF.pMatrix);

		*(ResultArray + 2 * 3 + 0) = *(Leg2Swing_Ending.RF.pMatrix + 0 * 4 + 3);
		*(ResultArray + 2 * 3 + 1) = *(Leg2Swing_Ending.RF.pMatrix + 1 * 4 + 3);
		*(ResultArray + 2 * 3 + 2) = *(Leg2Swing_Ending.RF.pMatrix + 2 * 4 + 3);
		*(ResultArray + 3 * 3 + 0) = *(Leg2Swing_Ending.LH.pMatrix + 0 * 4 + 3);
		*(ResultArray + 3 * 3 + 1) = *(Leg2Swing_Ending.LH.pMatrix + 1 * 4 + 3);
		*(ResultArray + 3 * 3 + 2) = *(Leg2Swing_Ending.LH.pMatrix + 2 * 4 + 3);

		free(Leg2Swing_Ending.RF.pMatrix);
		free(Leg2Swing_Ending.LH.pMatrix);
	}

	return Arr2Mat(ResultArray, 4, 3);
}

Matrix_t Calc_GaitTrajPoint_Walk(void)
{
	float t;
	float *ResultArray = (float *)malloc(sizeof(float) * 4 * 3);
	float Zero2Supporting_Ending_PosX, Zero2Supporting_Ending_PosZ, Zero2Supporting_Ending_PosY;
	float Zero2Swing_Ending_PosX, Zero2Swing_Ending_PosZ, Zero2Swing_Ending_PosY;
	float Theta_Supporting, Theta_Swing;
	Matrix_t MatrixRotation_Z, InverseMatrix;

	t = sRobot_MotionPara.Phase_LF / sRobot_MotionPara.PhaseTotal * sRobot_MotionPara.Cycle;
	if (sRobot_MotionPara.Phase_LF < sRobot_MotionPara.PhaseTotal * sRobot_MotionPara.DutyRatio)
	{
		Zero2Supporting_Ending_PosZ = sRobot_MotionPara.Span_Z / 2 * (1 - 4 * t / sRobot_MotionPara.Cycle);
		Zero2Supporting_Ending_PosX = sRobot_MotionPara.Span_X / 2 * (1 - 4 * t / sRobot_MotionPara.Cycle);
		Zero2Supporting_Ending_PosY = 0 * t;
		Theta_Supporting = sRobot_MotionPara.Span_W * (1 - 4 * t / sRobot_MotionPara.Cycle);

		MatrixRotation_Z = SE3_Rz(Theta_Supporting / PI * 180);
		InverseMatrix = Inv4(Plane2Body);

		//printf("Debug:Gait.c 2.0\n");
		Zero2Supporting_Ending = SE3_T(Zero2Supporting_Ending_PosX, Zero2Supporting_Ending_PosY, Zero2Supporting_Ending_PosZ);
		Position2Supporting_Ending.LF = MatrixMultiply(2, Position2Zero.LF, Zero2Supporting_Ending);
		Plane2Supporting_Ending.LF = MatrixMultiply(2, Plane2Position, Position2Supporting_Ending.LF);
		Body2Supporting_Ending.LF = MatrixMultiply(3, MatrixRotation_Z, InverseMatrix, Plane2Supporting_Ending.LF);
		free(Zero2Supporting_Ending.pMatrix);
		free(Position2Supporting_Ending.LF.pMatrix);
		free(Plane2Supporting_Ending.LF.pMatrix);

		free(MatrixRotation_Z.pMatrix);
		free(InverseMatrix.pMatrix);

		InverseMatrix = Inv4(Body2Leg.LF);
		Leg2Supporting_Ending.LF = MatrixMultiply(2, InverseMatrix, Body2Supporting_Ending.LF);
		free(InverseMatrix.pMatrix);
		free(Body2Supporting_Ending.LF.pMatrix);

		*(ResultArray + 0 * 3 + 0) = *(Leg2Supporting_Ending.LF.pMatrix + 0 * 4 + 3);
		*(ResultArray + 0 * 3 + 1) = *(Leg2Supporting_Ending.LF.pMatrix + 1 * 4 + 3);
		*(ResultArray + 0 * 3 + 2) = *(Leg2Supporting_Ending.LF.pMatrix + 2 * 4 + 3);

		free(Leg2Supporting_Ending.LF.pMatrix);
	}
	else
	{
		Zero2Swing_Ending_PosZ = sRobot_MotionPara.Span_Z / 2 * polyval(Coeff_X.pMatrix, 4, (t - SupportingTime_Walk));
		Zero2Swing_Ending_PosX = sRobot_MotionPara.Span_X / 2 * polyval(Coeff_Y.pMatrix, 4, (t - SupportingTime_Walk));
		Zero2Swing_Ending_PosY = sRobot_MotionPara.Span_Y * (1 - cos(2 * PI * (t - SupportingTime_Walk) / SwingTime_Walk)) / 2;
		Theta_Swing = sRobot_MotionPara.Span_W * polyval(Coeff_W.pMatrix, 4, (t - SupportingTime_Walk));

		MatrixRotation_Z = SE3_Rz(Theta_Swing / PI * 180);
		InverseMatrix = Inv4(Plane2Body);

		//printf("Debug:Gait.c 2.1\n");
		Zero2Swing_Ending = SE3_T(Zero2Swing_Ending_PosX, Zero2Swing_Ending_PosY, Zero2Swing_Ending_PosZ);
		Position2Swing_Ending.LF = MatrixMultiply(2, Position2Zero.LF, Zero2Swing_Ending);
		Plane2Swing_Ending.LF = MatrixMultiply(2, Plane2Position, Position2Swing_Ending.LF);
		Body2Swing_Ending.LF = MatrixMultiply(3, MatrixRotation_Z, InverseMatrix, Plane2Swing_Ending.LF);
		free(Zero2Swing_Ending.pMatrix);
		free(Position2Swing_Ending.LF.pMatrix);
		free(Plane2Swing_Ending.LF.pMatrix);

		free(MatrixRotation_Z.pMatrix);
		free(InverseMatrix.pMatrix);

		InverseMatrix = Inv4(Body2Leg.LF);
		Leg2Swing_Ending.LF = MatrixMultiply(2, InverseMatrix, Body2Swing_Ending.LF);
		free(InverseMatrix.pMatrix);
		free(Body2Swing_Ending.LF.pMatrix);

		*(ResultArray + 0 * 3 + 0) = *(Leg2Swing_Ending.LF.pMatrix + 0 * 4 + 3);
		*(ResultArray + 0 * 3 + 1) = *(Leg2Swing_Ending.LF.pMatrix + 1 * 4 + 3);
		*(ResultArray + 0 * 3 + 2) = *(Leg2Swing_Ending.LF.pMatrix + 2 * 4 + 3);

		free(Leg2Swing_Ending.LF.pMatrix);
	}

	t = sRobot_MotionPara.Phase_LH / sRobot_MotionPara.PhaseTotal * sRobot_MotionPara.Cycle;
	if (sRobot_MotionPara.Phase_LH < sRobot_MotionPara.PhaseTotal * sRobot_MotionPara.DutyRatio)
	{
		Zero2Supporting_Ending_PosZ = sRobot_MotionPara.Span_Z / 2 * (1 - 4 * t / sRobot_MotionPara.Cycle);
		Zero2Supporting_Ending_PosX = sRobot_MotionPara.Span_X / 2 * (1 - 4 * t / sRobot_MotionPara.Cycle);
		Zero2Supporting_Ending_PosY = 0 * t;
		Theta_Supporting = sRobot_MotionPara.Span_W * (1 - 4 * t / sRobot_MotionPara.Cycle);

		MatrixRotation_Z = SE3_Rz(Theta_Supporting / PI * 180);
		InverseMatrix = Inv4(Plane2Body);

		//printf("Debug:Gait.c 2.2\n");
		Zero2Supporting_Ending = SE3_T(Zero2Supporting_Ending_PosX, -Zero2Supporting_Ending_PosY, -Zero2Supporting_Ending_PosZ);
		Position2Supporting_Ending.LH = MatrixMultiply(2, Position2Zero.LH, Zero2Supporting_Ending);
		Plane2Supporting_Ending.LH = MatrixMultiply(2, Plane2Position, Position2Supporting_Ending.LH);
		Body2Supporting_Ending.LH = MatrixMultiply(3, MatrixRotation_Z, InverseMatrix, Plane2Supporting_Ending.LH);
		free(Zero2Supporting_Ending.pMatrix);
		free(Position2Supporting_Ending.LH.pMatrix);
		free(Plane2Supporting_Ending.LH.pMatrix);

		//printf("Debug:Gait.c 2.2.1\n");
		free(MatrixRotation_Z.pMatrix);
		free(InverseMatrix.pMatrix);

		//printf("Debug:Gait.c 2.2.2\n");
		InverseMatrix = Inv4(Body2Leg.LH);
		Leg2Supporting_Ending.LH = MatrixMultiply(2, InverseMatrix, Body2Supporting_Ending.LH);
		free(InverseMatrix.pMatrix);
		free(Body2Supporting_Ending.LH.pMatrix);

		//printf("Debug:Gait.c 2.2.3\n");
		*(ResultArray + 3 * 3 + 0) = *(Leg2Supporting_Ending.LH.pMatrix + 0 * 4 + 3);
		*(ResultArray + 3 * 3 + 1) = *(Leg2Supporting_Ending.LH.pMatrix + 1 * 4 + 3);
		*(ResultArray + 3 * 3 + 2) = *(Leg2Supporting_Ending.LH.pMatrix + 2 * 4 + 3);
		//printf("Debug:Gait.c 2.2.4\n");

		free(Leg2Supporting_Ending.LH.pMatrix);
	}
	else
	{
		Zero2Swing_Ending_PosZ = sRobot_MotionPara.Span_Z / 2 * polyval(Coeff_X.pMatrix, 4, (t - SupportingTime_Walk));
		Zero2Swing_Ending_PosX = sRobot_MotionPara.Span_X / 2 * polyval(Coeff_Y.pMatrix, 4, (t - SupportingTime_Walk));
		Zero2Swing_Ending_PosY = sRobot_MotionPara.Span_Y * (1 - cos(2 * PI * (t - SupportingTime_Walk) / SwingTime_Walk)) / 2;
		Theta_Swing = sRobot_MotionPara.Span_W * polyval(Coeff_W.pMatrix, 4, (t - SupportingTime_Walk));

		MatrixRotation_Z = SE3_Rz(Theta_Swing / PI * 180);
		InverseMatrix = Inv4(Plane2Body);

		//printf("Debug:Gait.c 2.3\n");
		Zero2Swing_Ending = SE3_T(Zero2Swing_Ending_PosX, -Zero2Swing_Ending_PosY, -Zero2Swing_Ending_PosZ);
		Position2Swing_Ending.LH = MatrixMultiply(2, Position2Zero.LH, Zero2Swing_Ending);
		Plane2Swing_Ending.LH = MatrixMultiply(2, Plane2Position, Position2Swing_Ending.LH);
		Body2Swing_Ending.LH = MatrixMultiply(3, MatrixRotation_Z, InverseMatrix, Plane2Swing_Ending.LH);
		free(Zero2Swing_Ending.pMatrix);
		free(Position2Swing_Ending.LH.pMatrix);
		free(Plane2Swing_Ending.LH.pMatrix);

		free(MatrixRotation_Z.pMatrix);
		free(InverseMatrix.pMatrix);

		InverseMatrix = Inv4(Body2Leg.LH);
		Leg2Swing_Ending.LH = MatrixMultiply(2, InverseMatrix, Body2Swing_Ending.LH);
		free(InverseMatrix.pMatrix);
		free(Body2Swing_Ending.LH.pMatrix);

		*(ResultArray + 3 * 3 + 0) = *(Leg2Swing_Ending.LH.pMatrix + 0 * 4 + 3);
		*(ResultArray + 3 * 3 + 1) = *(Leg2Swing_Ending.LH.pMatrix + 1 * 4 + 3);
		*(ResultArray + 3 * 3 + 2) = *(Leg2Swing_Ending.LH.pMatrix + 2 * 4 + 3);

		free(Leg2Swing_Ending.LH.pMatrix);
	}

	t = sRobot_MotionPara.Phase_RH / sRobot_MotionPara.PhaseTotal * sRobot_MotionPara.Cycle;
	if (sRobot_MotionPara.Phase_RH < sRobot_MotionPara.PhaseTotal * sRobot_MotionPara.DutyRatio)
	{
		Zero2Supporting_Ending_PosZ = sRobot_MotionPara.Span_Z / 2 * (1 - 4 * t / sRobot_MotionPara.Cycle);
		Zero2Supporting_Ending_PosX = sRobot_MotionPara.Span_X / 2 * (1 - 4 * t / sRobot_MotionPara.Cycle);
		Zero2Supporting_Ending_PosY = 0 * t;
		Theta_Supporting = sRobot_MotionPara.Span_W * (1 - 4 * t / sRobot_MotionPara.Cycle);

		MatrixRotation_Z = SE3_Rz(Theta_Supporting / PI * 180);
		InverseMatrix = Inv4(Plane2Body);

		//printf("Debug:Gait.c 2.4\n");
		Zero2Supporting_Ending = SE3_T(-Zero2Supporting_Ending_PosX, Zero2Supporting_Ending_PosY, -Zero2Supporting_Ending_PosZ);
		Position2Supporting_Ending.RH = MatrixMultiply(2, Position2Zero.RH, Zero2Supporting_Ending);
		Plane2Supporting_Ending.RH = MatrixMultiply(2, Plane2Position, Position2Supporting_Ending.RH);
		Body2Supporting_Ending.RH = MatrixMultiply(3, MatrixRotation_Z, InverseMatrix, Plane2Supporting_Ending.RH);
		free(Zero2Supporting_Ending.pMatrix);
		free(Position2Supporting_Ending.RH.pMatrix);
		free(Plane2Supporting_Ending.RH.pMatrix);

		free(MatrixRotation_Z.pMatrix);
		free(InverseMatrix.pMatrix);

		InverseMatrix = Inv4(Body2Leg.RH);
		Leg2Supporting_Ending.RH = MatrixMultiply(2, InverseMatrix, Body2Supporting_Ending.RH);
		free(InverseMatrix.pMatrix);
		free(Body2Supporting_Ending.RH.pMatrix);

		*(ResultArray + 1 * 3 + 0) = *(Leg2Supporting_Ending.RH.pMatrix + 0 * 4 + 3);
		*(ResultArray + 1 * 3 + 1) = *(Leg2Supporting_Ending.RH.pMatrix + 1 * 4 + 3);
		*(ResultArray + 1 * 3 + 2) = *(Leg2Supporting_Ending.RH.pMatrix + 2 * 4 + 3);

		free(Leg2Supporting_Ending.RH.pMatrix);
	}
	else
	{
		Zero2Swing_Ending_PosZ = sRobot_MotionPara.Span_Z / 2 * polyval(Coeff_X.pMatrix, 4, (t - SupportingTime_Walk));
		Zero2Swing_Ending_PosX = sRobot_MotionPara.Span_X / 2 * polyval(Coeff_Y.pMatrix, 4, (t - SupportingTime_Walk));
		Zero2Swing_Ending_PosY = sRobot_MotionPara.Span_Y * (1 - cos(2 * PI * (t - SupportingTime_Walk) / SwingTime_Walk)) / 2;
		Theta_Swing = sRobot_MotionPara.Span_W * polyval(Coeff_W.pMatrix, 4, (t - SupportingTime_Walk));

		MatrixRotation_Z = SE3_Rz(Theta_Swing / PI * 180);
		InverseMatrix = Inv4(Plane2Body);

		//printf("Debug:Gait.c 2.5\n");
		Zero2Swing_Ending = SE3_T(-Zero2Swing_Ending_PosX, Zero2Swing_Ending_PosY, -Zero2Swing_Ending_PosZ);
		Position2Swing_Ending.RH = MatrixMultiply(2, Position2Zero.RH, Zero2Swing_Ending);
		Plane2Swing_Ending.RH = MatrixMultiply(2, Plane2Position, Position2Swing_Ending.RH);
		Body2Swing_Ending.RH = MatrixMultiply(3, MatrixRotation_Z, InverseMatrix, Plane2Swing_Ending.RH);
		free(Zero2Swing_Ending.pMatrix);
		free(Position2Swing_Ending.RH.pMatrix);
		free(Plane2Swing_Ending.RH.pMatrix);

		free(MatrixRotation_Z.pMatrix);
		free(InverseMatrix.pMatrix);

		InverseMatrix = Inv4(Body2Leg.RH);
		Leg2Swing_Ending.RH = MatrixMultiply(2, InverseMatrix, Body2Swing_Ending.RH);
		free(InverseMatrix.pMatrix);
		free(Body2Swing_Ending.RH.pMatrix);

		*(ResultArray + 1 * 3 + 0) = *(Leg2Swing_Ending.RH.pMatrix + 0 * 4 + 3);
		*(ResultArray + 1 * 3 + 1) = *(Leg2Swing_Ending.RH.pMatrix + 1 * 4 + 3);
		*(ResultArray + 1 * 3 + 2) = *(Leg2Swing_Ending.RH.pMatrix + 2 * 4 + 3);

		free(Leg2Swing_Ending.RH.pMatrix);
	}

	t = sRobot_MotionPara.Phase_RF / sRobot_MotionPara.PhaseTotal * sRobot_MotionPara.Cycle;
	if (sRobot_MotionPara.Phase_RF < sRobot_MotionPara.PhaseTotal * sRobot_MotionPara.DutyRatio)
	{
		Zero2Supporting_Ending_PosZ = sRobot_MotionPara.Span_Z / 2 * (1 - 4 * t / sRobot_MotionPara.Cycle);
		Zero2Supporting_Ending_PosX = sRobot_MotionPara.Span_X / 2 * (1 - 4 * t / sRobot_MotionPara.Cycle);
		Zero2Supporting_Ending_PosY = 0 * t;
		Theta_Supporting = sRobot_MotionPara.Span_W * (1 - 4 * t / sRobot_MotionPara.Cycle);

		MatrixRotation_Z = SE3_Rz(Theta_Supporting / PI * 180);
		InverseMatrix = Inv4(Plane2Body);

		//printf("Debug:Gait.c 2.6\n");
		Zero2Supporting_Ending = SE3_T(-Zero2Supporting_Ending_PosX, -Zero2Supporting_Ending_PosY, Zero2Supporting_Ending_PosZ);
		Position2Supporting_Ending.RF = MatrixMultiply(2, Position2Zero.RF, Zero2Supporting_Ending);
		Plane2Supporting_Ending.RF = MatrixMultiply(2, Plane2Position, Position2Supporting_Ending.RF);
		Body2Supporting_Ending.RF = MatrixMultiply(3, MatrixRotation_Z, InverseMatrix, Plane2Supporting_Ending.RF);
		free(Zero2Supporting_Ending.pMatrix);
		free(Position2Supporting_Ending.RF.pMatrix);
		free(Plane2Supporting_Ending.RF.pMatrix);

		free(MatrixRotation_Z.pMatrix);
		free(InverseMatrix.pMatrix);

		InverseMatrix = Inv4(Body2Leg.RF);
		Leg2Supporting_Ending.RF = MatrixMultiply(2, InverseMatrix, Body2Supporting_Ending.RF);
		free(InverseMatrix.pMatrix);
		free(Body2Supporting_Ending.RF.pMatrix);

		*(ResultArray + 2 * 3 + 0) = *(Leg2Supporting_Ending.RF.pMatrix + 0 * 4 + 3);
		*(ResultArray + 2 * 3 + 1) = *(Leg2Supporting_Ending.RF.pMatrix + 1 * 4 + 3);
		*(ResultArray + 2 * 3 + 2) = *(Leg2Supporting_Ending.RF.pMatrix + 2 * 4 + 3);

		free(Leg2Supporting_Ending.RF.pMatrix);
	}
	else
	{
		Zero2Swing_Ending_PosZ = sRobot_MotionPara.Span_Z / 2 * polyval(Coeff_X.pMatrix, 4, (t - SupportingTime_Walk));
		Zero2Swing_Ending_PosX = sRobot_MotionPara.Span_X / 2 * polyval(Coeff_Y.pMatrix, 4, (t - SupportingTime_Walk));
		Zero2Swing_Ending_PosY = sRobot_MotionPara.Span_Y * (1 - cos(2 * PI * (t - SupportingTime_Walk) / SwingTime_Walk)) / 2;
		Theta_Swing = sRobot_MotionPara.Span_W * polyval(Coeff_W.pMatrix, 4, (t - SupportingTime_Walk));

		MatrixRotation_Z = SE3_Rz(Theta_Swing / PI * 180);
		InverseMatrix = Inv4(Plane2Body);

		//printf("Debug:Gait.c 2.7\n");
		Zero2Swing_Ending = SE3_T(-Zero2Swing_Ending_PosX, -Zero2Swing_Ending_PosY, Zero2Swing_Ending_PosZ);
		Position2Swing_Ending.RF = MatrixMultiply(2, Position2Zero.RF, Zero2Swing_Ending);
		Plane2Swing_Ending.RF = MatrixMultiply(2, Plane2Position, Position2Swing_Ending.RF);
		Body2Swing_Ending.RF = MatrixMultiply(3, MatrixRotation_Z, InverseMatrix, Plane2Swing_Ending.RF);
		free(Zero2Swing_Ending.pMatrix);
		free(Position2Swing_Ending.RF.pMatrix);
		free(Plane2Swing_Ending.RF.pMatrix);

		free(MatrixRotation_Z.pMatrix);
		free(InverseMatrix.pMatrix);

		InverseMatrix = Inv4(Body2Leg.RF);
		Leg2Swing_Ending.RF = MatrixMultiply(2, InverseMatrix, Body2Swing_Ending.RF);
		free(InverseMatrix.pMatrix);
		free(Body2Swing_Ending.RF.pMatrix);

		*(ResultArray + 2 * 3 + 0) = *(Leg2Swing_Ending.RF.pMatrix + 0 * 4 + 3);
		*(ResultArray + 2 * 3 + 1) = *(Leg2Swing_Ending.RF.pMatrix + 1 * 4 + 3);
		*(ResultArray + 2 * 3 + 2) = *(Leg2Swing_Ending.RF.pMatrix + 2 * 4 + 3);

		free(Leg2Swing_Ending.RF.pMatrix);
	}

	return Arr2Mat(ResultArray, 4, 3);
}
