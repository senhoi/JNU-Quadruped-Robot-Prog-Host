#include "inv_kine.h"

const float ServoAngleOffset[4][3] = {
	97 / 180.0 * PI, 149 / 180.0 * PI, 24 / 180.0 * PI,  //LF
	86 / 180.0 * PI, 144 / 180.0 * PI, 23 / 180.0 * PI,  //RB
	82 / 180.0 * PI, 46 / 180.0 * PI, 159 / 180.0 * PI,  //RF
	74 / 180.0 * PI, 35 / 180.0 * PI, 157 / 180.0 * PI}; //LB

static void ReviseJointAngle(float *pJointAngle, uint8_t leg_index)
{
	pJointAngle[leg_index * 3 + 0] += ServoAngleOffset[leg_index][0];
	pJointAngle[leg_index * 3 + 1] += ServoAngleOffset[leg_index][1];
	pJointAngle[leg_index * 3 + 2] += ServoAngleOffset[leg_index][2];
}

Matrix_t InvKineCalc(uint8_t leg_index, float pos_x, float pos_y, float pos_z)
{
	float *pJointAngle = (float *)malloc(sizeof(float) * 3 * 1);

	float theta1, theta2, theta3;
	float C1, S1, C3, S3;

	if (pJointAngle == NULL)
		printf("ERROR: Malloc failed!-01");

	switch (leg_index)
	{
	case 0: //LF
		C1 = -(sRobot_MechanicalPara.Leg_d2 * pos_y + sRobot_MechanicalPara.Leg_d3 * pos_y + pos_x * sqrt(-sRobot_MechanicalPara.Leg_d2 * sRobot_MechanicalPara.Leg_d2 - 2 * sRobot_MechanicalPara.Leg_d2 * sRobot_MechanicalPara.Leg_d3 - sRobot_MechanicalPara.Leg_d3 * sRobot_MechanicalPara.Leg_d3 + pos_x * pos_x + pos_y * pos_y)) / (pos_x * pos_x + pos_y * pos_y);
		S1 = sqrt(1 - C1 * C1);
		theta1 = atan2(S1, C1);

		C3 = (((cos(theta1) * pos_x) + (sin(theta1) * pos_y)) * ((cos(theta1) * pos_x) + (sin(theta1) * pos_y)) + (pos_z - sRobot_MechanicalPara.Leg_d1) * (pos_z - sRobot_MechanicalPara.Leg_d1) - sRobot_MechanicalPara.Leg_a1 * sRobot_MechanicalPara.Leg_a1 - sRobot_MechanicalPara.Leg_a2 * sRobot_MechanicalPara.Leg_a2) / (2 * sRobot_MechanicalPara.Leg_a1 * sRobot_MechanicalPara.Leg_a2);
		S3 = -sqrt((1 - C3 * C3));
		theta3 = atan2(S3, C3);

		theta2 = PI + acos((sRobot_MechanicalPara.Leg_a1 * sqrt(2 * sRobot_MechanicalPara.Leg_d1 * pos_z + sRobot_MechanicalPara.Leg_a2 * sRobot_MechanicalPara.Leg_a2 * cos(theta3) * cos(theta3) + sRobot_MechanicalPara.Leg_a2 * sRobot_MechanicalPara.Leg_a2 * sin(theta3) * sin(theta3) + sRobot_MechanicalPara.Leg_a1 * sRobot_MechanicalPara.Leg_a1 - sRobot_MechanicalPara.Leg_d1 * sRobot_MechanicalPara.Leg_d1 - pos_z * pos_z + 2 * sRobot_MechanicalPara.Leg_a1 * sRobot_MechanicalPara.Leg_a2 * cos(theta3)) + sRobot_MechanicalPara.Leg_a2 * sRobot_MechanicalPara.Leg_d1 * sin(theta3) - sRobot_MechanicalPara.Leg_a2 * pos_z * sin(theta3) + sRobot_MechanicalPara.Leg_a2 * cos(theta3) * sqrt(2 * sRobot_MechanicalPara.Leg_d1 * pos_z + sRobot_MechanicalPara.Leg_a2 * sRobot_MechanicalPara.Leg_a2 * cos(theta3) * cos(theta3) + sRobot_MechanicalPara.Leg_a2 * sRobot_MechanicalPara.Leg_a2 * sin(theta3) * sin(theta3) + sRobot_MechanicalPara.Leg_a1 * sRobot_MechanicalPara.Leg_a1 - sRobot_MechanicalPara.Leg_d1 * sRobot_MechanicalPara.Leg_d1 - pos_z * pos_z + 2 * sRobot_MechanicalPara.Leg_a1 * sRobot_MechanicalPara.Leg_a2 * cos(theta3))) / (sRobot_MechanicalPara.Leg_a2 * sRobot_MechanicalPara.Leg_a2 * cos(theta3) * cos(theta3) + sRobot_MechanicalPara.Leg_a2 * sRobot_MechanicalPara.Leg_a2 * sin(theta3) * sin(theta3) + sRobot_MechanicalPara.Leg_a1 * sRobot_MechanicalPara.Leg_a1 + 2 * sRobot_MechanicalPara.Leg_a1 * sRobot_MechanicalPara.Leg_a2 * cos(theta3)));

		LOG("LF:%.3f %.3f %.3f", theta1, theta2, theta3);
#ifndef __RPI
		*(pJointAngle + 0) = theta1 - PI / 2;
		*(pJointAngle + 1) = -theta2 + PI * 3 / 2;
		*(pJointAngle + 2) = theta3 + PI / 2;
#else
		*(pJointAngle + 0) = theta1 - PI / 2;
		*(pJointAngle + 1) = -theta2 + PI * 3 / 2;
		*(pJointAngle + 2) = theta3 + PI;
#endif

		break;
	case 1: //RB
		C1 = -(sRobot_MechanicalPara.Leg_d2 * pos_y + sRobot_MechanicalPara.Leg_d3 * pos_y + pos_x * sqrt(-sRobot_MechanicalPara.Leg_d2 * sRobot_MechanicalPara.Leg_d2 - 2 * sRobot_MechanicalPara.Leg_d2 * sRobot_MechanicalPara.Leg_d3 - sRobot_MechanicalPara.Leg_d3 * sRobot_MechanicalPara.Leg_d3 + pos_x * pos_x + pos_y * pos_y)) / (pos_x * pos_x + pos_y * pos_y);
		S1 = sqrt(1 - C1 * C1);
		theta1 = atan2(S1, C1);

		switch (sRobot_MotionPara.Structure)
		{
		case ELBOW_ELBOW:
			C3 = (((cos(theta1) * pos_x) + (sin(theta1) * pos_y)) * ((cos(theta1) * pos_x) + (sin(theta1) * pos_y)) + (pos_z - sRobot_MechanicalPara.Leg_d1) * (pos_z - sRobot_MechanicalPara.Leg_d1) - sRobot_MechanicalPara.Leg_a1 * sRobot_MechanicalPara.Leg_a1 - sRobot_MechanicalPara.Leg_a2 * sRobot_MechanicalPara.Leg_a2) / (2 * sRobot_MechanicalPara.Leg_a1 * sRobot_MechanicalPara.Leg_a2);
			S3 = sqrt((1 - C3 * C3));
			theta3 = atan2(S3, C3);

			theta2 = PI - acos((sRobot_MechanicalPara.Leg_a1 * sqrt(2 * sRobot_MechanicalPara.Leg_d1 * pos_z + sRobot_MechanicalPara.Leg_a2 * sRobot_MechanicalPara.Leg_a2 * cos(theta3) * cos(theta3) + sRobot_MechanicalPara.Leg_a2 * sRobot_MechanicalPara.Leg_a2 * sin(theta3) * sin(theta3) + sRobot_MechanicalPara.Leg_a1 * sRobot_MechanicalPara.Leg_a1 - sRobot_MechanicalPara.Leg_d1 * sRobot_MechanicalPara.Leg_d1 - pos_z * pos_z + 2 * sRobot_MechanicalPara.Leg_a1 * sRobot_MechanicalPara.Leg_a2 * cos(theta3)) + sRobot_MechanicalPara.Leg_a2 * sRobot_MechanicalPara.Leg_d1 * sin(theta3) - sRobot_MechanicalPara.Leg_a2 * pos_z * sin(theta3) + sRobot_MechanicalPara.Leg_a2 * cos(theta3) * sqrt(2 * sRobot_MechanicalPara.Leg_d1 * pos_z + sRobot_MechanicalPara.Leg_a2 * sRobot_MechanicalPara.Leg_a2 * cos(theta3) * cos(theta3) + sRobot_MechanicalPara.Leg_a2 * sRobot_MechanicalPara.Leg_a2 * sin(theta3) * sin(theta3) + sRobot_MechanicalPara.Leg_a1 * sRobot_MechanicalPara.Leg_a1 - sRobot_MechanicalPara.Leg_d1 * sRobot_MechanicalPara.Leg_d1 - pos_z * pos_z + 2 * sRobot_MechanicalPara.Leg_a1 * sRobot_MechanicalPara.Leg_a2 * cos(theta3))) / (sRobot_MechanicalPara.Leg_a2 * sRobot_MechanicalPara.Leg_a2 * cos(theta3) * cos(theta3) + sRobot_MechanicalPara.Leg_a2 * sRobot_MechanicalPara.Leg_a2 * sin(theta3) * sin(theta3) + sRobot_MechanicalPara.Leg_a1 * sRobot_MechanicalPara.Leg_a1 + 2 * sRobot_MechanicalPara.Leg_a1 * sRobot_MechanicalPara.Leg_a2 * cos(theta3)));
			break;

		case ELBOW_KNEE:
			C3 = (((cos(theta1) * pos_x) + (sin(theta1) * pos_y)) * ((cos(theta1) * pos_x) + (sin(theta1) * pos_y)) + (pos_z - sRobot_MechanicalPara.Leg_d1) * (pos_z - sRobot_MechanicalPara.Leg_d1) - sRobot_MechanicalPara.Leg_a1 * sRobot_MechanicalPara.Leg_a1 - sRobot_MechanicalPara.Leg_a2 * sRobot_MechanicalPara.Leg_a2) / (2 * sRobot_MechanicalPara.Leg_a1 * sRobot_MechanicalPara.Leg_a2);
			S3 = -sqrt((1 - C3 * C3));
			theta3 = atan2(S3, C3);

			theta2 = PI + acos((sRobot_MechanicalPara.Leg_a1 * sqrt(2 * sRobot_MechanicalPara.Leg_d1 * pos_z + sRobot_MechanicalPara.Leg_a2 * sRobot_MechanicalPara.Leg_a2 * cos(theta3) * cos(theta3) + sRobot_MechanicalPara.Leg_a2 * sRobot_MechanicalPara.Leg_a2 * sin(theta3) * sin(theta3) + sRobot_MechanicalPara.Leg_a1 * sRobot_MechanicalPara.Leg_a1 - sRobot_MechanicalPara.Leg_d1 * sRobot_MechanicalPara.Leg_d1 - pos_z * pos_z + 2 * sRobot_MechanicalPara.Leg_a1 * sRobot_MechanicalPara.Leg_a2 * cos(theta3)) + sRobot_MechanicalPara.Leg_a2 * sRobot_MechanicalPara.Leg_d1 * sin(theta3) - sRobot_MechanicalPara.Leg_a2 * pos_z * sin(theta3) + sRobot_MechanicalPara.Leg_a2 * cos(theta3) * sqrt(2 * sRobot_MechanicalPara.Leg_d1 * pos_z + sRobot_MechanicalPara.Leg_a2 * sRobot_MechanicalPara.Leg_a2 * cos(theta3) * cos(theta3) + sRobot_MechanicalPara.Leg_a2 * sRobot_MechanicalPara.Leg_a2 * sin(theta3) * sin(theta3) + sRobot_MechanicalPara.Leg_a1 * sRobot_MechanicalPara.Leg_a1 - sRobot_MechanicalPara.Leg_d1 * sRobot_MechanicalPara.Leg_d1 - pos_z * pos_z + 2 * sRobot_MechanicalPara.Leg_a1 * sRobot_MechanicalPara.Leg_a2 * cos(theta3))) / (sRobot_MechanicalPara.Leg_a2 * sRobot_MechanicalPara.Leg_a2 * cos(theta3) * cos(theta3) + sRobot_MechanicalPara.Leg_a2 * sRobot_MechanicalPara.Leg_a2 * sin(theta3) * sin(theta3) + sRobot_MechanicalPara.Leg_a1 * sRobot_MechanicalPara.Leg_a1 + 2 * sRobot_MechanicalPara.Leg_a1 * sRobot_MechanicalPara.Leg_a2 * cos(theta3)));
			break;
		default:
			break;
		}

		LOG("RB:%.3f %.3f %.3f", theta1, theta2, theta3);
#ifndef __RPI
		*(pJointAngle + 0) = theta1 - PI / 2;
		*(pJointAngle + 1) = -theta2 + PI / 2;
		*(pJointAngle + 2) = theta3 - PI / 2;
#else
		*(pJointAngle + 0) = theta1 - PI / 2;
		*(pJointAngle + 1) = -theta2 + PI * 3 / 2;
		*(pJointAngle + 2) = theta3 + PI;
#endif

		break;
	case 2: //RF
		C1 = (sRobot_MechanicalPara.Leg_d2 * pos_y + sRobot_MechanicalPara.Leg_d3 * pos_y - pos_x * sqrt(-sRobot_MechanicalPara.Leg_d2 * sRobot_MechanicalPara.Leg_d2 - 2 * sRobot_MechanicalPara.Leg_d2 * sRobot_MechanicalPara.Leg_d3 - sRobot_MechanicalPara.Leg_d3 * sRobot_MechanicalPara.Leg_d3 + pos_x * pos_x + pos_y * pos_y)) / (pos_x * pos_x + pos_y * pos_y);
		S1 = -sqrt(1 - C1 * C1);
		theta1 = atan2(S1, C1);

		C3 = (((cos(theta1) * pos_x) + (sin(theta1) * pos_y)) * ((cos(theta1) * pos_x) + (sin(theta1) * pos_y)) + (pos_z - sRobot_MechanicalPara.Leg_d1) * (pos_z - sRobot_MechanicalPara.Leg_d1) - sRobot_MechanicalPara.Leg_a1 * sRobot_MechanicalPara.Leg_a1 - sRobot_MechanicalPara.Leg_a2 * sRobot_MechanicalPara.Leg_a2) / (2 * sRobot_MechanicalPara.Leg_a1 * sRobot_MechanicalPara.Leg_a2);
		S3 = sqrt(1 - C3 * C3);
		theta3 = atan2(S3, C3);

		theta2 = PI - acos((sRobot_MechanicalPara.Leg_a1 * sqrt(2 * sRobot_MechanicalPara.Leg_d1 * pos_z + sRobot_MechanicalPara.Leg_a2 * sRobot_MechanicalPara.Leg_a2 * cos(theta3) * cos(theta3) + sRobot_MechanicalPara.Leg_a2 * sRobot_MechanicalPara.Leg_a2 * sin(theta3) * sin(theta3) + sRobot_MechanicalPara.Leg_a1 * sRobot_MechanicalPara.Leg_a1 - sRobot_MechanicalPara.Leg_d1 * sRobot_MechanicalPara.Leg_d1 - pos_z * pos_z + 2 * sRobot_MechanicalPara.Leg_a1 * sRobot_MechanicalPara.Leg_a2 * cos(theta3)) - sRobot_MechanicalPara.Leg_a2 * sRobot_MechanicalPara.Leg_d1 * sin(theta3) + sRobot_MechanicalPara.Leg_a2 * pos_z * sin(theta3) + sRobot_MechanicalPara.Leg_a2 * cos(theta3) * sqrt(2 * sRobot_MechanicalPara.Leg_d1 * pos_z + sRobot_MechanicalPara.Leg_a2 * sRobot_MechanicalPara.Leg_a2 * cos(theta3) * cos(theta3) + sRobot_MechanicalPara.Leg_a2 * sRobot_MechanicalPara.Leg_a2 * sin(theta3) * sin(theta3) + sRobot_MechanicalPara.Leg_a1 * sRobot_MechanicalPara.Leg_a1 - sRobot_MechanicalPara.Leg_d1 * sRobot_MechanicalPara.Leg_d1 - pos_z * pos_z + 2 * sRobot_MechanicalPara.Leg_a1 * sRobot_MechanicalPara.Leg_a2 * cos(theta3))) / (sRobot_MechanicalPara.Leg_a2 * sRobot_MechanicalPara.Leg_a2 * cos(theta3) * cos(theta3) + sRobot_MechanicalPara.Leg_a2 * sRobot_MechanicalPara.Leg_a2 * sin(theta3) * sin(theta3) + sRobot_MechanicalPara.Leg_a1 * sRobot_MechanicalPara.Leg_a1 + 2 * sRobot_MechanicalPara.Leg_a1 * sRobot_MechanicalPara.Leg_a2 * cos(theta3)));

		LOG("RF:%.3f %.3f %.3f", theta1, theta2, theta3);
#ifndef __RPI
		*(pJointAngle + 0) = theta1 + PI / 2;
		*(pJointAngle + 1) = -theta2 + PI / 2;
		*(pJointAngle + 2) = theta3 - PI / 2;
#else
		*(pJointAngle + 0) = theta1 + PI / 2;
		*(pJointAngle + 1) = theta2 - PI / 2;
		*(pJointAngle + 2) = -theta3 + PI;
#endif

		break;
	case 3: //LB
		C1 = (sRobot_MechanicalPara.Leg_d2 * pos_y + sRobot_MechanicalPara.Leg_d3 * pos_y - pos_x * sqrt(-sRobot_MechanicalPara.Leg_d2 * sRobot_MechanicalPara.Leg_d2 - 2 * sRobot_MechanicalPara.Leg_d2 * sRobot_MechanicalPara.Leg_d3 - sRobot_MechanicalPara.Leg_d3 * sRobot_MechanicalPara.Leg_d3 + pos_x * pos_x + pos_y * pos_y)) / (pos_x * pos_x + pos_y * pos_y);
		S1 = -sqrt(1 - C1 * C1);
		theta1 = atan2(S1, C1);

		switch (sRobot_MotionPara.Structure)
		{
		case ELBOW_ELBOW:
			C3 = (((cos(theta1) * pos_x) + (sin(theta1) * pos_y)) * ((cos(theta1) * pos_x) + (sin(theta1) * pos_y)) + (pos_z - sRobot_MechanicalPara.Leg_d1) * (pos_z - sRobot_MechanicalPara.Leg_d1) - sRobot_MechanicalPara.Leg_a1 * sRobot_MechanicalPara.Leg_a1 - sRobot_MechanicalPara.Leg_a2 * sRobot_MechanicalPara.Leg_a2) / (2 * sRobot_MechanicalPara.Leg_a1 * sRobot_MechanicalPara.Leg_a2);
			S3 = -sqrt(1 - C3 * C3);
			theta3 = atan2(S3, C3);

			theta2 = PI + acos((sRobot_MechanicalPara.Leg_a1 * sqrt(2 * sRobot_MechanicalPara.Leg_d1 * pos_z + sRobot_MechanicalPara.Leg_a2 * sRobot_MechanicalPara.Leg_a2 * cos(theta3) * cos(theta3) + sRobot_MechanicalPara.Leg_a2 * sRobot_MechanicalPara.Leg_a2 * sin(theta3) * sin(theta3) + sRobot_MechanicalPara.Leg_a1 * sRobot_MechanicalPara.Leg_a1 - sRobot_MechanicalPara.Leg_d1 * sRobot_MechanicalPara.Leg_d1 - pos_z * pos_z + 2 * sRobot_MechanicalPara.Leg_a1 * sRobot_MechanicalPara.Leg_a2 * cos(theta3)) - sRobot_MechanicalPara.Leg_a2 * sRobot_MechanicalPara.Leg_d1 * sin(theta3) + sRobot_MechanicalPara.Leg_a2 * pos_z * sin(theta3) + sRobot_MechanicalPara.Leg_a2 * cos(theta3) * sqrt(2 * sRobot_MechanicalPara.Leg_d1 * pos_z + sRobot_MechanicalPara.Leg_a2 * sRobot_MechanicalPara.Leg_a2 * cos(theta3) * cos(theta3) + sRobot_MechanicalPara.Leg_a2 * sRobot_MechanicalPara.Leg_a2 * sin(theta3) * sin(theta3) + sRobot_MechanicalPara.Leg_a1 * sRobot_MechanicalPara.Leg_a1 - sRobot_MechanicalPara.Leg_d1 * sRobot_MechanicalPara.Leg_d1 - pos_z * pos_z + 2 * sRobot_MechanicalPara.Leg_a1 * sRobot_MechanicalPara.Leg_a2 * cos(theta3))) / (sRobot_MechanicalPara.Leg_a2 * sRobot_MechanicalPara.Leg_a2 * cos(theta3) * cos(theta3) + sRobot_MechanicalPara.Leg_a2 * sRobot_MechanicalPara.Leg_a2 * sin(theta3) * sin(theta3) + sRobot_MechanicalPara.Leg_a1 * sRobot_MechanicalPara.Leg_a1 + 2 * sRobot_MechanicalPara.Leg_a1 * sRobot_MechanicalPara.Leg_a2 * cos(theta3)));
			break;

		case ELBOW_KNEE:
			C3 = (((cos(theta1) * pos_x) + (sin(theta1) * pos_y)) * ((cos(theta1) * pos_x) + (sin(theta1) * pos_y)) + (pos_z - sRobot_MechanicalPara.Leg_d1) * (pos_z - sRobot_MechanicalPara.Leg_d1) - sRobot_MechanicalPara.Leg_a1 * sRobot_MechanicalPara.Leg_a1 - sRobot_MechanicalPara.Leg_a2 * sRobot_MechanicalPara.Leg_a2) / (2 * sRobot_MechanicalPara.Leg_a1 * sRobot_MechanicalPara.Leg_a2);
			S3 = sqrt(1 - C3 * C3);
			theta3 = atan2(S3, C3);

			theta2 = PI - acos((sRobot_MechanicalPara.Leg_a1 * sqrt(2 * sRobot_MechanicalPara.Leg_d1 * pos_z + sRobot_MechanicalPara.Leg_a2 * sRobot_MechanicalPara.Leg_a2 * cos(theta3) * cos(theta3) + sRobot_MechanicalPara.Leg_a2 * sRobot_MechanicalPara.Leg_a2 * sin(theta3) * sin(theta3) + sRobot_MechanicalPara.Leg_a1 * sRobot_MechanicalPara.Leg_a1 - sRobot_MechanicalPara.Leg_d1 * sRobot_MechanicalPara.Leg_d1 - pos_z * pos_z + 2 * sRobot_MechanicalPara.Leg_a1 * sRobot_MechanicalPara.Leg_a2 * cos(theta3)) - sRobot_MechanicalPara.Leg_a2 * sRobot_MechanicalPara.Leg_d1 * sin(theta3) + sRobot_MechanicalPara.Leg_a2 * pos_z * sin(theta3) + sRobot_MechanicalPara.Leg_a2 * cos(theta3) * sqrt(2 * sRobot_MechanicalPara.Leg_d1 * pos_z + sRobot_MechanicalPara.Leg_a2 * sRobot_MechanicalPara.Leg_a2 * cos(theta3) * cos(theta3) + sRobot_MechanicalPara.Leg_a2 * sRobot_MechanicalPara.Leg_a2 * sin(theta3) * sin(theta3) + sRobot_MechanicalPara.Leg_a1 * sRobot_MechanicalPara.Leg_a1 - sRobot_MechanicalPara.Leg_d1 * sRobot_MechanicalPara.Leg_d1 - pos_z * pos_z + 2 * sRobot_MechanicalPara.Leg_a1 * sRobot_MechanicalPara.Leg_a2 * cos(theta3))) / (sRobot_MechanicalPara.Leg_a2 * sRobot_MechanicalPara.Leg_a2 * cos(theta3) * cos(theta3) + sRobot_MechanicalPara.Leg_a2 * sRobot_MechanicalPara.Leg_a2 * sin(theta3) * sin(theta3) + sRobot_MechanicalPara.Leg_a1 * sRobot_MechanicalPara.Leg_a1 + 2 * sRobot_MechanicalPara.Leg_a1 * sRobot_MechanicalPara.Leg_a2 * cos(theta3)));
			break;
		default:
			break;
		}

		LOG("LB:%.3f %.3f %.3f", theta1, theta2, theta3);
#ifndef __RPI
		*(pJointAngle + 0) = theta1 + PI / 2;
		*(pJointAngle + 1) = -theta2 + PI * 3 / 2;
		*(pJointAngle + 2) = theta3 + PI / 2;
#else
		*(pJointAngle + 0) = theta1 + PI / 2;
		*(pJointAngle + 1) = theta2 - PI / 2;
		*(pJointAngle + 2) = -theta3 + PI;
#endif

		break;
	}

#ifdef __RPI
	ReviseJointAngle(pJointAngle, leg_index);
#endif

	return Arr2Mat(pJointAngle, 3, 1);
}
