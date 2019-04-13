#include "scurve.h"

/**
* @brief  S曲线位置控制限制最大速度条件下(MaxSpd)计算加、减匀速时间
* @param
*		*pSCurvePosCtrl	S曲线位置控制结构体指针
* @retval
*
*/
static void SCurvePosCtrl_MaxSpdLimited(SCurvePosCtrl_t *pSCurvePosCtrl)
{
	if (pSCurvePosCtrl->Spd < pSCurvePosCtrl->MaxSpd)
	{
		pSCurvePosCtrl->DeltaPos_MaxSpdLimitedCondition0 = 0.5f * (pSCurvePosCtrl->MaxSpd * pSCurvePosCtrl->MaxSpd - pSCurvePosCtrl->Spd * pSCurvePosCtrl->Spd) / pSCurvePosCtrl->Acc + 0.5f * pSCurvePosCtrl->MaxSpd * pSCurvePosCtrl->MaxSpd / pSCurvePosCtrl->Dec;
		pSCurvePosCtrl->DeltaPos_MaxSpdLimitedCondition1 = 0.5f * pSCurvePosCtrl->Spd * pSCurvePosCtrl->Spd / pSCurvePosCtrl->Dec;
		pSCurvePosCtrl->DeltaPos_MaxSpdLimitedCondition2 = 0.5f * (pSCurvePosCtrl->MaxSpd * pSCurvePosCtrl->MaxSpd - pSCurvePosCtrl->Spd * pSCurvePosCtrl->Spd) / (-pSCurvePosCtrl->Acc) - 0.5f * pSCurvePosCtrl->MaxSpd * pSCurvePosCtrl->MaxSpd / pSCurvePosCtrl->Dec;
	}
	else if (pSCurvePosCtrl->Spd >= pSCurvePosCtrl->MaxSpd)
	{
		pSCurvePosCtrl->DeltaPos_MaxSpdLimitedCondition0 = 0.5f * pSCurvePosCtrl->Spd * pSCurvePosCtrl->Spd / pSCurvePosCtrl->Acc;
		pSCurvePosCtrl->DeltaPos_MaxSpdLimitedCondition1 = 0.5f * (pSCurvePosCtrl->MaxSpd * pSCurvePosCtrl->MaxSpd - pSCurvePosCtrl->Spd * pSCurvePosCtrl->Spd) / (-pSCurvePosCtrl->Acc) - 0.5f * pSCurvePosCtrl->MaxSpd * pSCurvePosCtrl->MaxSpd / pSCurvePosCtrl->Dec;
	}

	if (pSCurvePosCtrl->Spd < pSCurvePosCtrl->MaxSpd)
	{
		if (pSCurvePosCtrl->DeltaPos > pSCurvePosCtrl->DeltaPos_MaxSpdLimitedCondition0)
		{
			pSCurvePosCtrl->AccTime = (pSCurvePosCtrl->MaxSpd - pSCurvePosCtrl->Spd) / pSCurvePosCtrl->Acc;
			pSCurvePosCtrl->DecTime = pSCurvePosCtrl->MaxSpd / pSCurvePosCtrl->Dec;
			pSCurvePosCtrl->MidTime = (pSCurvePosCtrl->FinishPos - pSCurvePosCtrl->OriginPos - 1.0f / 2.0f * (pSCurvePosCtrl->MaxSpd + pSCurvePosCtrl->Spd) * pSCurvePosCtrl->AccTime - 1.0f / 2.0f * pSCurvePosCtrl->MaxSpd * pSCurvePosCtrl->DecTime) / pSCurvePosCtrl->MaxSpd;
		}
		else if (pSCurvePosCtrl->DeltaPos > pSCurvePosCtrl->DeltaPos_MaxSpdLimitedCondition1 && pSCurvePosCtrl->DeltaPos <= pSCurvePosCtrl->DeltaPos_MaxSpdLimitedCondition0)
		{
			pSCurvePosCtrl->DecTime = sqrt((2.0f * pSCurvePosCtrl->Acc * (pSCurvePosCtrl->FinishPos - pSCurvePosCtrl->OriginPos) + pSCurvePosCtrl->Spd * pSCurvePosCtrl->Spd) / (pSCurvePosCtrl->Acc * pSCurvePosCtrl->Dec + pSCurvePosCtrl->Dec * pSCurvePosCtrl->Dec));
			pSCurvePosCtrl->AccTime = (pSCurvePosCtrl->DecTime * pSCurvePosCtrl->Dec - pSCurvePosCtrl->Spd) / pSCurvePosCtrl->Acc;
			pSCurvePosCtrl->MidTime = 0.0f;
		}
		else if (pSCurvePosCtrl->DeltaPos > pSCurvePosCtrl->DeltaPos_MaxSpdLimitedCondition2 && pSCurvePosCtrl->DeltaPos <= pSCurvePosCtrl->DeltaPos_MaxSpdLimitedCondition1)
		{
			pSCurvePosCtrl->Acc = -pSCurvePosCtrl->Acc;
			pSCurvePosCtrl->Dec = -pSCurvePosCtrl->Dec;
			pSCurvePosCtrl->MaxSpd = -pSCurvePosCtrl->MaxSpd;
			pSCurvePosCtrl->DecTime = sqrt((2.0f * pSCurvePosCtrl->Acc * (pSCurvePosCtrl->FinishPos - pSCurvePosCtrl->OriginPos) + pSCurvePosCtrl->Spd * pSCurvePosCtrl->Spd) / (pSCurvePosCtrl->Acc * pSCurvePosCtrl->Dec + pSCurvePosCtrl->Dec * pSCurvePosCtrl->Dec));
			pSCurvePosCtrl->AccTime = (pSCurvePosCtrl->DecTime * pSCurvePosCtrl->Dec - pSCurvePosCtrl->Spd) / pSCurvePosCtrl->Acc;
			pSCurvePosCtrl->MidTime = 0.0f;
		}
		else if (pSCurvePosCtrl->DeltaPos <= pSCurvePosCtrl->DeltaPos_MaxSpdLimitedCondition2)
		{
			pSCurvePosCtrl->Acc = -pSCurvePosCtrl->Acc;
			pSCurvePosCtrl->Dec = -pSCurvePosCtrl->Dec;
			pSCurvePosCtrl->MaxSpd = -pSCurvePosCtrl->MaxSpd;
			pSCurvePosCtrl->AccTime = (pSCurvePosCtrl->MaxSpd - pSCurvePosCtrl->Spd) / pSCurvePosCtrl->Acc;
			pSCurvePosCtrl->DecTime = pSCurvePosCtrl->MaxSpd / pSCurvePosCtrl->Dec;
			pSCurvePosCtrl->MidTime = (pSCurvePosCtrl->FinishPos - pSCurvePosCtrl->OriginPos - 1.0f / 2.0f * (pSCurvePosCtrl->MaxSpd + pSCurvePosCtrl->Spd) * pSCurvePosCtrl->AccTime - 1.0f / 2.0f * pSCurvePosCtrl->MaxSpd * pSCurvePosCtrl->DecTime) / pSCurvePosCtrl->MaxSpd;
		}
	}
	else if (pSCurvePosCtrl->Spd >= pSCurvePosCtrl->MaxSpd)
	{
		if (pSCurvePosCtrl->DeltaPos > pSCurvePosCtrl->DeltaPos_MaxSpdLimitedCondition0)
		{
			pSCurvePosCtrl->Acc = -pSCurvePosCtrl->Acc;
			pSCurvePosCtrl->AccTime = (pSCurvePosCtrl->MaxSpd - pSCurvePosCtrl->Spd) / pSCurvePosCtrl->Acc;
			pSCurvePosCtrl->DecTime = pSCurvePosCtrl->MaxSpd / pSCurvePosCtrl->Dec;
			pSCurvePosCtrl->MidTime = (pSCurvePosCtrl->FinishPos - pSCurvePosCtrl->OriginPos - 1.0f / 2.0f * (pSCurvePosCtrl->MaxSpd + pSCurvePosCtrl->Spd) * pSCurvePosCtrl->AccTime - 1.0f / 2.0f * pSCurvePosCtrl->MaxSpd * pSCurvePosCtrl->DecTime) / pSCurvePosCtrl->MaxSpd;
		}
		else if (pSCurvePosCtrl->DeltaPos > pSCurvePosCtrl->DeltaPos_MaxSpdLimitedCondition1 && pSCurvePosCtrl->DeltaPos <= pSCurvePosCtrl->DeltaPos_MaxSpdLimitedCondition0)
		{
			pSCurvePosCtrl->Acc = -pSCurvePosCtrl->Acc;
			pSCurvePosCtrl->Dec = -pSCurvePosCtrl->Dec;
			pSCurvePosCtrl->MaxSpd = -pSCurvePosCtrl->MaxSpd;
			pSCurvePosCtrl->DecTime = sqrt((2.0f * pSCurvePosCtrl->Acc * (pSCurvePosCtrl->FinishPos - pSCurvePosCtrl->OriginPos) + pSCurvePosCtrl->Spd * pSCurvePosCtrl->Spd) / (pSCurvePosCtrl->Acc * pSCurvePosCtrl->Dec + pSCurvePosCtrl->Dec * pSCurvePosCtrl->Dec));
			pSCurvePosCtrl->AccTime = (pSCurvePosCtrl->DecTime * pSCurvePosCtrl->Dec - pSCurvePosCtrl->Spd) / pSCurvePosCtrl->Acc;
			pSCurvePosCtrl->MidTime = 0.0f;
		}
		else if (pSCurvePosCtrl->DeltaPos <= pSCurvePosCtrl->DeltaPos_MaxSpdLimitedCondition1)
		{
			pSCurvePosCtrl->Acc = -pSCurvePosCtrl->Acc;
			pSCurvePosCtrl->Dec = -pSCurvePosCtrl->Dec;
			pSCurvePosCtrl->MaxSpd = -pSCurvePosCtrl->MaxSpd;
			pSCurvePosCtrl->AccTime = (pSCurvePosCtrl->MaxSpd - pSCurvePosCtrl->Spd) / pSCurvePosCtrl->Acc;
			pSCurvePosCtrl->DecTime = pSCurvePosCtrl->MaxSpd / pSCurvePosCtrl->Dec;
			pSCurvePosCtrl->MidTime = (pSCurvePosCtrl->FinishPos - pSCurvePosCtrl->OriginPos - 1.0f / 2.0f * (pSCurvePosCtrl->MaxSpd + pSCurvePosCtrl->Spd) * pSCurvePosCtrl->AccTime - 1.0f / 2.0f * pSCurvePosCtrl->MaxSpd * pSCurvePosCtrl->DecTime) / pSCurvePosCtrl->MaxSpd;
		}
	}
}

/**
* @brief  S曲线位置控制限制总运行周期条件下(Cycle)计算加、减匀速时间
* @param
*		*pSCurvePosCtrl	S曲线位置控制结构体指针
* @retval
*
*/
static void SCurvePosCtrl_CycleLimited(SCurvePosCtrl_t *pSCurvePosCtrl)
{
	pSCurvePosCtrl->DeltaPos_CycleLimitedCondition0 = 0.5f * (2.0f * pSCurvePosCtrl->Spd + pSCurvePosCtrl->Acc * (pSCurvePosCtrl->Dec * pSCurvePosCtrl->Cycle - pSCurvePosCtrl->Spd) / (pSCurvePosCtrl->Acc + pSCurvePosCtrl->Dec)) * (pSCurvePosCtrl->Dec * pSCurvePosCtrl->Cycle - pSCurvePosCtrl->Spd) / (pSCurvePosCtrl->Acc + pSCurvePosCtrl->Dec) + 0.5f * (pSCurvePosCtrl->Spd + pSCurvePosCtrl->Acc * (pSCurvePosCtrl->Dec * pSCurvePosCtrl->Cycle - pSCurvePosCtrl->Spd) / (pSCurvePosCtrl->Acc + pSCurvePosCtrl->Dec)) * ((pSCurvePosCtrl->Acc * pSCurvePosCtrl->Cycle + pSCurvePosCtrl->Spd) / (pSCurvePosCtrl->Acc + pSCurvePosCtrl->Dec));
	pSCurvePosCtrl->DeltaPos_CycleLimitedCondition1 = 0.5f * pSCurvePosCtrl->Spd * (2.0f * pSCurvePosCtrl->Cycle - fabs(pSCurvePosCtrl->Spd) / pSCurvePosCtrl->Acc);
	pSCurvePosCtrl->DeltaPos_CycleLimitedCondition2 = 0.5f * pSCurvePosCtrl->Spd * fabs(pSCurvePosCtrl->Spd) / pSCurvePosCtrl->Acc;
	pSCurvePosCtrl->DeltaPos_CycleLimitedCondition3 = 0.5f * (2.0f * pSCurvePosCtrl->Spd - pSCurvePosCtrl->Acc * (-pSCurvePosCtrl->Dec * pSCurvePosCtrl->Cycle - pSCurvePosCtrl->Spd) / (-pSCurvePosCtrl->Acc - pSCurvePosCtrl->Dec)) * (-pSCurvePosCtrl->Dec * pSCurvePosCtrl->Cycle - pSCurvePosCtrl->Spd) / (-pSCurvePosCtrl->Acc - pSCurvePosCtrl->Dec) + 0.5f * (pSCurvePosCtrl->Spd - pSCurvePosCtrl->Acc * (-pSCurvePosCtrl->Dec * pSCurvePosCtrl->Cycle - pSCurvePosCtrl->Spd) / (-pSCurvePosCtrl->Acc - pSCurvePosCtrl->Dec)) * ((-pSCurvePosCtrl->Acc * pSCurvePosCtrl->Cycle + pSCurvePosCtrl->Spd) / (-pSCurvePosCtrl->Acc - pSCurvePosCtrl->Dec));

	while (1)
	{
		if (pSCurvePosCtrl->Spd > 0)
		{
			if (pSCurvePosCtrl->DeltaPos > pSCurvePosCtrl->DeltaPos_CycleLimitedCondition0)
			{
				pSCurvePosCtrl->Acc = pSCurvePosCtrl->Acc + pSCurvePosCtrl->AccStep;
				pSCurvePosCtrl->Dec = pSCurvePosCtrl->Acc;
			}
			else if (pSCurvePosCtrl->DeltaPos > pSCurvePosCtrl->DeltaPos_CycleLimitedCondition1 && pSCurvePosCtrl->DeltaPos <= pSCurvePosCtrl->DeltaPos_CycleLimitedCondition0)
			{
				pSCurvePosCtrl->Acc = pSCurvePosCtrl->Acc;
				pSCurvePosCtrl->Dec = fabs(pSCurvePosCtrl->Acc);
			}
			else if (pSCurvePosCtrl->DeltaPos > pSCurvePosCtrl->DeltaPos_CycleLimitedCondition2 && pSCurvePosCtrl->DeltaPos <= pSCurvePosCtrl->DeltaPos_CycleLimitedCondition1)
			{
				pSCurvePosCtrl->Acc = -pSCurvePosCtrl->Acc;
				pSCurvePosCtrl->Dec = fabs(pSCurvePosCtrl->Acc);
			}
			else if (pSCurvePosCtrl->DeltaPos > pSCurvePosCtrl->DeltaPos_CycleLimitedCondition3 && pSCurvePosCtrl->DeltaPos <= pSCurvePosCtrl->DeltaPos_CycleLimitedCondition2)
			{
				pSCurvePosCtrl->Acc = -pSCurvePosCtrl->Acc;
				pSCurvePosCtrl->Dec = pSCurvePosCtrl->Acc;
			}
			else if (pSCurvePosCtrl->DeltaPos <= pSCurvePosCtrl->DeltaPos_CycleLimitedCondition3)
			{
				if (pSCurvePosCtrl->Acc > 0)
				{
					pSCurvePosCtrl->Acc = -pSCurvePosCtrl->Acc;
					pSCurvePosCtrl->Dec = pSCurvePosCtrl->Acc;
				}
				pSCurvePosCtrl->Acc = pSCurvePosCtrl->Acc - pSCurvePosCtrl->AccStep;
				pSCurvePosCtrl->Dec = pSCurvePosCtrl->Acc;
			}
		}
		else
		{
			if (pSCurvePosCtrl->DeltaPos > pSCurvePosCtrl->DeltaPos_CycleLimitedCondition0)
			{
				pSCurvePosCtrl->Acc = pSCurvePosCtrl->Acc + pSCurvePosCtrl->AccStep;
				pSCurvePosCtrl->Dec = pSCurvePosCtrl->Acc;
			}

			else if (pSCurvePosCtrl->DeltaPos > pSCurvePosCtrl->DeltaPos_CycleLimitedCondition2 && pSCurvePosCtrl->DeltaPos <= pSCurvePosCtrl->DeltaPos_CycleLimitedCondition0)
			{
				pSCurvePosCtrl->Acc = pSCurvePosCtrl->Acc;
				pSCurvePosCtrl->Dec = fabs(pSCurvePosCtrl->Acc);
			}

			else if (pSCurvePosCtrl->DeltaPos > pSCurvePosCtrl->DeltaPos_CycleLimitedCondition1 && pSCurvePosCtrl->DeltaPos <= pSCurvePosCtrl->DeltaPos_CycleLimitedCondition2)
			{
				pSCurvePosCtrl->Acc = pSCurvePosCtrl->Acc;
				pSCurvePosCtrl->Dec = -pSCurvePosCtrl->Acc;
			}
			else if (pSCurvePosCtrl->DeltaPos > pSCurvePosCtrl->DeltaPos_CycleLimitedCondition3 && pSCurvePosCtrl->DeltaPos <= pSCurvePosCtrl->DeltaPos_CycleLimitedCondition1)
			{
				pSCurvePosCtrl->Acc = -pSCurvePosCtrl->Acc;
				pSCurvePosCtrl->Dec = pSCurvePosCtrl->Acc;
			}
			else if (pSCurvePosCtrl->DeltaPos <= pSCurvePosCtrl->DeltaPos_CycleLimitedCondition3)
			{
				if (pSCurvePosCtrl->Acc > 0)
				{
					pSCurvePosCtrl->Acc = -pSCurvePosCtrl->Acc;
					pSCurvePosCtrl->Dec = pSCurvePosCtrl->Acc;
				}
				pSCurvePosCtrl->Acc = pSCurvePosCtrl->Acc - pSCurvePosCtrl->AccStep;
				pSCurvePosCtrl->Dec = pSCurvePosCtrl->Acc;
			}
		}

		float coeff_a = 1.0f + pSCurvePosCtrl->Acc / pSCurvePosCtrl->Dec;
		float coeff_b = -(2.0f * pSCurvePosCtrl->Cycle - 2.0f * pSCurvePosCtrl->Spd / pSCurvePosCtrl->Dec);
		float coeff_c = -(2.0f * pSCurvePosCtrl->Cycle * pSCurvePosCtrl->Spd * pSCurvePosCtrl->Dec - pSCurvePosCtrl->Spd * pSCurvePosCtrl->Spd) / (pSCurvePosCtrl->Acc * pSCurvePosCtrl->Dec) + 2.0f * pSCurvePosCtrl->DeltaPos / pSCurvePosCtrl->Acc;

		if (fabs(coeff_a) < SCURVE_EQUAL_ERROR)
		{
			pSCurvePosCtrl->AccTime = (2.0f * pSCurvePosCtrl->DeltaPos - pSCurvePosCtrl->Spd * pSCurvePosCtrl->Spd / pSCurvePosCtrl->Acc - 2.0f * pSCurvePosCtrl->Cycle * pSCurvePosCtrl->Spd) / (2.0f * pSCurvePosCtrl->Spd + 2.0f * pSCurvePosCtrl->Acc * pSCurvePosCtrl->Cycle);
			pSCurvePosCtrl->DecTime = -pSCurvePosCtrl->Spd / pSCurvePosCtrl->Acc - pSCurvePosCtrl->AccTime;
			pSCurvePosCtrl->MidTime = pSCurvePosCtrl->Cycle - pSCurvePosCtrl->AccTime - pSCurvePosCtrl->DecTime;
		}
		else
		{
			pSCurvePosCtrl->AccTime = (-coeff_b - sqrt(coeff_b * coeff_b - 4.0f * coeff_a * coeff_c)) / (2.0f * coeff_a);
			pSCurvePosCtrl->DecTime = (pSCurvePosCtrl->Spd + pSCurvePosCtrl->Acc * pSCurvePosCtrl->AccTime) / pSCurvePosCtrl->Dec;
			pSCurvePosCtrl->MidTime = pSCurvePosCtrl->Cycle - pSCurvePosCtrl->AccTime - pSCurvePosCtrl->DecTime;
		}

		if (pSCurvePosCtrl->AccTime >= 0 && pSCurvePosCtrl->DecTime >= 0 && pSCurvePosCtrl->MidTime >= 0 && coeff_b * coeff_b - 4.0f * coeff_a * coeff_c >= 0.0f)
		{
			printf("acc:%f dec:%f", pSCurvePosCtrl->Acc, pSCurvePosCtrl->Dec);
			break;
		}
		printf("AccTime:%f DecTime:%f MidTime:%f det:%f acc:%f dec:%f\n", pSCurvePosCtrl->AccTime, pSCurvePosCtrl->DecTime, pSCurvePosCtrl->MidTime, coeff_b * coeff_b - 4.0f * coeff_a * coeff_c, pSCurvePosCtrl->Acc, pSCurvePosCtrl->Dec);
	}
}

/**
* @brief  初始化S曲线位置控制结构体
* @param
*		*pSCurvePosCtrl	S曲线位置控制结构体指针
*		precision_level		求解精度等级
*		time_inc			每次执行的离散时间增量	[s]
*		cycle				每次执行的总时长（若Cycle>0则为CycleLimited模式，否则为MaxSpdLimited模式）[s]
*		acc_step			自动求解加速度时步长（绝对值）
*		acc				变速段加速度（绝对值）
*		max_spd			最大速度（绝对值）
*		origin_pos		起点位置
*		finish_pos		终点位置
* @retval
*
*/
void SCurvePosCtrl_New(SCurvePosCtrl_t *pSCurvePosCtrl, unsigned int precision_level, float time_inc, float cycle, float acc_step, float acc, float max_spd, float origin_pos, float finish_pos)
{
	pSCurvePosCtrl->Flag = 0;
	pSCurvePosCtrl->Precision = precision_level;
	pSCurvePosCtrl->Time = 0.0f;
	pSCurvePosCtrl->TimeInc = time_inc;
	pSCurvePosCtrl->Cycle = cycle;
	pSCurvePosCtrl->AccStep = acc_step;
	pSCurvePosCtrl->Acc = acc;
	pSCurvePosCtrl->Dec = pSCurvePosCtrl->Acc;
	pSCurvePosCtrl->MaxSpd = max_spd;
	pSCurvePosCtrl->OriginPos = origin_pos;
	pSCurvePosCtrl->FinishPos = finish_pos;

	pSCurvePosCtrl->Spd = 0.0f;
	pSCurvePosCtrl->Pos = origin_pos;

	pSCurvePosCtrl->DeltaPos = pSCurvePosCtrl->FinishPos - pSCurvePosCtrl->OriginPos;

	if (fabs(pSCurvePosCtrl->Cycle) > SCURVE_EQUAL_ERROR)
	{
		SCurvePosCtrl_CycleLimited(pSCurvePosCtrl);
	}
	else
	{
		SCurvePosCtrl_MaxSpdLimited(pSCurvePosCtrl);
	}
}

/**
* @brief  S曲线位置控制设定新位置
* @param
*		*pSCurvePosCtrl	S曲线位置控制结构体指针
*		cycle				每次执行的总时长（若Cycle>0则为CycleLimited模式，否则为MaxSpdLimited模式）[s]
*		acc				加速段加速度（绝对值）
*		dec				减速段加速度（绝对值）
*		max_spd			最大速度（绝对值）
*		finish_pos		终点位置
* @retval
*
*/
void SCurvePosCtrl_SetNewPos(SCurvePosCtrl_t *pSCurvePosCtrl, float cycle, float max_spd, float acc, float finish_pos)
{
	pSCurvePosCtrl->Time = 0.0f;
	pSCurvePosCtrl->Cycle = cycle;
	pSCurvePosCtrl->Acc = acc;
	pSCurvePosCtrl->Dec = pSCurvePosCtrl->Acc;
	pSCurvePosCtrl->MaxSpd = max_spd;
	pSCurvePosCtrl->OriginPos = pSCurvePosCtrl->Pos;
	pSCurvePosCtrl->FinishPos = finish_pos;

	pSCurvePosCtrl->DeltaPos = pSCurvePosCtrl->FinishPos - pSCurvePosCtrl->OriginPos;

	if (fabs(pSCurvePosCtrl->Cycle) > SCURVE_EQUAL_ERROR)
	{
		SCurvePosCtrl_CycleLimited(pSCurvePosCtrl);
	}
	else
	{
		SCurvePosCtrl_MaxSpdLimited(pSCurvePosCtrl);
	}
}

/**
* @brief  S曲线位置控制计算
* @param
*		*pSCurvePosCtrl:	S曲线位置控制结构体指针
* @retval
*		0	标志位没有置位，本次不进行计算
*		1	已到达目标位置
*		2	位置计算完成
*/
int SCurvePosCtrl_Calc(SCurvePosCtrl_t *pSCurvePosCtrl)
{
	float real_acc;

	if (pSCurvePosCtrl->Flag == 0)
		return 0;

	if (pSCurvePosCtrl->Time >= pSCurvePosCtrl->AccTime + pSCurvePosCtrl->MidTime + pSCurvePosCtrl->DecTime)
		return 1;

	for (int i = 0; i < pSCurvePosCtrl->Precision; i++)
	{
		if (pSCurvePosCtrl->Time < pSCurvePosCtrl->AccTime && pSCurvePosCtrl->Time >= 0)
			real_acc = pSCurvePosCtrl->Acc;
		else if (pSCurvePosCtrl->Time < pSCurvePosCtrl->AccTime + pSCurvePosCtrl->MidTime && pSCurvePosCtrl->Time >= pSCurvePosCtrl->AccTime)
			real_acc = 0;
		else if (pSCurvePosCtrl->Time < pSCurvePosCtrl->AccTime + pSCurvePosCtrl->MidTime + pSCurvePosCtrl->DecTime && pSCurvePosCtrl->Time >= pSCurvePosCtrl->AccTime + pSCurvePosCtrl->MidTime)
			real_acc = -pSCurvePosCtrl->Dec;
		else
			real_acc = 0;

		pSCurvePosCtrl->Spd += real_acc * pSCurvePosCtrl->TimeInc / pSCurvePosCtrl->Precision;
		pSCurvePosCtrl->Pos += pSCurvePosCtrl->Spd * pSCurvePosCtrl->TimeInc / pSCurvePosCtrl->Precision;
		pSCurvePosCtrl->PosOutput = pSCurvePosCtrl->Pos;

		pSCurvePosCtrl->Time += pSCurvePosCtrl->TimeInc / pSCurvePosCtrl->Precision;
	}

	return 2;
}

/**
* @brief  S曲线速度控制设定新速度
* @param
*		*pSCurveSpdCtrl	S曲线速度控制结构体指针
*		time_inc			每次执行的离散时间增量	[s]
*		acc				加速段加速度（绝对值）
*		dec				减速段加速度（绝对值）
*		target_spd		目标速度
* @retval
*
*/
void SCurveSpdCtrl_New(SCurveSpdCtrl_t *pSCurveSpdCtrl, float time_inc, float acc, float dec, float target_spd)
{
	pSCurveSpdCtrl->Flag = 0;
	pSCurveSpdCtrl->TimeInc = time_inc;
	pSCurveSpdCtrl->Acc = acc;
	pSCurveSpdCtrl->Dec = dec;
	pSCurveSpdCtrl->TargetSpd = target_spd;

	pSCurveSpdCtrl->Spd = 0.0f;

	pSCurveSpdCtrl->SpdOutput = 0.0f;
}

/**
* @brief  S曲线速度控制设定新速度
* @param
*		*pSCurveSpdCtrl	S曲线速度控制结构体指针
*		target_spd		目标速度
* @retval
*
*/
void SCurveCtrl_SetNewSpd(SCurveSpdCtrl_t *pSCurveSpdCtrl, float target_spd)
{
	pSCurveSpdCtrl->TargetSpd = target_spd;
}

/**
* @brief  S曲线速度控制计算
* @param
*		*pSCurveSpdCtrl:	S曲线速度控制结构体指针
* @retval
*		0	标志位没有置位，本次不进行计算
*		1	已到达目标速度
*		2	速度计算完成
*/
int SCurveSpdCtrl_Calc(SCurveSpdCtrl_t *pSCurveSpdCtrl)
{
	if (pSCurveSpdCtrl->Flag == 0)
		return 0;

	if (pSCurveSpdCtrl->Spd - pSCurveSpdCtrl->TargetSpd < -0.01f)
	{
		if (pSCurveSpdCtrl->Spd + pSCurveSpdCtrl->Acc * pSCurveSpdCtrl->TimeInc <= pSCurveSpdCtrl->TargetSpd)
			pSCurveSpdCtrl->Spd += pSCurveSpdCtrl->Acc * pSCurveSpdCtrl->TimeInc;
		else
			return 1;
		pSCurveSpdCtrl->SpdOutput = pSCurveSpdCtrl->Spd;
	}
	else if (pSCurveSpdCtrl->Spd - pSCurveSpdCtrl->TargetSpd > 0.01f)
	{
		if (pSCurveSpdCtrl->Spd - pSCurveSpdCtrl->Dec * pSCurveSpdCtrl->TimeInc >= pSCurveSpdCtrl->TargetSpd)
			pSCurveSpdCtrl->Spd -= pSCurveSpdCtrl->Dec * pSCurveSpdCtrl->TimeInc;
		else
			return 1;
		pSCurveSpdCtrl->SpdOutput = pSCurveSpdCtrl->Spd;
	}
	else
	{
		return 1;
	}

	return 2;
}
