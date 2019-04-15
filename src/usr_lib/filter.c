#include "filter.h"

/**
  * @brief  implement 1 order RC low pass filter
  *         raw data filtered by a simple RC low pass filter@cufoff=5Hz
  * @param  Vi 		: 	Vi(k)
  * 		Vo 		: 	Vo(k)
  * 		Vo_p 	: 	Vo(k-1)
  * 		sampleFrq:	sample frequency
  * 		cutFrq	:	cutoff frequency
  * @note   This example shows a simple way to report end of conversion
  *         and get conversion result. You can add your own implementation.
  * @retval None
  */
void LowPassFilter_RC_1order(float *Vi, float *Vo, float *Vo_p, float sampleFrq, float cutFrq)
{
	float RC, Cof1, Cof2;

	RC = (float)1.0 / 2.0 / 3.1415f / cutFrq;
	Cof1 = 1 / (1 + RC * sampleFrq);
	Cof2 = RC * sampleFrq / (1 + RC * sampleFrq);

	*Vo = Cof1 * (*Vi) + Cof2 * (*Vo_p);

	//update
	*Vo_p = *Vo;
}
