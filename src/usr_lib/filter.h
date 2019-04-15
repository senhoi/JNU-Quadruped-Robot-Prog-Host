#ifndef FILTER_H
#define FILTER_H

void LowPassFilter_RC_1order(float *Vi, float *Vo, float *Vo_p, float sampleFrq, float cutFrq);

#endif
