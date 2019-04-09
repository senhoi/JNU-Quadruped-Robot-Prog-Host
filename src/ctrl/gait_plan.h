#ifndef __GAIT_PLAN_H
#define __GAIT_PLAN_H

#include "../usr_lib/matrix.h"
#include "../usr_lib/ploy.h"
#include "para_ctrl.h"
#include "../main.h"

void Calc_GaitTrajPolyCoeffi_Trot(void);
void Calc_GaitTrajPolyCoeffi_Walk(void);
Matrix_t Calc_GaitTrajPoint_Trot(void);
Matrix_t Calc_GaitTrajPoint_Walk(void);

#endif
