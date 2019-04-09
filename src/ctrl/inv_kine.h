#ifndef _INV_KINE_H
#define _INV_KINE_H

#include "../usr_lib/matrix.h"
#include "para_ctrl.h"
#include "../main.h"

Matrix_t InvKineCalc(uint8_t leg_index, float pos_x, float pos_y, float pos_z);

#endif
