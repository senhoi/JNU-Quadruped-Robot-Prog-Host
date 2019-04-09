#ifndef _TASK_H
#define _TASK_H

#include "main.h"
#include "dev_data.h"
#include "dev/uart.h"
#include "dev/priority.h"
#include "usr_lib/matrix.h"
#include "ctrl/para_ctrl.h"
#include "ctrl/gait_plan.h"
#include "ctrl/inv_kine.h"

void InitTask(void);
void InterruptTask(void);
void LowPriorityTask(void);
void DisplayTask(void);
void ParaUpdate(int mode);

#endif
