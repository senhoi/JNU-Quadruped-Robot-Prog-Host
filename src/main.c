#include "main.h"
#ifdef __SERVO_ROBOT
#include <wiringPi.h>
#endif

#define CALC_CYCLE_US 10000

extern int usleep(__useconds_t __useconds);

static uint64_t sys_timer_us()
{
	struct timeval tv;

	gettimeofday(&tv, NULL);

	return tv.tv_sec * 1000 * 1000 + tv.tv_usec;
}

static void handle_IRQ(void)
{
	InterruptTask();
}

int main(int argc, char *argv[])
{
	uint64_t time_begin_us, time_end_us;
	int64_t time_diff_us;

	if (argc == 1)
	{
		if (strcmp((const char *)argv, "--servo"))
			sRobot_MotionPara.Structure = ELBOW_KNEE;
		else if (strcmp((const char *)argv, "--actr"))
			sRobot_MotionPara.Structure = ELBOW_ELBOW;
		else
			PRINTF_TIPS("Wrong parameters, please input 'servo' or 'actr'\n");
	}
	else
	{
		PRINTF_TIPS("Too many parameters, please input 'servo' or 'actr'\n");
	}

	InitTask();

#ifdef __SERVO_ROBOT
	wiringPiSetup();
	wiringPiISR(1, INT_EDGE_FALLING, &handle_IRQ);
#endif

	PRINTF_TIPS("Successfully Initialized");

	while (1)
	{
#ifndef __SERVO_ROBOT
		time_begin_us = sys_timer_us();
		handle_IRQ();
		time_end_us = sys_timer_us();

		time_diff_us = time_end_us - time_begin_us;

		if (time_diff_us < CALC_CYCLE_US)
			usleep(CALC_CYCLE_US - time_diff_us);
		else
			PRINTF_WARNING("Execute Overtime, -beyond_us:\t%ld", time_diff_us);
#endif
		LowPriorityTask();
	}
}
