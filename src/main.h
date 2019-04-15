#ifndef _MAIN_H
#define _MAIN_H

#include <sys/time.h>
#include <sys/unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>

#include "tasks.h"

//#define __SERVO_ROBOT
extern FILE *fd_log;

#define PRINTF_ERROR(contents, args...) \
	{                                   \
		printf("[ERROR]:");             \
		printf(contents, ##args);       \
		printf("\n");                   \
	}

#define PRINTF_WARNING(contents, args...) \
	{                                     \
		printf("[WARNING]:");             \
		printf(contents, ##args);         \
		printf("\n");                     \
	}

#define PRINTF_TIPS(contents, args...) \
	{                                  \
		printf("[TIPS]:");             \
		printf(contents, ##args);      \
		printf("\n");                  \
	}

#define DEBUG(contents, args...)  \
	{                             \
		printf("[DEBUG]:");       \
		printf(contents, ##args); \
		printf("\n");             \
	}

#define LOG(contents, args...)             \
	{                                      \
		fprintf(fd_log, contents, ##args); \
		fprintf(fd_log, "\n");             \
	}

#endif

#define GYRO_LOG
