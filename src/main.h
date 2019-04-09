#ifndef _MAIN_H
#define _MAIN_H

#include <sys/time.h>
#include <sys/unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "tasks.h"

//#define __SERVO_ROBOT

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

#endif
