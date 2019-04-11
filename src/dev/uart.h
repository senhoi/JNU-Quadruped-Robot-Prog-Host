#ifndef __UART_H__
#define __UART_H__

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

typedef union float_uint8_t {
	float fl;
	uint8_t u8[4];
} float_uint8_t;

typedef union uint16_uint8_t {
	uint16_t u16;
	uint8_t u8[2];
} uint16_uint8_t;

typedef union uint32_uint8_t {
	uint32_t u32;
	uint8_t u8[4];
} uint32_uint8_t;

typedef struct serial_frame_t
{
	uint8_t type;
	uint8_t num;
	uint8_t *pdata;
} serial_frame_t;

extern int serialOpen(const char *device, const int baud);
extern void serialClose(const int fd);
extern void serialFlush(const int fd);
extern void serialPutchar(const int fd, const unsigned char c);
extern void serialPuts(const int fd, const char *s);
extern void serialPrintf(const int fd, const char *message, ...);
extern int serialDataAvail(const int fd);
extern int serialGetchar(const int fd);

extern void serialSendByteArr(int fd, int arr_len, uint8_t *arr, int end_symbol);
extern void serialSendFloatArr(int fd, int arr_len, float *arr, int end_symbol);
extern int serialRevFrame(serial_frame_t *pFrame, int fd, uint16_t frame_head);
extern void serialSendFrameHead(int fd, uint16_t frame_head);
extern void serialTest(int fd, uint16_t frame_head);

#endif
