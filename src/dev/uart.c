#include "uart.h"

/*
 * serialOpen:
 *	Open and initialise the serial port, setting all the right
 *	port parameters - or as many as are required - hopefully!
 *********************************************************************************
 */

int serialOpen(const char *device, const int baud)
{
	struct termios options;
	speed_t myBaud;
	int status, fd;

	switch (baud)
	{
	case 50:
		myBaud = B50;
		break;
	case 75:
		myBaud = B75;
		break;
	case 110:
		myBaud = B110;
		break;
	case 134:
		myBaud = B134;
		break;
	case 150:
		myBaud = B150;
		break;
	case 200:
		myBaud = B200;
		break;
	case 300:
		myBaud = B300;
		break;
	case 600:
		myBaud = B600;
		break;
	case 1200:
		myBaud = B1200;
		break;
	case 1800:
		myBaud = B1800;
		break;
	case 2400:
		myBaud = B2400;
		break;
	case 4800:
		myBaud = B4800;
		break;
	case 9600:
		myBaud = B9600;
		break;
	case 19200:
		myBaud = B19200;
		break;
	case 38400:
		myBaud = B38400;
		break;
	case 57600:
		myBaud = B57600;
		break;
	case 115200:
		myBaud = B115200;
		break;
	case 230400:
		myBaud = B230400;
		break;
	case 460800:
		myBaud = B460800;
		break;
	case 500000:
		myBaud = B500000;
		break;
	case 576000:
		myBaud = B576000;
		break;
	case 921600:
		myBaud = B921600;
		break;
	case 1000000:
		myBaud = B1000000;
		break;
	case 1152000:
		myBaud = B1152000;
		break;
	case 1500000:
		myBaud = B1500000;
		break;
	case 2000000:
		myBaud = B2000000;
		break;
	case 2500000:
		myBaud = B2500000;
		break;
	case 3000000:
		myBaud = B3000000;
		break;
	case 3500000:
		myBaud = B3500000;
		break;
	case 4000000:
		myBaud = B4000000;
		break;

	default:
		return -2;
	}

	if ((fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK)) == -1)
		return -1;

	fcntl(fd, F_SETFL, O_RDWR);

	// Get and modify current options:

	tcgetattr(fd, &options);

	cfmakeraw(&options);
	cfsetispeed(&options, myBaud);
	cfsetospeed(&options, myBaud);

	options.c_cflag |= (CLOCAL | CREAD);
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	options.c_oflag &= ~OPOST;

	options.c_cc[VMIN] = 0;
	options.c_cc[VTIME] = 100; // Ten seconds (100 deciseconds)

	tcsetattr(fd, TCSANOW, &options);

	ioctl(fd, TIOCMGET, &status);

	status |= TIOCM_DTR;
	status |= TIOCM_RTS;

	ioctl(fd, TIOCMSET, &status);

	usleep(10000); // 10mS

	return fd;
}

/*
 * serialFlush:
 *	Flush the serial buffers (both tx & rx)
 *********************************************************************************
 */

void serialFlush(const int fd)
{
	tcflush(fd, TCIOFLUSH);
}

/*
 * serialClose:
 *	Release the serial port
 *********************************************************************************
 */

void serialClose(const int fd)
{
	close(fd);
}

/*
 * serialPutchar:
 *	Send a single character to the serial port
 *********************************************************************************
 */

void serialPutchar(const int fd, const uint8_t c)
{
	write(fd, &c, 1);
}

/*
 * serialPuts:
 *	Send a string to the serial port
 *********************************************************************************
 */

void serialPuts(const int fd, const char *s)
{
	write(fd, s, strlen(s));
}

/*
 * serialPrintf:
 *	Printf over Serial
 *********************************************************************************
 */

void serialPrintf(const int fd, const char *message, ...)
{
	va_list argp;
	char buffer[1024];

	va_start(argp, message);
	vsnprintf(buffer, 1023, message, argp);
	va_end(argp);

	serialPuts(fd, buffer);
}

/*
 * serialDataAvail:
 *	Return the number of bytes of data avalable to be read in the serial port
 *********************************************************************************
 */

int serialDataAvail(const int fd)
{
	int result;

	if (ioctl(fd, FIONREAD, &result) == -1)
		return -1;

	return result;
}

/*
 * serialGetchar:
 *	Get a single character from the serial device.
 *	Note: Zero is a valid character and this function will time-out after
 *	10 seconds.
 *********************************************************************************
 */

int serialGetchar(const int fd)
{
	uint8_t x;

	if (read(fd, &x, 1) != 1)
		return -1;

	return ((int)x) & 0xFF;
}

/**
 * @brief	send a group of bytes through serial port
 * @para	fd:	file descriptor
 * 			arr_len:size of array
 *         	*arr:	pointer of number array
 * 			end_symbol: to decide whether to send a line break 
 * @retval	
 */
void serialSendByteArr(int fd, int arr_len, uint8_t *arr, int end_symbol)
{
	for (int i = 0; i < arr_len; i++)
	{
		serialPutchar(fd, arr[i]);
	}
	if (end_symbol)
	{
		serialPutchar(fd, 0xAA);
		serialPutchar(fd, 0x55);
	}
}

/**
 * @brief	send a group of float numbers through serial port
 * @para	fd:	file descriptor
 * 			arr_len:size of array
 *         	*arr:	pointer of number array
 * 			end_symbol: to decide whether to send a line break 
 * @retval	
 */
void serialSendFloatArr(int fd, int arr_len, float *arr, int end_symbol)
{
	float_uint8_t temp;
	uint8_t buff[4];
	for (int i = 0; i < arr_len; i++)
	{
		temp.fl = arr[i];
		for (int k = 0; k < 4; k++)
		{
			buff[k] = temp.u8[3 - k];
			//printf("%x \t", buff[k]);
		}
		serialSendByteArr(fd, 4, buff, 0);
		//printf("\n");
	}
	if (end_symbol)
	{
		serialPutchar(fd, 0xAA);
		serialPutchar(fd, 0x55);
	}
}

/**
 * @brief	receive the data from serial port
 * @para	*pFrame:	Data type - Data num - Data group
 * 			fd:	file descriptor
 * 			frame_head: defined by user, to judge the head of a frame
 * @retval	
 */
int serialRevFrame(serial_frame_t *pFrame, int fd, uint16_t frame_head)
{
	uint16_uint8_t temp_head;
	serial_frame_t rev;

	if ((serialDataAvail(fd)) > 0)
	{
		temp_head.u8[1] = serialGetchar(fd);
		temp_head.u8[0] = serialGetchar(fd);
		for (int i = 0; i < 32; i++)
		{
			//printf("%x\n",temp_head.u16);
			if (temp_head.u16 == frame_head) //receive the start byte, set start flag
			{
				uint8_t counter = 0;
				rev.type = serialGetchar(fd);
				rev.num = serialGetchar(fd);
				rev.pdata = (char *)malloc(sizeof(uint8_t) * rev.num);
				while (1)
				{
					*(rev.pdata + counter) = serialGetchar(fd);
					if (counter == (rev.num - 1))
					{
						*pFrame = rev;
						if ((serialDataAvail(fd) > 64)) //To avoid the blocking of the buffer.
							serialFlush(fd);
						return 0;
					}
					counter++;
				}
			}
			else
			{
				temp_head.u8[1] = temp_head.u8[0];
				temp_head.u8[0] = serialGetchar(fd);
			}
		}
		serialFlush(fd);
		return -1;
	}
	else
	{
		return -2;
	}
}

/**
 * @brief	send the head of a frame
 * @para	fd:	file descriptor
 * 		frame_head: defined by user, to judge the head of a frame
 * @retval	
 */
void serialSendFrameHead(int fd, uint16_t frame_head)
{
	uint32_uint8_t temp_head;
	temp_head.u32 = frame_head;
	serialPutchar(fd, temp_head.u8[1]);
	serialPutchar(fd, temp_head.u8[0]);
}

/**
 * @brief	display all frame
 * @para	fd:	file descriptor
 * 		frame_head: defined by user, to judge the head of a frame
 * @retval	
 */
void serialTest(int fd, uint16_t frame_head)
{
	int i = 0, j = 0;
	int rev;
	serial_frame_t sFrame;
	while (1)
	{
		rev = serialRevFrame(&sFrame, fd, frame_head);
		if (!rev)
		{
			printf("\nFrame.type:%x\n", sFrame.type);
			printf("Frame.num:%d\n", sFrame.num);
			for (i = 0; i < sFrame.num; i++)
			{
				if (i % 4 == 0 && i != 0)
					printf("\n");
				printf("%x\t", sFrame.pdata[i]);
			}
			free(sFrame.pdata);
		}
		else
		{
			/*if (j == 0)
				printf("\nWait Frame. -res:%d\n", rev);
			j = 1;*/
		}
	}
}
