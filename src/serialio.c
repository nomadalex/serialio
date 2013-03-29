/**
 * @file  serialio.c
 * @author Kun Wang <ifreedom.cn@gmail.com>
 * @date 2013/03/28 06:36:17
 *
 *  Copyright  2013  Kun Wang <ifreedom.cn@gmail.com>
 */

#include "serialio.h"

#include <stdlib.h>
#include <string.h>

#ifdef SIO_WIN
#define STRICT
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

#ifdef SIO_LINUX
#include <sys/types.h>
#include <sys/shm.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#endif

struct serialio_s
{
	SIObool isOpened;

#ifdef SIO_WIN
	HANDLE fd;
#else
	int fd;
	struct termios oldOpt;
#endif
};

void SIO_setArg(SIO_arg_t* arg, int baudrate, SIOenum dataBit, SIOenum parType, SIOenum stopBit, SIOenum flowType)
{
	arg->baudrate = baudrate;

	if (dataBit == 0)
		arg->dataBit = SIO_EDATA_8;
	else
		arg->dataBit = dataBit;

	if (parType == 0)
		arg->parType = SIO_EPAR_NONE;
	else
		arg->parType = parType;

	if (stopBit == 0)
		arg->stopBit = SIO_ESTOP_1;
	else
		arg->stopBit = stopBit;

	if (flowType == 0)
		arg->flowType = SIO_EFLOW_NONE;
	else
		arg->flowType = flowType;
}


SIO_t* SIO_create()
{
	SIO_t* io = (SIO_t*)malloc(sizeof(SIO_t));

	io->isOpened = FALSE;

#ifdef SIO_WIN
	io->fd = INVALID_HANDLE_VALUE;
#else
	io->fd = -1;
	memset(&io->oldOpt, 0, sizeof(struct termios));
#endif

	return io;
}

void SIO_release(SIO_t* io)
{
	if (io->isOpened) SIO_close(io);

	free(io);
}

static SIOenum _openPort(SIO_t* io, const char* port);
static void _closePort(SIO_t* io);
static SIOenum _setArg(SIO_t* io, const SIO_arg_t* arg);

SIOenum SIO_open(SIO_t* io, const char* port, const SIO_arg_t* arg)
{
	SIOenum ret;

	if (io->isOpened) return SIO_ERR_ALREADY_OPENED;
	if (arg == NULL) return SIO_ERR_WRONG_ARG;

	ret = _openPort(io, port);
	if (ret < 0) return ret;

	ret = _setArg(io, arg);
	if (ret < 0)
	{
		_closePort(io);
		return ret;
	}

	io->isOpened = TRUE;

	if (arg->flowType == SIO_EFLOW_NONE)
	{
		SIO_setBit(io, SIO_EBIT_DTR);
		SIO_setBit(io, SIO_EBIT_RTS);
	}

	return SIO_SUCCESS;
}

#define CHECK_OPENED(io)							\
	if (!io->isOpened) return SIO_ERR_NOT_OPENED

SIOenum SIO_close(SIO_t* io)
{
#ifndef SIO_WIN
	int status;
#endif

	CHECK_OPENED(io);

	_closePort(io);

	io->isOpened = FALSE;
	return SIO_SUCCESS;
}

/* return actual perform count, if negative, then it is an error code */
int SIO_read(SIO_t* io, uint8_t* buf, int size)
{
	int n;

	CHECK_OPENED(io);

#ifdef SIO_WIN
	ReadFile(io->fd, buf, size, (LPDWORD)((void *)&n), NULL);
#else
	n = read(io->fd, buf, size);
#endif

	return n;
}

int SIO_write(SIO_t* io, const uint8_t* buf, int size)
{
	int n;

	CHECK_OPENED(io);

#ifdef SIO_WIN
	if(WriteFile(io->fd, buf, size, (LPDWORD)((void *)&n), NULL))
		return n;
	return SIO_ERR_UNKNOWN;
#else
	return write(io->fd, buf, size);
#endif
}

/* specified bit */
SIOenum SIO_isSetBit(SIO_t* io, SIOenum bit, SIObool* value)
{
	int status;

	CHECK_OPENED(io);

	*value = FALSE;

	switch (bit)
	{
#ifdef SIO_WIN
	case SIO_EBIT_CTS:
		GetCommModemStatus(io->fd, (LPDWORD)((void *)&status));
		if(status & MS_CTS_ON) *value = TRUE;
		break;

	case SIO_EBIT_DSR:
		GetCommModemStatus(io->fd, (LPDWORD)((void *)&status));
		if(status & MS_DSR_ON) *value = TRUE;
		break;

#else
	case SIO_EBIT_CTS:
		if(ioctl(io->fd, TIOCMGET, &status) < 0)
			return SIO_ERR_UNKNOWN;

		if(status & TIOCM_CTS) *value = TRUE;
		break;

	case SIO_EBIT_DSR:
		if(ioctl(io->fd, TIOCMGET, &status) < 0)
			return SIO_ERR_UNKNOWN;

		if(status & TIOCM_DSR) *value = TRUE;
		break;

#endif

	default:
		return SIO_ERR_UNSUPPORT;
	}

	return SIO_SUCCESS;
}

SIOenum SIO_setBit(SIO_t* io, SIOenum bit)
{
#ifndef SIO_WIN
	int status;
#endif

	CHECK_OPENED(io);

	switch (bit)
	{
#ifdef SIO_WIN
	case SIO_EBIT_DTR:
		EscapeCommFunction(io->fd, SETDTR);
		break;

	case SIO_EBIT_RTS:
		EscapeCommFunction(io->fd, SETRTS);
		break;

#else
	case SIO_EBIT_DTR:
		if(ioctl(io->fd, TIOCMGET, &status) < 0)
			return SIO_ERR_UNKNOWN;

		status |= TIOCM_DTR;    /* turn on DTR */

		if(ioctl(io->fd, TIOCMSET, &status) < 0)
			return SIO_ERR_UNKNOWN;
		break;

	case SIO_EBIT_RTS:
		if(ioctl(io->fd, TIOCMGET, &status) < 0)
			return SIO_ERR_UNKNOWN;

		status |= TIOCM_RTS;    /* turn on RTS */

		if(ioctl(io->fd, TIOCMSET, &status) < 0)
			return SIO_ERR_UNKNOWN;
		break;

#endif

	default:
		return SIO_ERR_UNSUPPORT;
	}

	return SIO_SUCCESS;
}

SIOenum SIO_resetBit(SIO_t* io, SIOenum bit)
{
#ifndef SIO_WIN
	int status;
#endif

	CHECK_OPENED(io);

	switch (bit)
	{
#ifdef SIO_WIN
	case SIO_EBIT_DTR:
		EscapeCommFunction(io->fd, CLRDTR);
		break;

	case SIO_EBIT_RTS:
		EscapeCommFunction(io->fd, CLRRTS);
		break;

#else
	case SIO_EBIT_DTR:
		if(ioctl(io->fd, TIOCMGET, &status) < 0)
			return SIO_ERR_UNKNOWN;

		status &= ~TIOCM_DTR;    /* turn off DTR */

		if(ioctl(io->fd, TIOCMSET, &status) < 0)
			return SIO_ERR_UNKNOWN;
		break;

	case SIO_EBIT_RTS:
		if(ioctl(io->fd, TIOCMGET, &status) < 0)
			return SIO_ERR_UNKNOWN;

		status &= ~TIOCM_RTS;    /* turn off RTS */

		if(ioctl(io->fd, TIOCMSET, &status) < 0)
			return SIO_ERR_UNKNOWN;
		break;

#endif

	default:
		return SIO_ERR_UNSUPPORT;
	}

	return SIO_SUCCESS;
}

#ifdef SIO_WIN
static SIOenum _openPort(SIO_t* io, const char* port)
{
	io->fd = CreateFileA(port,
						 GENERIC_READ|GENERIC_WRITE,
						 0,                          /* no share  */
						 NULL,                       /* no security */
						 OPEN_EXISTING,
						 0,                          /* no threads */
						 NULL);                      /* no templates */

	if (io->fd == INVALID_HANDLE_VALUE) return SIO_ERR_OPEN_FAILED;

	return SIO_SUCCESS;
}

static void _closePort(SIO_t* io)
{
	CloseHandle(io->fd);
	io->fd = INVALID_HANDLE_VALUE;
}

static SIOenum _setArg(SIO_t* io, const SIO_arg_t* arg)
{
	DCB portDCB;
	COMMTIMEOUTS timeouts;

	memset(&portDCB, 0, sizeof(DCB));
	portDCB.DCBlength = sizeof(DCB);

	if (!GetCommState(io->fd, &portDCB))
		return SIO_ERR_OPEN_FAILED;

	portDCB.BaudRate = arg->baudrate;

	switch (arg->dataBit)
	{
	case SIO_EDATA_5:
		portDCB.ByteSize = 5;
		break;

	case SIO_EDATA_6:
		portDCB.ByteSize = 6;
		break;

	case SIO_EDATA_7:
		portDCB.ByteSize = 7;
		break;

	case SIO_EDATA_8:
		portDCB.ByteSize = 8;
		break;

	default:
		return SIO_ERR_WRONG_ARG;
	}

	switch (arg->parType)
	{
	case SIO_EPAR_NONE:
		portDCB.fParity = FALSE;
		portDCB.Parity = NOPARITY;
		break;

	case SIO_EPAR_ODD:
		portDCB.fParity = TRUE;
		portDCB.Parity = ODDPARITY;
		break;

	case SIO_EPAR_EVEN:
		portDCB.fParity = TRUE;
		portDCB.Parity = EVENPARITY;
		break;

	case SIO_EPAR_MARK:
		portDCB.fParity = TRUE;
		portDCB.Parity = MARKPARITY;
		break;

	case SIO_EPAR_SPACE:
		portDCB.fParity = TRUE;
		portDCB.Parity = SPACEPARITY;
		break;

	default:
		return SIO_ERR_WRONG_ARG;
	}

	switch (arg->stopBit)
	{
	case SIO_ESTOP_1:
		portDCB.StopBits = ONESTOPBIT;
		break;

	case SIO_ESTOP_1_5:
		portDCB.StopBits = ONE5STOPBITS;
		break;

	case SIO_ESTOP_2:
		portDCB.StopBits = TWOSTOPBITS;
		break;

	default:
		return SIO_ERR_WRONG_ARG;
	}

	switch (arg->flowType)
	{
	case SIO_EFLOW_NONE:
		portDCB.fOutxCtsFlow = FALSE;					/* CTS monitoring */
		portDCB.fOutxDsrFlow = FALSE;					/* DSR monitoring */
		portDCB.fDtrControl = DTR_CONTROL_DISABLE;		/* DTR monitoring */
		portDCB.fOutX = FALSE;							/* XON/XOFF for transmission */
		portDCB.fInX = FALSE;							/* XON/XOFF for receiving */
		portDCB.fRtsControl = RTS_CONTROL_DISABLE;		/* RTS (Ready To Send) */
		break;

	case SIO_EFLOW_HARD:
		portDCB.fOutxCtsFlow = TRUE;
		portDCB.fOutxDsrFlow = TRUE;
		portDCB.fDtrControl = DTR_CONTROL_HANDSHAKE;
		portDCB.fOutX = FALSE;
		portDCB.fInX = FALSE;
		portDCB.fRtsControl = RTS_CONTROL_HANDSHAKE;
		break;

	case SIO_EFLOW_SOFT:
		/* portDCB.fOutxCtsFlow = FALSE; */
		/* portDCB.fOutxDsrFlow = FALSE; */
		/* portDCB.fDtrControl = DTR_CONTROL_DISABLE; */
		/* portDCB.fOutX = TRUE; */
		/* portDCB.fInX = TRUE; */
		/* portDCB.fRtsControl = RTS_CONTROL_DISABLE; */
		/* break; */
		return SIO_ERR_UNSUPPORT;

	default:
		return SIO_ERR_WRONG_ARG;
	}

	if(!SetCommState(io->fd, &portDCB))
		return SIO_ERR_WRONG_ARG;

	timeouts.ReadIntervalTimeout         = 0;
	timeouts.ReadTotalTimeoutMultiplier  = 0;
	timeouts.ReadTotalTimeoutConstant    = MAXDWORD;
	timeouts.WriteTotalTimeoutMultiplier = 0;
	timeouts.WriteTotalTimeoutConstant   = MAXDWORD;

	if(!SetCommTimeouts(io->fd, &timeouts))
		return SIO_ERR_OPEN_FAILED;

	return SIO_SUCCESS;
}

#else

#define CLOSE_FD(io)							\
	close(io->fd);								\
	io->fd = -1

static SIOenum _openPort(SIO_t* io, const char* port)
{
	io->fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
	if (io->fd == -1) return SIO_ERR_OPEN_FAILED;

	if (tcgetattr(io->fd, &io->oldOpt) < 0)
	{
		CLOSE_FD(io);
		return SIO_ERR_OPEN_FAILED;
	}

	return SIO_SUCCESS;
}

static void _closePort(SIO_t* io)
{
	int status;

	if (ioctl(io->fd, TIOCMGET, &status) == 0)
	{
		status &= ~TIOCM_DTR;    /* turn off DTR */
		status &= ~TIOCM_RTS;    /* turn off RTS */

		ioctl(io->fd, TIOCMSET, &status);
	}

	tcsetattr(io->fd, TCSANOW, &io->oldOpt);

	CLOSE_FD(io);
}

static SIOenum _setArg(SIO_t* io, const SIO_arg_t* arg)
{
	int status, speed;
	struct termios newOpt;

	memset(&newOpt, 0, sizeof(struct termios));

	newOpt.c_cflag |= CREAD | CLOCAL;

    /* raw mode, here for remember */
	/* newOpt.c_lflag  &= ~(ICANON | ECHO | ECHOE | ISIG); */
	/* newOpt.c_oflag  &= ~OPOST; */

	/* block untill n bytes are received */
	newOpt.c_cc[VMIN] = 0;
	/* block untill a timer expires (n * 100 mSec.) */
	newOpt.c_cc[VTIME] = 0;

	switch (arg->baudrate)
	{
	case 110    : speed=B110; break;
	case 300    : speed=B300; break;
	case 600    : speed=B600; break;
	case 1200   : speed=B1200; break;
	case 2400   : speed=B2400; break;
	case 4800   : speed=B4800; break;
	case 9600   : speed=B9600; break;
	case 19200  : speed=B19200; break;
	case 38400  : speed=B38400; break;
	case 57600  : speed=B57600; break;
	case 115200 : speed=B115200; break;
	default:
		return SIO_ERR_WRONG_ARG;
	}

	cfsetispeed(&newOpt, speed);
	cfsetospeed(&newOpt, speed);

	switch (arg->dataBit)
	{
	case SIO_EDATA_5:
		newOpt.c_cflag |= CS5;
		break;

	case SIO_EDATA_6:
		newOpt.c_cflag |= CS6;
		break;

	case SIO_EDATA_7:
		newOpt.c_cflag |= CS7;
		break;

	case SIO_EDATA_8:
		newOpt.c_cflag |= CS8;
		break;

	default:
		return SIO_ERR_WRONG_ARG;
	}

	switch (arg->parType)
	{
	case SIO_EPAR_NONE:
		newOpt.c_cflag &= ~PARENB;
		newOpt.c_iflag |= IGNPAR;
		break;

	case SIO_EPAR_ODD:
		newOpt.c_cflag |= PARODD | PARENB;
		newOpt.c_iflag |= INPCK;
		break;

	case SIO_EPAR_EVEN:
		newOpt.c_cflag |= PARENB;
		newOpt.c_cflag &= ~PARODD;
		newOpt.c_iflag |= INPCK;
		break;

	case SIO_EPAR_MARK:
		newOpt.c_cflag |= PARENB | PARODD | CMSPAR;
		newOpt.c_iflag |= INPCK;
		break;

	case SIO_EPAR_SPACE:
		newOpt.c_cflag |= PARENB | CMSPAR;
		newOpt.c_cflag &= ~PARODD;
		newOpt.c_iflag |= INPCK;
		break;

	default:
		return SIO_ERR_WRONG_ARG;
	}

	switch (arg->stopBit)
	{
	case SIO_ESTOP_1:
		newOpt.c_cflag &= ~CSTOPB;
		break;

	case SIO_ESTOP_1_5:
		newOpt.c_cflag &= ~CSTOPB;
		break;

	case SIO_ESTOP_2:
		newOpt.c_cflag |= CSTOPB;
		break;

	default:
		return SIO_ERR_WRONG_ARG;
	}

	switch (arg->flowType)
	{
	case SIO_EFLOW_NONE:
		newOpt.c_cflag &= ~CRTSCTS;
		break;

	case SIO_EFLOW_HARD:
		newOpt.c_cflag |= CRTSCTS;
		break;

	case SIO_EFLOW_SOFT:
		/* newOpt.c_cflag |= IXON|IXOFF|IXANY; */
		/* break; */
		return SIO_ERR_UNSUPPORT;

	default:
		return SIO_ERR_WRONG_ARG;
	}

	if (tcsetattr(io->fd, TCSANOW, &newOpt) < 0)
		return SIO_ERR_WRONG_ARG;

	return SIO_SUCCESS;
}
#endif
