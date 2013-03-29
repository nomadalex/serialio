/**
 * @file  serialio.h
 * @author Kun Wang <ifreedom.cn@gmail.com>
 * @date 2013/03/27 02:00:22
 *
 *  Copyright  2013  Kun Wang <ifreedom.cn@gmail.com>
 */

#ifndef _SERIALIO_H
#define _SERIALIO_H

#include <stdint.h>

#if defined (_WIN32) || defined( _WIN64)
#define SIO_WIN
#else
#define SIO_LINUX
#endif

#ifdef SIO_WIN
#ifdef SIO_EXPORT
#define SIO_API __declspec(dllexport)
#else
#define SIO_API __declspec(dllimport)
#endif
#else
#define SIO_API
#endif

typedef int SIObool;
typedef int SIOenum;

#ifndef TRUE
#define TRUE  1
#endif
#ifndef FALSE
#define FALSE 0
#endif

typedef struct serialio_s SIO_t;
typedef struct serialio_arg_s
{
	int baudrate;
	SIOenum dataBit;
	SIOenum parType;
	SIOenum stopBit;
	SIOenum flowType;
} SIO_arg_t;

#define SIO_SUCCESS 0

/* errors */
#define SIO_ERR_UNKNOWN          -1
#define SIO_ERR_UNSUPPORT        -2
#define SIO_ERR_OPEN_FAILED      -3
#define SIO_ERR_WRONG_ARG        -4
#define SIO_ERR_ALREADY_OPENED   -5
#define SIO_ERR_NOT_OPENED       -6

/* data bit */
#define SIO_EDATA_5  0x1
#define SIO_EDATA_6  0x2
#define SIO_EDATA_7  0x3
#define SIO_EDATA_8  0x4

/* parity */
#define SIO_EPAR_NONE  0x10
#define SIO_EPAR_ODD   0x11
#define SIO_EPAR_EVEN  0x12
#define SIO_EPAR_MARK  0x13
#define SIO_EPAR_SPACE 0x14

/* stop bit */
#define SIO_ESTOP_1    0x20
#define SIO_ESTOP_1_5  0x21
#define SIO_ESTOP_2    0x22

/* flow control */
#define SIO_EFLOW_NONE 0x30
#define SIO_EFLOW_HARD 0x31
/* soft flow current unsupported */
#define SIO_EFLOW_SOFT 0x32

/* for specified bit */
#define SIO_EBIT_CTS   0x40
#define SIO_EBIT_DSR   0x41
#define SIO_EBIT_DTR   0x42
#define SIO_EBIT_RTS   0x43

#ifdef __cplusplus
extern "C" {
#endif

	/* baudrate specified in int value, like 300, 1200 and so on */
	SIO_API void SIO_setArg(SIO_arg_t* arg, int baudrate, SIOenum dataBit, SIOenum parType, SIOenum stopBit, SIOenum flowType);


	SIO_API SIO_t* SIO_create();
	SIO_API void SIO_release(SIO_t* io);

	/* return 0 success, negative is an error code */
	SIO_API SIOenum SIO_open(SIO_t* io, const char* port, const SIO_arg_t* arg);
	SIO_API SIOenum SIO_close(SIO_t* io);

	/* return actual perform count, if negative, then it is an error code */
	SIO_API int SIO_read(SIO_t* io, uint8_t* buf, int size);
	SIO_API int SIO_write(SIO_t* io, const uint8_t* buf, int size);

	/* specified bit test & set & reset */
	SIO_API SIOenum SIO_isSetBit(SIO_t* io, SIOenum bit, SIObool* value);
	SIO_API SIOenum SIO_setBit(SIO_t* io, SIOenum bit);
	SIO_API SIOenum SIO_resetBit(SIO_t* io, SIOenum bit);

#ifdef __cplusplus
}
#endif

#endif /* _SERIALIO_H */
