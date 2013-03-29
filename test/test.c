/**
 * @file  test.c
 * @author Kun Wang <ifreedom.cn@gmail.com>
 * @date 2013/03/29 02:42:39
 *
 *  Copyright  2013  Kun Wang <ifreedom.cn@gmail.com>
 */

#include "serialio.h"
#include <stdio.h>

int main(int argc, const char* argv[])
{
	SIO_arg_t arg;
	SIO_t* com1;
	uint8_t buf[2];
	int n;

	SIO_setArg(&arg, 1200, SIO_EDATA_7, SIO_EPAR_NONE, SIO_ESTOP_1, SIO_EFLOW_NONE);

	com1 = SIO_create();

	if (SIO_open(com1, "COM1", &arg) < 0)
	{
		printf("COM1 Open Error!\n");
		SIO_release(com1);
		return -1;
	}

	n = SIO_read(com1, buf, 2);
	if (n == 2)
	{
		printf("Read %x, %x", buf[0], buf[1]);
	}
	else
	{
		printf("Read Error!\n");
	}

	SIO_close(com1);
	SIO_release(com1);
	return 0;
}
