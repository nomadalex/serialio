serialio
========

serial port library for win &amp; linux, in C.

## API:

### enums:

	SIO_SUCCESS

	/* errors */
    SIO_ERR_UNKNOWN            unknown error
    SIO_ERR_UNSUPPORT
	SIO_ERR_OPEN_FAILED
	SIO_ERR_WRONG_ARG
	SIO_ERR_ALREADY_OPENED
	SIO_ERR_NOT_OPENED

	/* data bit */
	SIO_EDATA_5
	SIO_EDATA_6
	SIO_EDATA_7
	SIO_EDATA_8

	/* parity */
	SIO_EPAR_NONE
	SIO_EPAR_ODD
	SIO_EPAR_EVEN
	SIO_EPAR_MARK
	SIO_EPAR_SPACE

	/* stop bit */
	SIO_ESTOP_1
	SIO_ESTOP_1_5
	SIO_ESTOP_2

	/* flow control */
	SIO_EFLOW_NONE
	SIO_EFLOW_HARD
	SIO_EFLOW_SOFT               soft flow control current not support

	/* for specified bit */
	SIO_EBIT_CTS
	SIO_EBIT_DSR
	SIO_EBIT_DTR
	SIO_EBIT_RTS


### functions:

	/* baudrate specified in int value, like 300, 1200 and so on */
	void SIO_setArg(SIO_arg_t* arg, int baudrate, SIOenum dataBit, SIOenum parType, SIOenum stopBit, SIOenum flowType);


	SIO_t* SIO_create();
	void SIO_release(SIO_t* io);

	/* return 0 success, negative is an error code */
	SIOenum SIO_open(SIO_t* io, const char* port, const SIO_arg_t* arg);
	SIOenum SIO_close(SIO_t* io);

	/* return actual perform count, if negative, then it is an error code */
	int SIO_read(SIO_t* io, uint8_t* buf, int size);
	int SIO_write(SIO_t* io, const uint8_t* buf, int size);

	/* specified bit test & set & reset */
	SIOenum SIO_isSetBit(SIO_t* io, SIOenum bit, SIObool* value);
	SIOenum SIO_setBit(SIO_t* io, SIOenum bit);
	SIOenum SIO_resetBit(SIO_t* io, SIOenum bit);
