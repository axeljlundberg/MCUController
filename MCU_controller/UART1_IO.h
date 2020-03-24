/*
 * UART1_IO.h
 *
 * Created: 11-Jun-19 9:27:26 AM
 *  Author: teamv
 */ 


#ifndef UART1_IO_H_
#define UART1_IO_H_

#include <avr/io.h>

#define RX_BUFFER_SIZE 1024
#define F_CPU 16000000
#define  BAUD_LABVIEW 115200
#define BRC_LABVIEW ((F_CPU/8/BAUD_LABVIEW)-1) //full duplex

enum RESULT{ERROR,
	SUCCES,
	TO_SHORT,
	NO_MSG,
	NO_STARTBYTE,
	MSG_LENGHT_ERROR,
	CHECK_SUM_ERROR};

extern void initUART1();
extern void transmitLVData(unsigned char type, unsigned char data[], unsigned int dataLength);
extern void transmitLVOscilloscope(char type, uint8_t *data, unsigned int dataLength);
extern enum RESULT getMsgUART1(unsigned char data[]);

extern uint8_t getBytesInBuffer();


#endif /* UART1_IO_H_ */