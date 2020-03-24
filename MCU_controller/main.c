/*
 * MCU_controller.c
 *
 * Created: 24-03-2020 10:37:59
 * Author : Køb En Windows
 */ 

#include <avr/io.h>
#include "UART1_IO.h"


int main(void)
{
    initUART1();
	
	unsigned char data[8];
	
    while (1) 
    {
		getMsgUART1(data);
    }
}

