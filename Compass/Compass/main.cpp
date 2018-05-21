/*
 * Compass.cpp
 *
 * Created: 1/10/2018 8:22:09 PM
 * Author : Prakash Chaudhary
 */ 
//
#define F_CPU 16000000UL
//#define RESTART_BYTE 0xFF

#include <avr/io.h>
#include <avr/interrupt.h>

//#include "UART9bit.h"
#include "uart.h"
#include "Compass.h"
#include <util/delay.h>
Compass C1;

int main(void)
{	
	sei();
	uart0_init(UART_BAUD_SELECT(9600,F_CPU));
	//UART_Initialize_9bit();
	C1.init_compass();
    /* Replace with your application code */
    while (1) 
    {
		C1.read_Compass();
		uart0_putc(uint8_t(C1.Get_Angle()/2));
		//UART_Transmit_9bit(257);
		_delay_ms(15);
    }
}