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
#include "Parallel_9bit.h"
#include "Compass.h"
#include <util/delay.h>
Compass C1;

int main(void)
{	
	sei();
	C1.init_compass();
	init_communication();
    /* Replace with your application code */
    while (1) 
    {
		C1.read_Compass();
		Send_data(C1.Get_Angle());
		_delay_ms(10);
    }
}