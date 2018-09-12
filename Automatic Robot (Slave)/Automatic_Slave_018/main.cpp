/*
 * I2C Communication (Slave).cpp
 *
 * Created: 6/8/2018 10:58:21 AM
 * Author : Prakash Chaudhary
 */ 

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "TWI.h"
#include "uart.h"
#include "Motor.h"
#include "encoder.h"
#include "pid.h"

Motor m1(1),m2(2),m3(3),m4(4);
encoder e1,e2,e3,e4;
extern pid ma,mb,mc,md;
	
///Tuning parameter//
/*
	first time when installed suspension without communicaiton
	motor1 -> 2.6, 0 , 3
	motor2 -> 2.2, 0 , 4
	motor3 -> 1.9, 0 , 6
	motor4 -> 1.95, 0 , 6
*/

;
char data = 0;

void doall(void){

UART3TransmitData(ma.kp * 100);
UART3TransmitString(" ");
UART3TransmitData(ma.ki * 100);
UART3TransmitString(" ");
UART3TransmitData(ma.kd * 2);
UART3TransmitString(" ");

UART3TransmitData(mb.kp * 100);
UART3TransmitString(" ");
UART3TransmitData(mb.ki * 100);
UART3TransmitString(" ");
UART3TransmitData(mb.kd * 2);
UART3TransmitString(" ");

UART3TransmitData(mc.kp * 100);
UART3TransmitString(" ");
UART3TransmitData(mc.ki * 100);
UART3TransmitString(" ");
UART3TransmitData(mc.kd * 2);
UART3TransmitString(" ");

UART3TransmitData(md.kp * 100);
UART3TransmitString(" ");
UART3TransmitData(md.ki * 100);
UART3TransmitString(" ");
UART3TransmitData(md.kd * 2);

UART3TransmitString("\r\n");
}

int main(void)
{
 	PCICR |= (1<<PCIE0);
 	PCMSK0 |= (1<<PCINT4);
	
	//initUART0();
	initUART2();
	//initUART3();

	sei();
	
	ma.setpid(1.25,0.004,6.5);//(1.48,0,0.50);//(0.75,0,0.1);//(0,0,0);//(2.6,0,3); 1,0,6
	mb.setpid(1.30,0.003,6.5);//(1.53,0,0.50);//(0.8,0,0.1);//(0,0,0);//(2.2,0,4); 1,0,6
	mc.setpid(1.25,0.004,6.0);//(1.33,0,0.70);//(0.74,0,0.1);//(0,0,0);//(1.9,0,6); 1.45,0,2
	md.setpid(1.10,0.003,5.5);//(1.33,0,0.70);//(0.75,0,0.1);//(0,0,0);//(1.95,0,6);	1,0,3
	
	while (1)
 	{
 		//UART0TransmitData(rcvdata[0]);
 		//UART0TransmitString("  ");
 		//UART0TransmitData(e1.getspeed());
 		//UART0TransmitString("\t");
 		//UART0TransmitData(rcvdata[1]);
 		//UART0TransmitString("  ");
 		//UART0TransmitData(e2.getspeed());
 		//UART0TransmitString("\t");
 		//UART0TransmitData(rcvdata[2]);
 		//UART0TransmitString("  ");
 		//UART0TransmitData(e3.getspeed());
 		//UART0TransmitString("\t");
 		//UART0TransmitData(rcvdata[3]);
 		//UART0TransmitString("  ");
 		//UART0TransmitData(e4.getspeed());
 		//UART0TransmitString("\r\n");
		computePid();
	}
}
 ISR(PCINT0_vect)
 {
	stopDrive();
}



////////////////////To tune motor/////////////////////////////

///////////To tune Motor/////////////////////
