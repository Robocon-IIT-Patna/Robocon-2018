/*
 * Automatic Robot.cpp
 *
 * Created: 5/12/2018 11:45:06 AM
 * Author : Prakash Chaudhary
 */ 

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "Motor.h"
#include "drive.h"
#include "encoder.h"
#include "uart.h"
#include "pid.h"
#include "headers.h"
#include "zonedrive.h"

#define UART0BAUD	9600
#define UART2BAUD	9600
#define UART3BAUD	9600

/////////////	Objects //////////////////
encoder encoderX,encoderY,e1,e2,e3,e4;
Motor m1(1) , m2(2) , m3(3) , m4(4);
Linetracker ltSensorX(1),ltSensorY(0);
extern bodyPid compass,driveX,driveY;
/////////////////////////////////////////

char rcvdata;
bool gofront = false;
bool goback = false;

volatile int svalue =0;

int main(void)
{
	sei();
	compass.Set_Max_Min_Output(150,0);
	uart0_init(UART_BAUD_SELECT(UART0BAUD,F_CPU));
	uart2_init(UART_BAUD_SELECT(UART2BAUD,F_CPU));
	uart3_init(UART_BAUD_SELECT(UART3BAUD,F_CPU));
	//ltSensorX.initialise();
	//ltSensorY.initialise();
	initializeData();
	while (1)
	{
		if(uart2_available())
		{
			Compass_angle = 2*uart2_getc();
		}
		
		calculateCompassPID();
		velocity_robot[0] = 0;
		velocity_robot[1] = 0;
		calculatevel();
		computePid();
		
		rcvdata = uart3_getc();
		
		if (rcvdata != '\0')
		{
			
			switch(rcvdata)
			{
				case 'p':
				compass.incrkp();
				break;
				case 'o':
				compass.dcrkp();
				break;
				case 'i':
				compass.incrki();
				break;
				case 'u':
				compass.dcrki();
				break;
				case 'd':
				compass.incrkd();
				break;
				case 's':
				compass.dcrkd();
				break;
				case 'f':
				compass.incrSetpoint();
				break;
				case 'g':
				compass.dcrSetpoint();
				break;
				case 'v':
				svalue++;
				break;
				case 'c':
				svalue--;
				break;
			}
			
			uart3_puts("KP : ");
			uart3_putint(compass.kp*10);
			uart3_puts("\t KD : ");
			uart3_putint(compass.kd*10);
			uart3_puts("\t KI : ");
			uart3_putint(compass.ki*100);
			uart3_puts("\t SP : ");
			uart3_putint(compass.SETPOINT);
			uart3_puts("\t Angle : ");
			uart3_putint(Compass_angle);
			uart3_puts("\t Out : ");
			uart3_putint(velocity_robot[2]);
			uart3_puts("\r\n");
		}
		
		uart0_putint(compass.SETPOINT);
		uart0_puts("\t");
		uart0_putint(Compass_angle);
		uart0_puts("\r\n");
		//uart3_putint(Compass_angle);
		//if ( Compass_angle & UART_FRAME_ERROR )
		//{
		//uart3_puts("\nFrame error \r\n");
		//}
		//if ( Compass_angle & UART_OVERRUN_ERROR )
		//{
		//uart3_puts("\nOver run error \r\n");
		//}
		//if ( Compass_angle & UART_BUFFER_OVERFLOW )
		//{
		//uart3_puts("\nBuffer overflow \r\n");
		//}
		//
		//}
		//else
		//{
		//uart3_puts("no data available\r\n");
		//}
		
	}
}

// x speed - velocity_robot[0]
//y speed - velocity_robot[1]
//ltSensorX.getsensordata
//ltSensorY.
//e1.
//e2.
//e3.
//e4.
//encoderX.getdistance()
//encoderY.getdistance()
//movx(distance, direction)
//movy(distance, direction) direction = Front, Back _ distance = 2000
