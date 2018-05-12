/* 
* Linetracker.cpp
*
* Created: 12/24/2017 1:42:17 PM
* Author: Subash Timilsina
* Edited: TopsyKreet
*/




#include "Linetracker.h"

Linetracker::Linetracker(int addr) // 0 to 255
{
	address = addr;
	junction_detect = false;
	junction_count = 0;
}

void Linetracker::initialise()
{
	sei();
	if(address == 1){
		//initUART2();	//initialise communication
		INPUT(JUNCTION_PIN_AXISX);	//To detect junction pin
		CLEAR(JUNCTION_PIN_AXISX);	//Internal Pull down junction pin
	//OUTPUT(UART_ENABLE);		//to receive data uart enable pin as output
	//CLEAR(UART_ENABLE);		//active low pin //always ready to send data so cleared
	}
	else if(address == 0){
		INPUT(JUNCTION_PIN_AXISY);
		CLEAR(JUNCTION_PIN_AXISY);
	}
}

/****************************************optional if junction pulse use interrupt************************************/

void Linetracker::initialise_interrupt() //call this function in initialise
{
	sei();
	PCICR |= (1<<PCIE0);
	PCICR |= (1<<PCIE2);
	if(address == 0)	                                       //JPULSE 1- PCINT4, JPULSE2 - PCINT16
		PCMSK2 |= (1<<PCINT16);
	else if(address == 1)
		PCMSK0 |= (1<<PCINT4);

	/******************************************ISR FOR JUNCTION COUNT*************************************/
	/* put it in main
	*	ISR(INT_JUNC_VECT)
	*	{
	*		objectname.Inc_junc_count();
	*   }
	*/

}

void Linetracker::Off_interrupt()
{
	if(address == 0){
		PCMSK2 &= ~(1<<PCINT16);		//clearing the interrupt pin 
	}
	else if (address == 1){
		PCMSK0 &= ~(1<<PCINT4);
	}
}

/***********************************************************************************************************************/

void Linetracker::send_data(char command, char data)
{
	checksum = address + command + data;
	if(address == 1){
		uart2_putc(address);
		uart2_putc(command);
		uart2_putc(data);
		uart2_putc(checksum);
	}
	else if(address == 0){
		uart3_putc(address);
		uart3_putc(command);
		uart3_putc(data);
		uart3_putc(checksum);
	}
}

/************************************ Setting of LSA08 *********************************/

void Linetracker::Calibrate()	//calibrate
{
	send_data('C',0X00);
}

void Linetracker::Set_Line() //dark line or light line
{
	send_data('L',0X00); //light on 0X00 and dark on 0X01
}

void Linetracker::Set_Line_Threshhold()	//Minimum value to be read
{
	send_data('T',0X04);
}

void Linetracker::Set_Junction_Width()	//from 0 to 8
{
	send_data('J',0X08);		//width 8 Gamefield junction length 600 mm
}

void Linetracker::Set_LCD_Contrast() //set LCD contrast from 0 to 255
{
	send_data('S',0X5A);	//contrast 90
}

void Linetracker::Set_LCD_Backlit() //set lcd backlit level
{
	send_data('B',0x05); //light level 5
}

void Linetracker::Set_Baudrate()	//Set baudrate for communication
{
	send_data('R',0X02);	//38400
}

void Linetracker::Uart_Data_Outputmode()	//calibrate
{
	send_data('D',0X02);		//sensor data from 0X00 left to 0X70 right;
}

void Linetracker::clear_junction()	//clear junction count of LSA08
{
	send_data('X',0X00);
}

/*****************************************************************************************************************************/

int Linetracker::Get_Junc_pincount()		//count from avr using junction pulse  //using polling
{
	if(address == 1){
		if(junction_detect && !(READ(JUNCTION_PIN_AXISX)))
		{
			junction_count++;
			junction_detect = false;
		}
		if(READ(JUNCTION_PIN_AXISX))
		{
			junction_detect = true;
		}
		return junction_count;
	}
	else if(address == 0){
		if(junction_detect && !(READ(JUNCTION_PIN_AXISY)))
		{
			junction_count++;
			junction_detect = false;
		}
		if(READ(JUNCTION_PIN_AXISY))
		{
			junction_detect = true;
		}
		return junction_count;
	}
}

int Linetracker::Get_Sensors_Data()
{
	if(address == 1){
		if(uart2_available()>0){
			rcvdat = uart2_getc();
	}
		return rcvdat;
	}
	else if(address == 0){
		if(uart3_available()>0){
			rcvdat = uart3_getc();
		}
		return rcvdat;
	}
}

int Linetracker::Get_JunctionCount()	//junction count from line tracker
{
	if(address == 1){
		send_data('X',0X01);
		while(!(rcvdat=uart2_getc()));
		return rcvdat;
	}
	else if(address == 0){
		send_data('X',0x01);
		while(!(rcvdat=uart2_getc()));
		return rcvdat;
	}
}

