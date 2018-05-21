/*
 * Data_send.h
 *
 * Created: 5/21/2018 9:12:01 PM
 *  Author: Subash Timilsina
 */ 


#ifndef DATA_SEND_H_
#define DATA_SEND_H_

#define DATA_DIR_LOW	DDRD
#define DATA_DIR_HIGH	DDRB
#define PORT_LOW		PORTD
#define PORT_HIGH		PORTB
#define DATA_LOW		PIND	//lower bits to PORTD
//#define DATA_MSB		PINC		//msb PINB0

void init_communication()
{
	 DATA_DIR_LOW = 0XFF;
	 DATA_DIR_HIGH |= 0X01;	//setting pins as output
}

void Send_data(uint16_t data)
{
	PORTB &= ~(1<<PB1);
	PORT_LOW = data;
	PORT_HIGH = ((data >> 8)|(PORT_HIGH & 0xFE));	
	PORTB |= (1<<PINB1);	
}


#endif /* DATA_SEND_H_ */