/*
 * UART9bit.h
 *
 * Created: 4/18/2018 10:23:11 AM
 *  Author: Prakash Chaudhary
 */ 

#ifndef F_CPU
#define F_CPU           16000000UL 
#endif

#include <avr/io.h>
#include <util/delay.h>

const uint32_t Baud_rate = 9600;

inline uint16_t Calculate_UBRR(const uint32_t Baud_dum) { return F_CPU / 16 / Baud_dum - 1; }
inline uint8_t _bv(const uint8_t bit) { return (1 << bit); }


void UART_Initialize_9bit(void)
{
	uint16_t UBRR = Calculate_UBRR (Baud_rate);
	UBRR0H = (uint8_t)(UBRR >> 8);
	UBRR0L = (uint8_t)UBRR;
	UCSR0B |=(1 << RXEN0) | (1 << TXEN0) | (1 << UCSZ02);
	UCSR0C |= (1 << UCSZ00) | (1 << UCSZ01);
}

void UART_Transmit_9bit(const uint16_t Data_dum)
{
	while( !(UCSR0A & (1<<UDRE0)) );
	if( Data_dum & 0x0100 )
	UCSR0B |= (1<<TXB80);
	else
	UCSR0B &= ~(1<<TXB80);
	UDR0 = Data_dum;
}

uint16_t UART_Receive_9bit(void)
{
	while( !(UCSR0A & (1<<RXC0)) );
	uint8_t Higher_byte = UCSR0B;
	uint8_t Lower_byte = UDR0;
	return ( ((Higher_byte & (1<<RXB80)) << 7) | Lower_byte );
}