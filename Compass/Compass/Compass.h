/*
 * compass.h
 *
 * Created: 11/29/2016 10:22:53 AM
 *  Author: dhiraj.basnet
 *  Editor: Prakash Chaudhary
 */ 


#ifndef COMPASS_H_
#define COMPASS_H_

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <stdlib.h>


extern char buff[20];


class Compass
{
	private:
		int16_t raw_x;
		int16_t raw_y;
		int16_t raw_z;
		uint16_t Angle;
	public:
		Compass(void);
		void init_compass();
		void read_Compass();	
		inline int16_t read_rawx(void){return raw_x;};
		inline int16_t read_rawy(void){return raw_y;};
		inline int16_t Get_Angle(void){return Angle;};

};

#endif        