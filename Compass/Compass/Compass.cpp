/*
 * compass.cpp
 *
 * Created: 5/30/2017 9:21:48 AM
 *  Author: Rajesh Shrestha
 *  Editor : Prakash Chaudhary
 */ 

#include "Compass.h"
#include <math.h>
#include "TWI.h"

#define WRITE_ADDRESS 0x3C
#define READ_ADDRESS 0x3D

#define X_OFFSET -8.5
#define Y_OFFSET -85



Compass::Compass(void)
{
	i2c_init();
	raw_x=0;
	raw_y=0;
	raw_z=0;
	Angle = 0;
}

void Compass::init_compass(void){

	i2c_start(WRITE_ADDRESS );
	i2c_write(0x00);
	i2c_write(0x70);
	i2c_stop();

	i2c_start(WRITE_ADDRESS );
	i2c_write(0x01);
	i2c_write(0xA0);
	i2c_stop();

	i2c_start(WRITE_ADDRESS );
	i2c_write(0x02);
	i2c_write(0x00);
	i2c_stop();
}

void Compass::read_Compass(void){

	i2c_start(WRITE_ADDRESS );
	i2c_write(0x03);
	i2c_stop();

	i2c_rep_start(READ_ADDRESS );

	raw_x = ((uint8_t)i2c_readAck())<<8;
	raw_x |= i2c_readAck();
	raw_x-=	 X_OFFSET;
	
	
	
	raw_z = ((uint8_t)i2c_readAck())<<8;
	raw_z |= i2c_readAck();
	
	
	raw_y = ((uint8_t)i2c_readAck())<<8;
	raw_y |= i2c_readNak();
	raw_y-=Y_OFFSET;
	
	i2c_stop();
	
	Angle = (atan2((double)raw_y,(double)raw_x)* 180 / 3.14159265 + 180.0);
	
}