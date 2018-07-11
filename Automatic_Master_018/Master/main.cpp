/*
 * Master.cpp
 *
 * Created: 6/8/2018 5:24:41 PM
 * Author : Prakash Chaudhary
 */ 
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "TWI.h"
#include "uart.h"
#include "encoder.h"
#include "drive.h"
#include "zonedrive.h"
#include "headers.h"
#include "gy88.h"


////////////////objects of classes///////////
encoder encoderX,encoderY;
////////////////////////////////////////////
float roll,pitch,yaw;
bool junctionY;
bool junctionX;
bool FirstData = true;
bool calibrate = true;

uint16_t compass_Angle = 0;
uint16_t angle_Max = 0;
uint16_t angle_Min = 0;
uint16_t angle_Average = 0;

void checkJunctionOfY(){
	
	if(bit_is_set(PINK,PK0)){
		junctionY = true;
	}
	else{
		junctionY = false;
	}
	
}
void checkJunctionOfX(){
	if(bit_is_set(PINB,PB4)){
		junctionX = true;
	}
	else{
		junctionX = false;
	}
}
void changeCompassSetpoint(){
	if(FlagChangeSetpointCompass)
	{
		if(junctionX && lineTrackerData == 45 && !junctionY){
			if(FirstData){
				angle_Min = compass_Angle;
				angle_Max = compass_Angle;
				FirstData = false;
			}
			if(compass_Angle > angle_Max)
				angle_Max = compass_Angle;
			else if(angle_Min > compass_Angle)
				angle_Min = compass_Angle;
			
			angle_Average = (angle_Max + angle_Min)/2;
			
			compass.SETPOINT = angle_Average;
			
		}
 	}
	else if(FlagInitialAngleSetpoint){
		 compass.SETPOINT = initialCompassAngle;
	}
	// 		 uart0_putint(angle_Max);
	// 		 uart0_puts("\t");
	// 		 uart0_putint(angle_Min);
	// 		 uart0_puts("\t");
	// 		 uart0_putint(angle_Average);
	// 		 uart0_puts("\t");
	// 		 uart0_putint(compass_Angle);
	// 		 uart0_puts("\r\n");
}
void reactConditionOfLineLeft(){
	//////////Find the condition when line is left////////
	
	if(movingyback || movingyfront){
		if(lineTrackerData == 0 && previousLinetrackerData == 10 && !junctionY)
		{
			ltY.leftedgeleft = true;
			ltY.rightedgeleft = false;
			lineMeet = false;
		}
		else if(lineTrackerData == 0 && previousLinetrackerData == 80 && !junctionY)
		{
			ltY.rightedgeleft = true;
			ltY.leftedgeleft = false;
			lineMeet = false;
		}
		/////////////////////////////////////////////////////
		
		////If line left by linetracker find when line is meet
		if(ltY.rightedgeleft && lineTrackerData == 80){
			lineMeet = true;
			ltY.rightedgeleft = false;
			ltY.leftedgeleft = false;
		}
		else if(ltY.leftedgeleft && lineTrackerData == 10){
			lineMeet = true;
			ltY.leftedgeleft = false;
			ltY.rightedgeleft = false;
		}
		///////////////////////////////////////////////////////
		
		////////// if edge is left and junction is detected //////
		if((ltY.leftedgeleft || ltY.rightedgeleft) && junctionY){
			if(movingyfront){
				velocity_robot[0] = 0;		//x
				velocity_robot[1] = 40;		//y
				calculateCompassPID();
			}
			else if(movingyback){
				velocity_robot[0] = 0;
				velocity_robot[1] = -40;
				calculateCompassPID();
			}
		}
		/////////////////////////////////////////////////////////
		else if(ltY.leftedgeleft){
			if(movingyfront){
				velocity_robot[0] = -40;
				velocity_robot[1] = 10;
				calculateCompassPID();
			}
			else if(movingyback){
				velocity_robot[0] = -40;
				velocity_robot[1] = -10;
				calculateCompassPID();
			}
		}
		else if(ltY.rightedgeleft ){
			if(movingyfront){
				velocity_robot[0] = 40;
				velocity_robot[1] = 10;
				calculateCompassPID();
			}
			else if(movingyback){
				velocity_robot[0] = 40;
				velocity_robot[1] = -10;
				calculateCompassPID();
			}
		}
	}
}

int main(void)
{
	////////////SET COMMUNICATION PINS AS INPUT AND PULL UP////////
	INPUT(SHUTTLECOCKPIN);
	INPUT(ZONEPIN);
 	SET(SHUTTLECOCKPIN);
 	SET(ZONEPIN);
	///////PULL DOWN RACK PIN 
	INPUT(RACKPIN);
	CLEAR(RACKPIN);
	///SET PK1 AS OUTPUT TO SEND SIGNAL TO SLAVE TO BRAKE MOTOR 
	DDRK |= (1<<PK0);
	PORTK &= ~(1<<PK0); 
	/// INITIALIZE ALL THE UART
	uart0_init(UART_BAUD_SELECT(9600,F_CPU));
	uart2_init(UART_BAUD_SELECT(38400,F_CPU));
	uart3_init(UART_BAUD_SELECT(38400,F_CPU));
	//INITIALIZE EVERYTHING ELSE
 	
	uart0_puts("starting\r\n");
	initializeAll();
	char rcvdata = 'q';
	sei();
	
	task1 = task2 = true;
	where = inLZ1;
	_b_Transmit_once = true;
    while (1) 
    {
// 		rcvdata = uart0_getc();
// 		
// 		if (rcvdata == ' ')
// 		{
// 			encoderX.resetCount();
// 			encoderY.resetCount();
// 		}
// 		
// 		uart0_putint(encoderX.getdistance());
// 		uart0_putc('\t');
// 		uart0_putint(encoderY.getdistance());
// 		uart0_puts("\r\n");
		
		//For calibration of compass//////////
//    		  			if(calibrate){
//    		 				 velocity_robot[0] = 0;
//    		 				 velocity_robot[1] = 0;
//    		 				 velocity_robot[2] = 60;
//    		 				 calculatevel();
//    						 
//    		 				 calibrate_compass();
//    		 				 calibrate = false;
//    		 			 }
//    		 			 else{
//    		 				 velocity_robot[0] = 0;
//    		 				 velocity_robot[1] = 0;
//    		 				 velocity_robot[2] = 0;
//    		 				 calculatevel();
//    		 				 uart0_putint(X_offset);
//    		 				 uart0_puts("\t");
//    		 				 uart0_putint(Y_offset);
//    		 				 uart0_puts("\n");
//    		 			 }	 
		

///////////copy this code and run.//////////////
//get linetracker data
lineTrackerData = getLineTrackerYdata();
//get compass data
//compass_Angle = get_Angle();
//Check the junction of Y linetracker
checkJunctionOfY();
//call the gameplay function
//changeCompassSetpoint();
gorockthegamefield();
//check for line left condition and react to it
reactConditionOfLineLeft();
//calculate velocity of each motor and send to slave
calculatevel();
//set previous line tracker data
previousLinetrackerData = linetracker_data;
//////not below this line///////////////////////
// 		
	}
}


///////////////copy this code and run.//////////////
////get linetracker data
//lineTrackerData = getLineTrackerYdata();
////get compass data
//compass_Angle = get_Angle();
////Check the junction of Y linetracker
//checkJunctionOfY();
////call the gameplay function
//changeCompassSetpoint();
//gorockthegamefield();
////check for line left condition and react to it
//reactConditionOfLineLeft();
////calculate velocity of each motor and send to slave
//calculatevel();
////set previous line tracker data
//previousLinetrackerData = linetracker_data;
////////not below this line///////////////////////


////////For calibration of compass//////////
//  			if(calibrate){
// 				 velocity_robot[0] = 0;
// 				 velocity_robot[1] = 0;
// 				 velocity_robot[2] = 60;
// 				 calculatevel();
// 				 calibrate_compass();
// 				 calibrate = false;
// 			 }
// 			 else{
// 				 velocity_robot[0] = 0;
// 				 velocity_robot[1] = 0;
// 				 velocity_robot[2] = 0;
// 				 calculatevel();
// 				 uart0_putint(X_offset);
// 				 uart0_puts("\t");
// 				 uart0_putint(Y_offset);
// 				 uart0_puts("\n");
// 			 }


