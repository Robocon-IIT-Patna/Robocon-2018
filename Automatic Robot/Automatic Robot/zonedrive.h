/*
 * zonedrive.h
 *
 * Created: 12/20/2017 5:38:00 PM
 *  Author: abheesh
 */ 


#ifndef ZONEDRIVE_H_
#define ZONEDRIVE_H_

#include <util/delay.h>
#include "drive.h"
#include "pid.h"
#include "encoder.h"
#include "headers.h"

#define SHUTTLECOCK_STATUSPORT		PINH
#define SHUTTLECOCK_STATUSPIN		PINH5	
#define ZONE_STATUSPORT				PINH
#define ZONE_STATUSPIN				PINH6

#define SHUTTLECOCKPIN				H,5
#define ZONEPIN						H,6	


/////////////////////////////////////////
bool ShuttleCockGiven = false;
bool GoThrowingZone1 = false;
bool GoThrowingZone2 = false;
bool GoThrowingZone3 = false;
/////////////////////////////////////////
bool backtoLZ1 = false;
bool backtoLZ2 = false;
////////////////////////////////////////
enum {
	inStart_point,
	inFirstloadingCorner,
	inTZ1,
	inTZ2,
	inTZ3,
	inLZ1,
	inLZ2,
	moving,
	notmoving
	};

struct coordinates{
	int x;
	int y;
	};

const struct coordinates Throwingzone1 = {4600,1880};
const struct coordinates Throwingzone2 = {6500,2500};
const struct coordinates Throwingzone3 = {6500,4000};

unsigned int where = inStart_point;
unsigned int robotState = notmoving;

/*
task1 - find junction at X so that it could turn to Y to head towards LZ1
task2 - find junction at Y for loading of shuttlecock
task3 - find next junction at TZ1 and throw shuttlecock from there
task4 - come to loading zone 1 from TZ1
task5 - goto loading zone 2 from loading zone 1
task6 - goto throwing zone 2 from loading zone 2
task7 - goto loading zone 2 from throwing zone 2
task8 - goto throwing zone 3 from loading zone 2

*/
bool task1 = false;
bool task2 = false;
bool task3 = false;
bool task4 = false;
bool task5 = false;
bool task6 = false;
bool task7 = false;
bool task8 = false;

void updateZoneflag();


void gorockthegamefield(void)
{
	//if(task1)	uart0_puts("1 \t");
	//else        uart0_puts("0 \t");  
	//if(task2)	uart0_puts("2 \t");
	//else        uart0_puts("0 \t");
	//if(task3)	uart0_puts("3 \t");
	//else        uart0_puts("0 \t");
	//if(task4)	uart0_puts("4 \t");
	//else        uart0_puts("0 \t");
	//if(task5)	uart0_puts("5 \t");
	//else        uart0_puts("0 \t");
	//if(task6)	uart0_puts("6 \t");
	//else        uart0_puts("0 \t");
	//if(task7)	uart0_puts("7 \n");
	//else        uart0_puts("0 \n");
	if((where == inLZ1 || where == inLZ2) && robotState == notmoving){
		updateZoneflag();
		stopDrive();
	}
	if(!task1 && where == inStart_point){		//move from start zone to corner of loading zone
		uart0_puts("1\n");
		movx(Throwingzone1.x,Front);
		robotState = moving;
		if(abs(encoderX.getdistance()) >= 4500){
			ltSensorX.initialise_interrupt();
		}
	}
	else if(task1 && !task2){	//move from corner to loading zone
		uart0_puts("2\n");
		where = inFirstloadingCorner;
		robotState = moving;
		ltSensorX.Off_interrupt();
		ltSensorY.initialise_interrupt();
		movYForwardSlow();
	}
	else if(task1 && task2 && where == inFirstloadingCorner && (robotState == moving)){	//wait for loading
		uart0_puts("3\n");
		where = inLZ1;
		robotState = notmoving;
		ltSensorY.Off_interrupt();
		stopDrive();
		encoderX.resetCount();
		encoderY.resetCount();
	}
	/////////////////////////REACHED LOADING ZONE 1////////////////////////////
	
	if(ShuttleCockGiven){
		////////////////////////////////////////////////////////////////////////////////////////////////
		if(GoThrowingZone1 && !task3 && where == inLZ1){
			uart0_puts("4\n");
			//uart0_puts("going throwing zone 1\n");	
			robotState = moving;
			movy(Throwingzone1.y,Front);
			if(abs(encoderY.getdistance()) >= 1200){
				ltSensorY.initialise_interrupt();
			}
		}
		else if(task3 && !task4 && where == inLZ1 && robotState == moving){		//if task3 completed
			/*
				Give mechanism permission to throw shuttlecock.
				after acknowledge,
				backtolz1 = true;
				robotstate = moving;
			*/
			//uart0_puts("Reached throwing zone 1 and returning to loading zone 1\n");
			uart0_puts("5\n");
			stopDrive();
			where = inTZ1;
			ltSensorY.Off_interrupt();
			encoderX.resetCount();
			encoderY.resetCount();
			robotState = notmoving;
			//tocheck///////////////////
			backtoLZ1 = true;
			GoThrowingZone1 = false;
		}
		if(backtoLZ1 && task3 && !task4){
			uart0_puts("6\n");
			//uart0_puts("returning to loading zone 1\n");
			movy(Throwingzone1.y,Back);
			robotState = moving;
			if(abs(encoderY.getdistance()) >= 1200){
				ltSensorY.initialise_interrupt();
			}
		}
		else if(task4 && robotState == moving && where == inTZ1){	//if task4 completed,reached loading zone 1
			uart0_puts("7\n");
			ltSensorY.Off_interrupt();
			//uart0_puts("reached loading zone1 \n");
			stopDrive();
			encoderX.resetCount();
			encoderY.resetCount();
			where = inLZ1;
			robotState = notmoving;
			ShuttleCockGiven = false;
			backtoLZ1 = false;
		}
		///////////////////////////////////THROWING ZONE 1 COMPLETED/////////////////////////////
		if(GoThrowingZone2){
			if(where == inLZ1 && !task5){
				uart0_puts("8\n");
				movx((Throwingzone2.x - Throwingzone1.x),Front);
				robotState = moving;
				if(abs(encoderX.getdistance()) >= 1000){
					ltSensorX.initialise_interrupt();
				}
			}
			else if(where == inLZ1 && task5){
				uart0_puts("9\n");
				encoderX.resetCount();
				encoderY.resetCount();
				where = inLZ2;
				robotState = notmoving;
				ltSensorX.Off_interrupt();
			}
		}
		/////////////////////////////REACHED LZ2 FROM LZ1//////////////////////////////////////////
		if(GoThrowingZone2 && where == inLZ2 && !task6){
			uart0_puts("10\n");
			robotState = moving;
			movy(Throwingzone2.y,Front);
			if(abs(encoderY.getdistance()) >= 200){
				ltSensorY.initialise_interrupt();
			}
		}
		else if(task6 && !task7 && where == inLZ2 && robotState == moving){
			uart0_puts("11\n");
			where = inTZ2;
			ltSensorY.Off_interrupt();
			encoderX.resetCount();
			encoderY.resetCount();
			robotState = notmoving;
			stopDrive();
			/* give command to throwing mechanism to throw.
				if ack received,
				backtolz2 = true;
				gothrowingzone2 = false;
			*/
			////////to check/////////////////
			backtoLZ2 = true;
			GoThrowingZone2 = false;	
		}
		
		if(backtoLZ2 && task6 && !task7){
			uart0_puts("12\n");
			movy(Throwingzone2.y,Back);
			robotState = moving;
			if(abs(encoderY.getdistance()) >= 1200){
				ltSensorY.initialise_interrupt();
			}
		}
		else if(task7 && robotState == moving && where == inTZ2){		//task7 completed,reached loading zone 2
			uart0_puts("13\n");
			ltSensorY.Off_interrupt();
			stopDrive();
			encoderX.resetCount();
			encoderY.resetCount();
			where = inLZ2;
			robotState = notmoving;
			ShuttleCockGiven = false;
			backtoLZ2 = false;
		}
		//////////////////////THROWING ZONE 2 COMPLETED/////////////////////////////////////////////////////////
		if(GoThrowingZone3 && where == inLZ2 && !task8){
			movy(Throwingzone3.y,Front);
			robotState = moving;
			if(abs(encoderY.getdistance()) >= 2400){
				ltSensorY.initialise_interrupt();
			}
		}
		else if(task8){
			stopDrive();
			ltSensorY.Off_interrupt();
			robotState = notmoving;
			/* send command to throwing mechanism to throw golden shuttlecock.
			*/
		}
	}
	///////////// REACHED LOADING ZONE 1///////////////////////////////
	//if(ShuttleCockGiven)
	//{
		//if(GoThrowingZone1)
		//{
			//movy(Throwingzone1.y,Front);
			//robotState = moving;
			//if(abs(encoderY.getdistance()) >= 1600){
				//ltSensorY.initialise_interrupt();
			//}
		//}
		//
		//if(backtoLZ1 && where == inTZ1)
		//{
			//movy(Throwingzone1.y,Back);
			//robotState = moving;
			//if(abs(encoderY.getdistance()) >= 1600){
				//ltSensorY.initialise_interrupt();
			//}f
		//}
		//if(GoThrowingZone2)
		//{
			//if(where == inLZ1){
				//movx((Throwingzone2.x - Throwingzone1.x),Front);
				//robotState = moving;
				//if(abs(encoderX.getdistance()) >= 1500){
					//ltSensorX.initialise_interrupt();
				//}
			//}
			//else if(where == inLZ2)
			//{
				//movy(Throwingzone2.y,Front);
				//robotState = moving;
				//if(abs(encoderY.getdistance()) >= 1500){
					//ltSensorY.initialise_interrupt();
				//}
			//}
		//}
		//if(backtoLZ2 && where == inTZ2)
		//{
			//movy(Throwingzone2.y,Back);
			//robotState = moving;
			//if(abs(encoderY.getdistance()) >= 1600){
				//ltSensorY.initialise_interrupt();
			//}
		//}
	//
		 //if(GoThrowingZone3)
		 //{
			//movy(Throwingzone3.y,Front);
			//robotState = moving;
			//if(abs(encoderY.getdistance()) >= 2300){
				//ltSensorY.initialise_interrupt();
			//}
		//}
	//}
}

void updateZoneflag(void)
{
	INPUT(SHUTTLECOCKPIN);
	INPUT(ZONEPIN);
	SET(SHUTTLECOCKPIN);
	SET(ZONEPIN);
	
	////if low on shuttlecock pin then shuttlecock received///////////////
	///i.e if obstacle on shuttlecock IR ////////////////////////////////	
	if(!(SHUTTLECOCK_STATUSPORT & (1<<SHUTTLECOCK_STATUSPIN)) && (where == inLZ2 || where == inLZ1) ){
		ShuttleCockGiven = true;
		}	//shuttlecock given
	else{
		ShuttleCockGiven = false;
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////if low on zone pin then go to the next zone//////////////////////
	if((ZONE_STATUSPORT & (1<<ZONE_STATUSPIN)) && (where == inLZ1) ){			//if low on zone signal go next throwingzone2	
		GoThrowingZone1 = true;
		GoThrowingZone2 = false;
		/*  if goto throwing zone = true then
			task3 = notdone,
			task4 = notdone 
		*/
		task3 = false;
		task4 = false;
	}
	else if((!(ZONE_STATUSPORT & (1<<ZONE_STATUSPIN))) && (where == inLZ1)){	//if high on zone signal repeat throwingzone1
		//UART0TransmitString("\ngothrowingzone 2\n");
		GoThrowingZone1 = false;
		GoThrowingZone2 = true;
		task3 = true;
		task4 = true;
		
	}
	else if((ZONE_STATUSPORT & (1<<ZONE_STATUSPIN)) && (where == inLZ2)){		//if low on zone signal next go throwing zone 3
		GoThrowingZone2 = true;
		GoThrowingZone3 = false;
		/*	if goto throwing zone 2 = true then
			task6 = notdone,
			task7 = notdone
		*/
		task6 = false;
		task7 = false;
	}
	else if((!(ZONE_STATUSPORT & (1<<ZONE_STATUSPIN))) && (where == inLZ2)){
		GoThrowingZone2 = false;
		GoThrowingZone3 = true;
		
		task6 = true;
		task7 = true;
	}
}



ISR(PCINT0_vect)	//junction X
{
	if(!task1)
		task1 = true;		//reached to corner of loading zone
	else if(!task5)
		task5 = true;		//reached loading zone 2 from loading zone 1
}
ISR(PCINT2_vect)
{
	if(!task2)
		task2 = true;		//reached to loading zone 1
	else if(!task3)
		task3 = true;
	else if(!task4)
		task4 = true;
	else if(!task6)
		task6 = true;
	else if(!task7)
		task7 = true;
	else if(!task8)
		task8 = true;
}

#endif /* ZONEDRIVE_H_ */