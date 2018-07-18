/*
 * drive.h
 *
 * Created: 10/23/2017 3:16:43 PM
 *  Author: abheesh
 */ 


#ifndef DRIVE_H_
#define DRIVE_H_

#define PI		3.14159265
#define START_BYTE	127

#define STARTZONEtoCORNER	220
#define CORNERtoLZ1			40
#define LZ1toTZ1			150
#define	TZ1toLZ1			150
#define LZ1toLZ2			150
#define LZ2toTZ2			150
#define	TZ2toLZ2			150
#define LZ2toTZ3			240

#include "headers.h"
#include "uart.h"
#include "encoder.h"
#include "Flags.h"
#include "gy88.h"
#include <util/delay.h>
#include <math.h>

extern encoder encoderY, encoderX;


int currentDistance = 0, difference = 0;
bool fullspeed = false;
unsigned long startTime;

/*****************************Limit switch pins*************************/
#define RIGHT_LIMIT_SW F,1
#define LEFT_LIMIT_SW F,2
unsigned long time_of_limit_switches_pressed = 0;
bool first_data_time_of_limit_switches_pressed = true;
/********************************************************************/

////////////////////////////////////////////////////////////////////////////////
extern bool PidUpdateFlagCompass;
extern bool PidUpdateFlagDriveX;
extern bool PidUpdateFlagLinetracker;
extern bool PidUpdateFlagLinetrackerFront;
extern bool PidUpdateFlagLinetrackerBack;
extern bool PidUpdateFlagDriveY;
//////////////////// For Motor variables //////////////////////
signed char bufferMotorSpeed[4] = {0,0,0,0};
int velocity_motor[4];
int velocity_robot[3];
float coupling_matrix[4][3] = {{-1,1,1},{-1,-1,1},{1,-1,1},{1,1,1}};  //0.707
double value = PI / 180;
/////////////////////////// Other variables //////////////////////////////////

int distanceX;
int distanceY;
uint16_t initialCompassAngle;
////For initial conditions/////////
bool giveComponent = true;
bool movingFromStartPoint = true;

bool lineMeet = true;
bool movingxfront = false;
bool movingxback = false;
bool movingyfront = false;
bool movingyback = false;
bool inverseKinematicTrue = true;

//////////////////////// For Linetracker//////////////////////////////////
static uint8_t linestate = 0;
static int linetracker_data = 0;
static double totalSum = 0;
static double totalLine = 0;
static int lineBit[8];
static int weight[8] = {10,20,30,40,50,60,70,80};

unsigned int lineTrackerData = 0;
unsigned int previousLinetrackerData = 0;

////////////////////////////////////////////////////////////////////////////////
enum direction{
	X_Axis,
	Y_Axis,
	Front,
	Back
};
////////////////////////////////////////////////////////////////////////////////


struct bodyPid{ 
	bodyPid():FirstData(true){};
	int input,error,prevInput,output;
	bool leftedgeleft = false;
	bool rightedgeleft = false;
	double Iterm;
	unsigned int SETPOINT;	
	bool FirstData = true;
	int Max_output;
	int Min_output;
	double kp, ki, kd;
	
	void setPid(float p, float i, float d){
		kp = p;
		ki = i;
		kd = d;
	}
	void Set_Max_Min_Output (int Max_dum, int Min_dum)
	{
		Max_output = Max_dum;
		Min_output = Min_dum;
	}
	inline void incrkp(){kp += 0.1;}
	inline void dcrkp(){kp -= 0.1;}
	inline void incrki(){ki += 0.01;}
	inline void dcrki(){ki -= 0.01;}
	inline void incrkd(){kd += 0.5;}
	inline void dcrkd(){kd -= 0.5;}
	inline void incrSetpoint(){SETPOINT += 2;}
	inline void dcrSetpoint(){SETPOINT -= 2;}
	inline int getSETPOINT(){return SETPOINT;}
	inline float getkp(void){return kp;}
	inline float getki(void){return ki;}
	inline float getkd(void){return kd;}
	
};
bodyPid ltX,ltY,compass,driveX,driveY;


/////////////////////////////////////////////////////////
uint16_t stable_data_count = 0;
unsigned long millis_time_then = 0;
void Compensate_DistanceY_When_Moving_Towards_Fence(void);
void holdposition(void);
bool Stable_Robot(void);
void calculateCompassPID(void);
void calculatevel();
int filterLineTrackerData(int);
void calculateLineTrackerYPid();
void movx(int distance_setpoint, int direction,int speed);
bool Goto_Fence_And_Detect(void);
void initializeAll(void);
void sendDataToSlave(void);
void BrakeMotor(void);
int getLineTrackerYBackData();
int getLineTrackerYFrontData();

inline void linetrackerXjunctionWatch();
inline void lintrackerYjunctionWatch();

inline void linetrackerXjunctionWatchOff();
inline void linetrackerYjunctionWatchOff();
////////////////////////////////////////////////////
//16th july
int8_t _direction_matrix[5][4] = {{-1, -1, 1, 1}, {1, 1, -1, -1}, {1, -1, -1, 1}, {-1, 1, 1, -1}, {0, 0, 0, 0}};
uint8_t _axis = 7, _direction = 7;
bodyPid FrontLinetrackerY_, BackLinetrackerY_;

 void Calculate_Front_LinetrackerY_Pid(void);
 void Calculate_Back_LinetrackerY_Pid(void);
 void Calculate_Velocity(void);
 void Calculate_Motor_Differential_Velocity_With_Center_Pivot(int d_speed);
 void Calculate_Motor_Differential_Velocity_With_Wheel_Pivot(int d_speed);
 void MovX(int distance_setpoint, int direction, unsigned int speed);
 void MovY(int distance_setpoint, int direction, unsigned int speed);
// 
// //16th july

void Calculate_Front_LinetrackerY_Pid(void)
{
	if(FrontLinetrackerY_.FirstData && getLineTrackerYFrontData() != 0){
		FrontLinetrackerY_.prevInput = getLineTrackerYFrontData();
		FrontLinetrackerY_.FirstData = false;
	}
	else if(PidUpdateFlagLinetrackerBack && linetrackerPID){
		FrontLinetrackerY_.input = getLineTrackerYFrontData();
		//if linetracker input is not zero///
		if(FrontLinetrackerY_.input != 0 && lineMeet){
			//uart0_puts("calculating\n");
			FrontLinetrackerY_.error = FrontLinetrackerY_.SETPOINT - FrontLinetrackerY_.input;
			if((FrontLinetrackerY_.error) == 0)
			{
				FrontLinetrackerY_.Iterm = 0;
			}
			if(FrontLinetrackerY_.error == 0)
			FrontLinetrackerY_.prevInput = FrontLinetrackerY_.input;
			FrontLinetrackerY_.Iterm += FrontLinetrackerY_.ki * FrontLinetrackerY_.error;
			if(abs(FrontLinetrackerY_.Iterm) > 5){
				if(FrontLinetrackerY_.Iterm > 0)	FrontLinetrackerY_.Iterm = 5;
				else if(FrontLinetrackerY_.Iterm < 0)	FrontLinetrackerY_.Iterm = -5;
			}
			FrontLinetrackerY_.output = FrontLinetrackerY_.kp * FrontLinetrackerY_.error + FrontLinetrackerY_.Iterm - FrontLinetrackerY_.kd *(FrontLinetrackerY_.input - FrontLinetrackerY_.prevInput);
			FrontLinetrackerY_.prevInput = FrontLinetrackerY_.input;
			if (abs(FrontLinetrackerY_.output) > 80)
			{
				if (FrontLinetrackerY_.output > 0){FrontLinetrackerY_.output = 80;}
				else{FrontLinetrackerY_.output = -80;}
			}

		}

		PidUpdateFlagLinetrackerFront = false;

	}
	if(!linetrackerPID)
	FrontLinetrackerY_.output = 0;

}

void Calculate_Back_LinetrackerY_Pid(void)
{
	if(BackLinetrackerY_.FirstData && getLineTrackerYBackData() != 0){
		BackLinetrackerY_.prevInput = getLineTrackerYBackData();
		BackLinetrackerY_.FirstData = false;
	}

	if(PidUpdateFlagLinetrackerBack && linetrackerPID){
		BackLinetrackerY_.input = getLineTrackerYBackData();
		//if linetracker input is not zero///
		if(BackLinetrackerY_.input != 0 && lineMeet){
			BackLinetrackerY_.error = BackLinetrackerY_.SETPOINT - BackLinetrackerY_.input;
			if((BackLinetrackerY_.error) == 0)
			{
				BackLinetrackerY_.Iterm = 0;
			}
// 			if(BackLinetrackerY_.error == 0)
// 				BackLinetrackerY_.prevInput = BackLinetrackerY_.input;
			BackLinetrackerY_.Iterm += BackLinetrackerY_.ki * BackLinetrackerY_.error;
			if(abs(BackLinetrackerY_.Iterm) > 5){
				if(BackLinetrackerY_.Iterm > 0)	BackLinetrackerY_.Iterm = 5;
				else if(BackLinetrackerY_.Iterm < 0)	BackLinetrackerY_.Iterm = -5;
			}
			BackLinetrackerY_.output = BackLinetrackerY_.kp * BackLinetrackerY_.error + BackLinetrackerY_.Iterm - BackLinetrackerY_.kd *(BackLinetrackerY_.input - BackLinetrackerY_.prevInput);
			BackLinetrackerY_.prevInput = BackLinetrackerY_.input;
			if (abs(BackLinetrackerY_.output) > 80)
			{
				if (BackLinetrackerY_.output > 0){BackLinetrackerY_.output = 80;}
				else{BackLinetrackerY_.output = -80;}
			}
		}

		PidUpdateFlagLinetrackerBack
		 = false;

	}

	if(!linetrackerPID)
	BackLinetrackerY_.output = 0;

}

 void MovX(int distance_setpoint, int direction, unsigned int speed){
// 	//compass.setPid(2.1,0.04,32);
 	distanceX = abs(encoderX.getdistance());
	driveX.SETPOINT = distance_setpoint;
 	if(PidUpdateFlagDriveX)
 	{
  		movingyfront = false;
  		movingyback = false;
  		driveX.input = distanceX;
  		PidUpdateFlagDriveX = false;
 		if(distanceX >= 200){
  			driveX.error = driveX.SETPOINT - driveX.input;
  			driveX.Iterm += driveX.ki * driveX.error;
  			if(driveX.FirstData){
  				driveX.prevInput = driveX.input;
  				driveX.FirstData = false;
  			}
  			if(abs(driveX.Iterm) > 10){
  				if(driveX.Iterm > 0)	driveX.Iterm = 10;
  				if(driveX.Iterm < 0)	driveX.Iterm = -10;
  			}
  			if((driveX.error) > 0){
  				driveX.output = driveX.kp * driveX.error + driveX.Iterm - driveX.kd*(driveX.input - driveX.prevInput);
  			}
  			else{
  				driveX.output = 0;
  			}
  			driveX.prevInput = driveX.input;
  			//////////////////////////////////////////////////////
  			if(abs(driveX.output) > speed){
  				if(driveX.output >0)	driveX.output = speed;
  				else						driveX.output = -speed;
  			}
		 
  			if(abs(driveX.output) < 20){
  				if(movingxfront)		driveX.output = 20;
  				else if(movingxback)	driveX.output = -20;
  			}
		 }
  			////////////////////////////////////////////////////
  		else{
  			driveX.output = 60 + 0.45*distanceX;
  			}
	}
 // 
  		_axis = X_Axis;
  
		if(direction == Front){
			movingxfront = true;
			movingxback = false;
			movingyback = false;
			movingyfront = false;
			_direction = Front;
		}
		else if(direction == Back){
			movingxfront = false;
			movingxback = true;
			movingyfront = false;
			movingyback = false;
			_direction = Back;
		}

//	}

 	Calculate_Motor_Differential_Velocity_With_Center_Pivot(driveX.output);
}
// 
 void MovY(int distance_setpoint, int direction ,unsigned int speed)
 {
 	//compass.setPid(2.1,0.04,32);
 	distanceY = abs(encoderY.getdistance());
 	driveY.SETPOINT = distance_setpoint;
 	if(PidUpdateFlagDriveY)
 	{
 		PidUpdateFlagDriveY = false;
 		if(distanceY >= 600)
 		{
 			driveY.input = distanceY;
 			driveY.error = driveY.SETPOINT - driveY.input;
 			driveY.Iterm += driveY.ki * driveY.error;
 			if(driveY.FirstData){
 				driveY.prevInput = driveY.input;
 				driveY.FirstData = false;
 			}
 			if(abs(driveY.Iterm) > 10){
 				if(driveY.Iterm > 0)	driveY.Iterm = 10;
 				if(driveY.Iterm < 0)	driveY.Iterm = -10;
 			}
 			if(driveY.error > 0){
 				driveY.output = driveY.kp * driveY.error;// + driveY.Iterm - driveY.kd*(driveY.input - driveY.prevInput);
 			}
 			else{
 				driveY.output = 0;
 			}
 			driveY.prevInput = driveY.input;
 			////////////////////////////////////////////////////////////
 			/////////////////////////////////////////////////////////
 		}
 		else
 		{
 			driveY.output = 60 + (distanceY*0.15);
 		}
		 
		 
		if(abs(driveY.output) >= speed){
			driveY.output = speed;
			//if(movingyfront)		driveY.output = speed;
			//else if(movingyback)	driveY.output = -speed;
		}
		//if(abs(driveY.output) < 20){
			//driveY.output = 20;
			//if(movingyfront)		driveY.output = 20;
			//else if(movingyback)	driveY.output = -20;
		//}
 		_axis = Y_Axis;
 		if(direction == Front){
 			movingyfront = true;
 			movingyback = false;
 			movingxfront = false;
 			movingxback = false;
 			_direction = Front;
 		}
 		else if(direction == Back){
 			movingyback = true;
 			movingyfront = false;
 			movingxfront = false;
 			movingxback  = false;
 			_direction = Back;
 		}
 
 	}
	 Calculate_Motor_Differential_Velocity_With_Center_Pivot(driveY.output);
 	//Calculate_Motor_Differential_Velocity_With_Wheel_Pivot(driveY.output);
 }
 
 
  void Calculate_Motor_Differential_Velocity_With_Center_Pivot(int d_speed)
  {
  	inverseKinematicTrue = false;
 	Calculate_Front_LinetrackerY_Pid();
 	Calculate_Back_LinetrackerY_Pid();
 	
	 if (_direction == Front)
	 {
		 velocity_motor[0] = d_speed + FrontLinetrackerY_.output;
		 velocity_motor[1] = d_speed - FrontLinetrackerY_.output;
		 velocity_motor[2] = d_speed + BackLinetrackerY_.output;
		 velocity_motor[3] = d_speed - BackLinetrackerY_.output;
	 }
	 else
	 {
		 velocity_motor[0] = d_speed - FrontLinetrackerY_.output;
		 velocity_motor[1] = d_speed + FrontLinetrackerY_.output;
		 velocity_motor[2] = d_speed - BackLinetrackerY_.output;
		 velocity_motor[3] = d_speed + BackLinetrackerY_.output;
	 }
  	
 	Calculate_Velocity();
  }
 
  void Calculate_Motor_Differential_Velocity_With_Wheel_Pivot(int d_speed)
  {
	  inverseKinematicTrue = false;
	  Calculate_Front_LinetrackerY_Pid();
	  Calculate_Back_LinetrackerY_Pid();
	  
	  if (_direction == Front)
	  {
		  if (FrontLinetrackerY_.output > 0)
		  {
			  velocity_motor[0] = d_speed + FrontLinetrackerY_.output;
			  velocity_motor[1] = d_speed;
			  velocity_motor[2] = d_speed  + BackLinetrackerY_.output;
			  velocity_motor[3] = d_speed;
		  }
		  else
		  {
			  velocity_motor[0] = d_speed;
			  velocity_motor[1] = d_speed - FrontLinetrackerY_.output;
			  velocity_motor[2] = d_speed;
			  velocity_motor[3] = d_speed - BackLinetrackerY_.output;
		  }
	  }
	  else if (_direction == Back)
	  {
		  if (FrontLinetrackerY_.output > 0)
		  {
			  velocity_motor[0] = d_speed;
			  velocity_motor[1] = d_speed + FrontLinetrackerY_.output;
			  velocity_motor[2] = d_speed;
			  velocity_motor[3] = d_speed + BackLinetrackerY_.output;
		  }
		  else
		  {
			  velocity_motor[0] = d_speed - FrontLinetrackerY_.output;
			  velocity_motor[1] = d_speed;
			  velocity_motor[2] = d_speed - BackLinetrackerY_.output;
			  velocity_motor[3] = d_speed;
		  }
	  }
	  
	Calculate_Velocity();
	  
  }
// 
 void Calculate_Velocity(void)
 {
 	uint8_t ii = 4;
 
 	if (_axis == X_Axis)
 	{
 		if (_direction == Front){ ii = 0; }
 		else if (_direction == Back) { ii = 1; }
 	}
 	else if (_axis == Y_Axis)
 	{
 		if (_direction == Front){ ii = 2; }
 		else if (_direction == Back) { ii = 3; }
 	}
 
 	for (uint8_t jj = 0; jj < 4; jj++)
 	{
 		velocity_motor[jj] = velocity_motor[jj] * _direction_matrix[ii][jj];
 	}
 }
// 
// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 int getLineTrackerYBackData(void){
 	for(int i = 0; i <= 7; i++){
 		if(bit_is_set(PINC,i)){
 			lineBit[i] = 1;
 			linestate |= (1<<i);
 		}
 		else{
 			lineBit[i] = 0;
 		}
 		totalSum += weight[i]*lineBit[i];
 		totalLine += lineBit[i];
 	}
 	linetracker_data = totalSum/totalLine;
 	totalSum = 0;
 	totalLine = 0;
 	return linetracker_data;
 }
 
 int getLineTrackerYFrontData(void){

 	int data = /*filterLineTrackerData*/(uart2_getc());
 	
	if(data == 255){
 		data = 0;
 		return(data);
 	}
 	else
 	return (data + 10);
 }

void BrakeMotor(void){
	PORTK ^= (1<<PK0);
	movingxback = false;
	movingxfront = false;
	movingyfront = false;
	movingyback = false;
}

void sendDataToSlave(void){

uart2_putc(START_BYTE);
//_delay_ms(1);
uart2_putc(bufferMotorSpeed[0]);
//_delay_ms(1);
uart2_putc(bufferMotorSpeed[1]);
//_delay_ms(1);
uart2_putc(bufferMotorSpeed[2]);
//_delay_ms(1);
uart2_putc(bufferMotorSpeed[3]);
/*_delay_ms(1);*/
}



int getLineTrackerYdata(void){
	for(int i = 0; i <= 7; i++){
		if(bit_is_set(PINC,i)){
			lineBit[i] = 1;
			linestate |= (1<<i);
		}
		else{
			lineBit[i] = 0;
		}
		totalSum += weight[i]*lineBit[i];
		totalLine += lineBit[i];
	}
	linetracker_data = totalSum/totalLine;
	totalSum = 0;
	totalLine = 0;
	return linetracker_data;
}

inline void linetrackerXjunctionWatch(void){
	sei();
	PCICR |= (1<<PCIE0);
	PCMSK0 |= (1<<PCINT4);
}
inline void linetrackerYjunctionWatch(void){
	sei();
	PCICR |= (1<<PCIE2);
	PCMSK2 |= (1<<PCINT23);
}
inline void linetrackerXjunctionWatchOff(void){
	PCMSK0 &= ~(1<<PCINT4);
}
inline void linetrackerYjunctionWatchOff(void){
	PCMSK2 &= ~(1<<PCINT23);
}

void Compensate_DistanceY_When_Moving_Towards_Fence(void)
{
	velocity_robot[1] = 0.8* encoderY.getdistance();
}
bool Stable_Robot(void)
{
	holdposition();
	uint16_t _get_angle = 0;
	//uart0_puts
	//uart0_putint(millis());
	//uart0_puts("\t");
	//uart0_putint(millis_time_then);
	if ((millis() - millis_time_then) > 1)
	{
		_get_angle = getYawGY88();
		//uart3_putint(_get_angle);
		//uart3_puts("\r\n");
		if(_get_angle<=(compass.SETPOINT+1) && _get_angle>=(compass.SETPOINT-1))
		{
			stable_data_count++;
			//uart3_putint(stable_data_count);
			//uart3_putc('\t');
		}
		else{
			stable_data_count = 0;
		}
		if (stable_data_count == 100)
		{
			//uart3_puts("Stable data \r\n");
			stable_data_count = 0;
			return 1;
		}
		//else return 0;
		millis_time_then = millis();
	}
	
	return 0;
}

bool Goto_Fence_And_Detect(void)
{
	movingyfront = false;
	
	if (READ(RIGHT_LIMIT_SW) && !READ(LEFT_LIMIT_SW))
	{
		inverseKinematicTrue = false;
		//uart3_puts("LEFT\r\n");
		velocity_motor[0] = 30;
		velocity_motor[1] = 0;
		velocity_motor[2] = 0;
		velocity_motor[3] = -20;
// 		velocity_robot[0] = 0;
// 		velocity_robot[1] = 20;
// 		velocity_robot[2] = 20;
		time_of_limit_switches_pressed = 0;
		first_data_time_of_limit_switches_pressed = true;
	}
	else if (READ(LEFT_LIMIT_SW) && !READ(RIGHT_LIMIT_SW))
	{
		inverseKinematicTrue = false;
		//uart3_puts("RIGHT\r\n");
		velocity_motor[0] = 20;
		velocity_motor[1] = 0;
		velocity_motor[2] = 0;
		velocity_motor[3] = -30;
		time_of_limit_switches_pressed = 0;
		first_data_time_of_limit_switches_pressed = true;
	}
	else if (READ(LEFT_LIMIT_SW) && READ(RIGHT_LIMIT_SW))
	{
		inverseKinematicTrue = true;
		//velocity_robot[0] = -20;
		//movx(500,Back,30);
		//Compensate_DistanceY_When_Moving_Towards_Fence();
 		velocity_robot[0] = -40;
 		velocity_robot[1] = 5;
 		velocity_robot[2] = 0;
		time_of_limit_switches_pressed = 0;
		first_data_time_of_limit_switches_pressed = true;	
	}
	
	if (!READ(LEFT_LIMIT_SW) && !READ(RIGHT_LIMIT_SW))
	{
		inverseKinematicTrue = false;
		//uart3_puts("STRAIGHT\r\n");
		velocity_motor[0] = 10;
		velocity_motor[1] = 0;
		velocity_motor[2] = 0;
		velocity_motor[3] = -10;
		//uart3_puts("BOTH PRESSED\r\n");
		//BrakeMotor();
		if (first_data_time_of_limit_switches_pressed)
		{
			//uart3_puts("FIRST TIME LIMIT SWITCH PRESSED\r\n");
			time_of_limit_switches_pressed = millis();
			first_data_time_of_limit_switches_pressed = false;
		}
		if (millis() - time_of_limit_switches_pressed > 100)
		{
			return 1;
			//uart3_puts("\r\n\n\n JOB DONE \r\n\n\n");
			//uart3_putc(d_char_to_transmit);
		}
	}
	return 0;
}
void calculateCompassPID(void)
{
	if(PidUpdateFlagCompass && compassPID)
	{
		
		compass.input = getYawGY88();
		
		
		compass.error = compass.SETPOINT	-	compass.input;

		if (compass.error > 180)
		{
			compass.error = compass.error - 360;
		}
		else if (compass.error < -180)
		{
			compass.error = compass.error + 360;
		}
	
		compass.Iterm += compass.ki*compass.error;

		if (abs(compass.Iterm) > 0.2*compass.Max_output)
		{
			if(compass.Iterm > 0)
				compass.Iterm = 0.2*compass.Max_output;
			else
				compass.Iterm = -0.2*compass.Max_output;
		}
		
		if (abs(compass.error) > 1)
		{
			compass.output = compass.kp*compass.error	-	compass.kd*(compass.input-compass.prevInput)	+	compass.Iterm;
		}
		
		else
		{
			compass.Iterm = 0;
			compass.output = 0;
		}
			
		compass.prevInput = compass.input;
		
		if (abs(compass.output) > compass.Max_output)
		{
			compass.output = (compass.output > compass.Max_output) ?	compass.Max_output : -compass.Max_output;
		}

		velocity_robot[2] = compass.output;
		
		PidUpdateFlagCompass = false;
	}
	
	if(!compassPID){
		velocity_robot[2] = 0;
	}
}


void calculatevel()	//use matrix to find setpoint of individual motor and store in bufferMotorSpeed and send to slave
{
	if(inverseKinematicTrue){
		for(int i=0;i<4;i++)
		{
			velocity_motor[i] = 0;
			for(int j=0;j<3;j++)
			{
				velocity_motor[i] += velocity_robot[j] * coupling_matrix[i][j];
				
			}
		}
	}
	
	bufferMotorSpeed[0] = ((velocity_motor[0]) * 23)/249;
	bufferMotorSpeed[1] = ((velocity_motor[1]) * 23)/249;

bufferMotorSpeed[2] = ((velocity_motor[2]) * 23)/249;
bufferMotorSpeed[3] = ((velocity_motor[3]) * 23)/249 ;

	sendDataToSlave();
}		

int filterLineTrackerData(int newData)
{
	int data;
	
	if (previousLinetrackerData == 70 && newData >= 255)
	data = 255;
	else if (previousLinetrackerData == 0 && newData >= 255)
	data = 255;
	else if (previousLinetrackerData >= 0 && previousLinetrackerData <= 70 && abs(newData) > 70)
	data = previousLinetrackerData;
	else
	{
		data = newData;
		previousLinetrackerData = data;
	}
	
	return data;
}


void calculateLineTrackerYPid()
{
 	if(ltY.FirstData && getLineTrackerYdata() != 0){
 		ltY.prevInput = getLineTrackerYdata();
 		ltY.FirstData = false;
 	}
	else if(PidUpdateFlagLinetracker && linetrackerPID){
		ltY.input = getLineTrackerYdata();
		//if linetracker input is not zero///
		if(ltY.input != 0 && lineMeet){
			//uart0_puts("calculating\n");
			ltY.error = ltY.SETPOINT - ltY.input;
			if((ltY.error) == 0)
			{
				ltY.Iterm = 0;
			}
			if(ltY.error == 0)
				ltY.prevInput = ltY.input;
			ltY.Iterm += ltY.ki * ltY.error;
			if(abs(ltY.Iterm) > 10){
				if(ltY.Iterm > 0)	ltY.Iterm = 5;
				else if(ltY.Iterm < 0)	ltY.Iterm = -5;
			}
			ltY.output = ltY.kp * ltY.error + ltY.Iterm - ltY.kd *(ltY.input - ltY.prevInput);
			ltY.prevInput = ltY.input;
			if (abs(ltY.output) > 40)
			{
				if (ltY.output > 0){ltY.output = 40;}
				else{ltY.output = -40;}
			}
			velocity_robot[0] = -ltY.output;
		}

		PidUpdateFlagLinetracker = false;
		
	}
	if(!linetrackerPID)
		velocity_robot[0] = 0;
	
}



void initializeAll(void)
{
	// 2.1, 0, 12.5 ---> P I D values of compass after compass and linetrackr tune
	// 0.8, 0, 20   ---> P I D values of lintracker after compass and linetracker tune
	
	//compass.Set_Max_Min_Output(40,0);	
	
	//ltY.SETPOINT = 45;
	FrontLinetrackerY_.SETPOINT =45;
	BackLinetrackerY_.SETPOINT = 45;
	//compass.setPid(8,0,52);//3.7,0.3,12);//2,0,31);//4,0.09,18);	//5.5, 0, 500 , 2.1,0.04,32
	//ltY.setPid(1.2,0,25);//0.58,0.05,370);
	FrontLinetrackerY_.setPid(1.2,0,16);
	BackLinetrackerY_.setPid(1.2,0,16);
	driveX.setPid(0.1,0,1.5);		
	driveY.setPid(0.1,0,1.5);

 	//initGY88();
//  	startTime = millis();
//  	//uart0_puts("down loop \r\n");
//  	while((millis() - startTime) < 500){	//take 500 ms to set setpoint of compass
//   		initialCompassAngle = getYawGY88();
//  		//uart0_puts("1st \r\n");
//   		compass.FirstData = false;
//   		compass.SETPOINT = initialCompassAngle;
//  	}
	
}


void movx(int distance_setpoint, int direction,int speed){
	inverseKinematicTrue = true;
	distanceX = abs(encoderX.getdistance());
	driveX.SETPOINT = distance_setpoint;
	if(PidUpdateFlagDriveX)
	{
		movingyfront = false;
		movingyback = false;
		driveX.input = distanceX;
		PidUpdateFlagDriveX = false;
		if(distanceX >= 600){
			driveX.error = driveX.SETPOINT - driveX.input;
			driveX.Iterm += driveX.ki * driveX.error;
			if(driveX.FirstData){
				driveX.prevInput = driveX.input;
				driveX.FirstData = false;
			}
			if(abs(driveX.Iterm) > 10){
				if(driveX.Iterm > 0)	driveX.Iterm = 10;
				if(driveX.Iterm < 0)	driveX.Iterm = -10;
			}
			if((driveX.error) > 0){
				driveX.output = driveX.kp * driveX.error + driveX.Iterm - driveX.kd*(driveX.input - driveX.prevInput);
			}
			else{
				driveX.output = 0;
			}
			driveX.prevInput = driveX.input;
			//////////////////////////////////////////////////////
			if(abs(driveX.output) > speed){
				if(driveX.output >0)	driveX.output = speed;
				else						driveX.output = -speed;
			}
 			if(abs(driveX.output) < 20){
 				if(movingxfront)		driveX.output = 20;
 				else if(movingxback)	driveX.output = -20;
 			}
			//////////////////////////////////////////////////////
			velocity_robot[0] = driveX.output;
		}
		else{
			if(movingFromStartPoint){
				velocity_robot[0] = 30 + 0.316 * distanceX;
			}
			else{
				velocity_robot[0] = 60 + 0.15 * distanceX;
			}
		}
		if(direction == Front){
			velocity_robot[0] = velocity_robot[0];
			movingxfront = true;
			movingxback = false;
			movingyback = false;
			movingyfront = false;
		}
		else if(direction == Back){
			velocity_robot[0] = -abs(velocity_robot[0]);
			movingxfront = false;
			movingxback = true;
			movingyfront = false;
			movingyback = false;
		}
	
	}
	if(movingFromStartPoint){
		if(giveComponent){
			velocity_robot[1] = -30;
		}
		else{
			velocity_robot[1] = -25;
		}
	}
	else{
		velocity_robot[1] = 0;
		
	}
	//calculateCompassPID();
// 	uart0_putint(velocity_robot[0]);
// 	uart0_puts("  hello\r\n");
	velocity_robot[2] = 0;
	
}

void holdposition(){
	compass.SETPOINT = 181;
	inverseKinematicTrue = true;
	velocity_robot[0]  = 0;
	velocity_robot[1] = 0;
	velocity_robot[2] = 0;
	//compass.setPid(8,0,52);//4.2,0.24,32);	//5.1,0,31
	//calculateCompassPID();
}



#endif /* DRIVE_H_ */