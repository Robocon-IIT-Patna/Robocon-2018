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

#include "uart.h"
#include "encoder.h"
#include "Flags.h"
#include "qmccompass.h"
#include <util/delay.h>
#include <math.h>

extern uint8_t change;
extern encoder encoderY, encoderX;

////////////////////////////////////////////////////////////////////////////////
extern bool PidUpdateFlagCompass;
extern bool PidUpdateFlagDriveX;
extern bool PidUpdateFlagLinetracker;
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



bool lineMeet = true;
bool movingx = false;
bool movingyfront = false;
bool movingyback = false;
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
	int SETPOINT;	
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
	inline void incrSetpoint(){SETPOINT += change;}
	inline void dcrSetpoint(){SETPOINT -= change;}
	inline int getSETPOINT(){return SETPOINT;}
	inline float getkp(void){return kp;}
	inline float getki(void){return ki;}
	inline float getkd(void){return kd;}
	
};
bodyPid ltX,ltY,compass,driveX,driveY;



/////////////////////////////////////////////////////////
bool Stable_Robot(void);
void calculateCompassPID(void);
void calculatevel();
int filterLineTrackerData(int);
void calculateLineTrackerYPid();
void movx();
void movy();
void movYForwardSlow();
void initializeAll();
void sendDataToSlave();
void BrakeMotor();
int getLineTrackerYdata();

inline void linetrackerXjunctionWatch();
inline void lintrackerYjunctionWatch();

inline void linetrackerXjunctionWatchOff();
inline void linetrackerYjunctionWatchOff();
////////////////////////////////////////////////////

void BrakeMotor(){
	PORTK ^= (1<<PK1);
}

void sendDataToSlave(void){
//	uart0_puts("a");
// 	I2C_Start(0x20);
// 	I2C_Write_byte_array(bufferMotorSpeed,4);
// 	I2C_Stop();
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
	PCMSK2 |= (1<<PCINT16);
}
inline void linetrackerXjunctionWatchOff(void){
	PCMSK0 &= ~(1<<PCINT4);
}
inline void linetrackerYjunctionWatchOff(void){
	PCMSK2 &= ~(1<<PCINT16);
}


void calculateCompassPID(void)
{
	if(PidUpdateFlagCompass && compassPID)
	{
		
		compass.input = get_Angle();
		
		
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

		if (abs(compass.Iterm) > 0.1*compass.Max_output)
		{
			if(compass.Iterm > 0)
				compass.Iterm = 0.1*compass.Max_output;
			else
				compass.Iterm = -0.1*compass.Max_output;
		}
		
		if (abs(compass.error) > 2 )
		{
			compass.output = compass.kp*compass.error	-	compass.kd*(compass.input-compass.prevInput)	+	compass.Iterm;
		}
		else
		{
			compass.Iterm = 0;
			compass.output = 0;
		}
			
		compass.prevInput = compass.input;
		//uart0_puts("\tprevInput= ");
		//uart0_putint(compass.prevInput);
		
		if (abs(compass.output) > compass.Max_output)
		{
			compass.output = (compass.output > compass.Max_output) ?	compass.Max_output : -compass.Max_output;
		}

		velocity_robot[2] = -compass.output;
		
		PidUpdateFlagCompass = false;
	}
	
	if(!compassPID){
		velocity_robot[2] = 0;
	}
}

bool Stable_Robot(void)
{
	uint16_t _angle = 0;
	uint16_t _get_angle = 0;
	for (uint8_t i = 0; i< 100; i++)
	{
		_get_angle = get_Angle();
		if(_get_angle<(compass.SETPOINT+1) && _get_angle>(compass.SETPOINT-1))
		{ _angle += get_Angle();}
	}
	_angle= _angle/100;
	if (abs(compass.SETPOINT - _angle) <=2) return 1;
	else return 0;
}


void calculatevel()	//use matrix to find setpoint of individual motor and store in bufferMotorSpeed and send to slave
{
	for(int i=0;i<4;i++)
	{
		velocity_motor[i] = 0;
		for(int j=0;j<3;j++)
		{
			velocity_motor[i] += velocity_robot[j] * coupling_matrix[i][j];
			
		}
	}
	bufferMotorSpeed[0] = ((velocity_motor[0]) * 23)/249;	  
	bufferMotorSpeed[1] = ((velocity_motor[1]) * 23)/249;	  
	bufferMotorSpeed[2] = ((velocity_motor[2]) * 23)/249;	  
	bufferMotorSpeed[3] = ((velocity_motor[3]) * 23)/249 ;
	
	sendDataToSlave();  
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



void initializeAll()
{
	
	compass.Set_Max_Min_Output(40,0);	
	
	ltY.SETPOINT = 45;
	compass.setPid(2,0,31);//2,0,31);//4,0.09,18);	//5.5, 0, 500 , 2.1,0.04,32
	ltY.setPid(0.58,0.05,370);//0.58,0.05,370);
	driveX.setPid(0.15,0,1.5);		
	driveY.setPid(0.15,0,1.5);
	init_QMC5883L();
	
	if (compass.FirstData)
	{
		initialCompassAngle = get_Angle();
		compass.FirstData = false;
		compass.SETPOINT = initialCompassAngle;
	}
	
	
}

void movx(int distance_setpoint, int direction, uint8_t maxSpeed_u8, uint8_t minSpeed_u8){
	//compass.setPid(2.1,0.04,32);
	distanceX = abs(encoderX.getdistance());
	driveX.SETPOINT = distance_setpoint;
	if(PidUpdateFlagDriveX)
	{
		movingx = true;
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
			if(abs(driveX.output) > maxSpeed_u8){
				if(driveX.output >0)	driveX.output =maxSpeed_u8;	//150
				else						driveX.output = -maxSpeed_u8;	//150
			}
 			if(abs(driveX.output) < minSpeed_u8){
 				if(driveX.output >= 0)	driveX.output = minSpeed_u8;  //30
 				else					driveX.output = -minSpeed_u8;	//30
 			}
			//////////////////////////////////////////////////////
			velocity_robot[0] = driveX.output;
		}
		else{
				velocity_robot[0] = 60 + 0.45*distanceX;
		}
		if(direction == Front){
			velocity_robot[0] = velocity_robot[0];
		}
		else if(direction == Back){
			velocity_robot[0] = -abs(velocity_robot[0]);
		}
	
	}
	velocity_robot[1] = 0;
	//velocity_robot[2] = 0;
	calculateCompassPID();
}

void movy(int distance_setpoint, int direction, uint8_t maxSpeed_u8, uint8_t minSpeed_u8)
{
	//compass.setPid(2.1,0.04,32);
	distanceY = abs(encoderY.getdistance());
	driveY.SETPOINT = distance_setpoint;
	if(PidUpdateFlagDriveY)
	{
			PidUpdateFlagDriveY = false;
			if(distanceY >= 200)
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
					driveY.output = driveY.kp * driveY.error + driveY.Iterm - driveY.kd*(driveY.input - driveY.prevInput);
				}
				else{
					driveY.output = 0;
				}
				driveY.prevInput = driveY.input;
				////////////////////////////////////////////////////////////
				if(abs(driveY.output) >= maxSpeed_u8){
					if(driveY.output > 0)	driveY.output = maxSpeed_u8;	// 100
					else						driveY.output = -maxSpeed_u8;
				}
 				if(abs(driveY.output) < minSpeed_u8){
 					if(driveY.output >= 0)	driveY.output = minSpeed_u8;	//20
 					else					driveY.output = -minSpeed_u8;
 				}
				/////////////////////////////////////////////////////////
				velocity_robot[1] = driveY.output;
			}
			else
			{
				//uart0_puts("ramp up\t");
				velocity_robot[1] = 40 + (distanceY*0.3);
				//uart0_putint(velocity_robot[1]);
				//uart0_puts("\r\n");
			}
			if(direction == Front){
				movingyfront = true;
				movingyback = false;
				movingx = false;
				velocity_robot[1] = velocity_robot[1];
			}
			else if(direction == Back){
				movingyback = true;
				movingyfront = false;
				movingx = false;
				velocity_robot[1] = -abs(velocity_robot[1]);
			}
		
	}
	//velocity_robot[0] = 0;
	calculateLineTrackerYPid();
	//velocity_robot[2] = 0;
	calculateCompassPID();
}

void movYForwardSlow(){
	movingx = false;
	movingyfront = true;
	movingyback = false;
	compass.setPid(2,0,31);
	velocity_robot[1] = 30;
	calculateLineTrackerYPid();
	calculateCompassPID();
}

void holdposition(){
	velocity_robot[0]  = 0;
	velocity_robot[1] = 0;
	//velocity_robot[2] = 0;
	compass.setPid(4.2,0.24,32);	//5.1,0,31
	calculateCompassPID();
}



#endif /* DRIVE_H_ */