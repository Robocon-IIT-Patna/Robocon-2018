/*
 * drive.h
 *
 * Created: 10/23/2017 3:16:43 PM
 *  Author: abheesh
 */ 


#ifndef DRIVE_H_
#define DRIVE_H_

#define PI		3.14159265


#include "motor.h"
#include "uart.h"
#include "encoder.h"
#include "Flags.h"
#include "Linetracker.h"
#include <math.h>



extern encoder encoderY, encoderX;
extern Motor m1,m2,m3,m4;
extern Linetracker ltSensorX,ltSensorY;
extern void stopDrive();

volatile bool PidUpdateFlagLinetracker = true;
volatile bool PidUpdateFlagCompass = true;
volatile bool PidUpdateFlagDriveX = true;
volatile bool PidUpdateFlagDriveY = true;

extern volatile int svalue;

////////////////////////////////////////////////////////////////////////////////
int distance_input;
int distance_error;
int distance_iterm;
int distance_output;
int distance_previnput;
double distance_kp = 0.15;
double distance_kd = 1.5;
double distance_ki;

////////////////////////////////////////////////////////////////////////////////
int SETPOINT1;
int SETPOINT2;
int SETPOINT3;
int SETPOINT4;
float velocity_motor[4];
int velocity_robot[3];
float coupling_matrix[4][3] = {{-1,1,1},{-1,-1,1},{1,-1,1},{1,1,1}};  //0.707
double value = PI / 180;
////////////////////////////////////////////////////////////////////////////////

int Compass_angle;
int distanceX;
int distanceY;
int currentDistance;
int difference;
bool fullspeed = false;

int speed = 60;
int previousLineTrackerData = 35;
bool junctionX=false;
bool junctionY = false;
bool stopflag = false;
unsigned int count = 0;
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
	inline void incrSetpoint(){SETPOINT += svalue;}
	inline void dcrSetpoint(){SETPOINT -= svalue;}
	inline int getSETPOINT(){return SETPOINT;}
	inline float getkp(void){return kp;}
	inline float getki(void){return ki;}
	inline float getkd(void){return kd;}
	
};
bodyPid ltX,ltY,compass,driveX,driveY;


/////////////////////////////////////////////////////////
void calculateCompassPID(void);
void calculatevel();
int filterLineTrackerData(int);
void calculateLineTrackerYPid();
void calculateYawPid();
void movx();
void movy();
void initializeData();

void calculateCompassPID(void)
{
	if(PidUpdateFlagCompass && compassPID)
	{
		
		compass.input = Compass_angle;

		
		//uart0_putint(compass.input);
		//if (compass.FirstData)
		//{
			//compass.SETPOINT = compass.input;
			//compass.FirstData = false;
		//}
		
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
		compass.output = compass.kp*compass.error	-	compass.kd*(compass.input-compass.prevInput)	+	compass.Iterm;

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


void calculatevel()
{
	for(int i=0;i<4;i++)
	{
		velocity_motor[i] = 0;
		for(int j=0;j<3;j++)
		{
			velocity_motor[i] += velocity_robot[j] * coupling_matrix[i][j];
			
		}
	}
	SETPOINT1 = (velocity_motor[0] * 23)/249;	  
	SETPOINT2 = (velocity_motor[1] * 23)/249;	  
	SETPOINT3 = (velocity_motor[2] * 23)/249;	  
	SETPOINT4 = (velocity_motor[3] * 23)/249 ;  
}		//use matrix to find setpoint


int filterLineTrackerData(int newData)
{
	int data;
	
	if(previousLineTrackerData == 70 && newData >= 255)
		data = 255;
	else if(previousLineTrackerData == 0 && newData >= 255)
		data = 255;
	else if(previousLineTrackerData >= 0 && previousLineTrackerData <= 70 && abs(newData) > 70)
		data = previousLineTrackerData;
	else
		data = newData;
	previousLineTrackerData = data;
	
	return data;
	
}


void calculateLineTrackerYPid()
{
	if(ltY.FirstData && ltSensorY.Get_Sensors_Data() != 0){
		ltY.prevInput = ltSensorY.Get_Sensors_Data();
		ltY.FirstData = false;
	}
	else if(PidUpdateFlagLinetracker && linetrackerPID){
		ltY.input = filterLineTrackerData(ltSensorY.Get_Sensors_Data());
		if(ltY.input == 255 && ltY.prevInput < 35)
		{
			ltY.leftedgeleft = true;
			ltY.rightedgeleft = false;
		}
		else if(ltY.input == 255 && ltY.prevInput > 35)
		{
			ltY.rightedgeleft = true;
			ltY.leftedgeleft = false;
		}
		if(ltY.input != 255){
			ltY.error = ltY.SETPOINT - ltY.input;
			if((ltY.error) == 0)
			{
				ltY.Iterm = 0;
			}
			if(ltY.error == 0)
				ltY.prevInput = ltY.input;
			ltY.Iterm += ltY.ki * ltY.error;
			ltY.output = ltY.kp * ltY.error + ltY.Iterm - ltY.kd *(ltY.input - ltY.prevInput);
			ltY.prevInput = ltY.input;
			if (abs(ltY.output) > 80)
			{
				if (ltY.output > 0){ltY.output = 80;}
				else{ltY.output = -80;}
			}
			velocity_robot[0] = -ltY.output;
		}
		else if (ltY.rightedgeleft)
		velocity_robot[0] = 60;
		else if(ltY.leftedgeleft)
		velocity_robot[0] = -60;
		PidUpdateFlagLinetracker = false;
		
	}
	if(!linetrackerPID)
		velocity_robot[0] = 0;
	
}
//
//void calculateYawPid()
//{
	//if (compass.FirstData)
	//{
	////	compass.prevInput = 2*UART2Receive();
		//compass.FirstData = false;
	//}
	//else if(PidUpdateFlagCompass && compassPID)
	//{
		////compass.input = 2*UART2Receive();
		//compass.error = compass.SETPOINT - compass.input;
		//if (compass.error > 180)
		//{
			//compass.error = compass.error - 360;
		//}
		//else if (compass.error < -180)
		//{
			//compass.error = compass.error + 360;
		//}
		//if(compass.error == 0) compass.Iterm = 0;
		//float DelInput = compass.input - compass.prevInput;
		////if(DelInput > 180)
			////DelInput = (-360 + DelInput);
		////else if(DelInput < -180)
			////DelInput = (360 + DelInput);
		//compass.Iterm += compass.ki * compass.error;
		//compass.output = compass.kp * compass.error + compass.Iterm - compass.kd*(DelInput);
		//compass.prevInput = compass.input;
		//if (abs(compass.output) > 80)
		//{
			//if (compass.output > 0){compass.output = 80;}
			//else{compass.output = -80;}
		//}
		//velocity_robot[2] = -compass.output;			//for reverse x compensation, +ve sign
		//PidUpdateFlagCompass = false;
	//}
	//if(!compassPID)
		//velocity_robot[2] = 0;
//}


void initializeData()
{
	compass.SETPOINT = 18;
	ltX.SETPOINT = 35;
	ltY.SETPOINT = 35;
	compass.setPid(3,0.185,113);
	ltX.setPid(4.0,0.003,488);
	ltY.setPid(4.0,0.003,488);
	driveX.setPid(0.3,0.0002,5);		//0.3,0.0000004,46
	driveY.setPid(0.3,0.0002,5);
	
	//drive - 0.15, 0.00, 1.5
	//lt.setpid = 4.0, 0.003, 450
	//compass.setpid = 8.9 , 488, 0.008
}

void movx(int distance_setpoint, int direction){
	distanceX = abs(encoderX.getdistance());
	driveX.SETPOINT = distance_setpoint;
	if(PidUpdateFlagDriveX){
		PidUpdateFlagDriveX = false;
		if(distanceX > 200){
			driveX.input = distanceX;
			driveX.error = driveX.SETPOINT - driveX.input;
			driveX.Iterm += driveX.ki * driveX.error;
			if(abs(driveX.Iterm) > 30){
				if(driveX.Iterm > 0)	driveX.Iterm = 30;
				if(driveX.Iterm < 0)	driveX.Iterm = -30;
			}
			if((driveX.error) > 0){
				driveX.output = driveX.kp * driveX.error + driveX.Iterm - driveX.kd*(driveX.input - driveX.prevInput);
			}
			else if (driveX.error <= 0){
				stopDrive();
				driveX.output = 0;
			}
			driveX.prevInput = driveX.input;
			if(abs(driveX.output) > 130){
				if(driveX.output >130)	driveX.output = 130;
				else						driveX.output = -130;
			}
			velocity_robot[0] = driveX.output;
		}
		else{
				velocity_robot[0] = 40 + 0.6*distanceX;
		}
	}
	if(direction == Front){
		velocity_robot[0] = velocity_robot[0];
	}
	else if(direction == Back){
		velocity_robot[0] = -abs(velocity_robot[0]);
	}
	velocity_robot[1] = 0;
	calculateCompassPID();
	calculatevel();
}

void movy(int distance_setpoint, int direction){
	distanceY = abs(encoderY.getdistance());
	driveY.SETPOINT = distance_setpoint;
	if(PidUpdateFlagDriveY){
		PidUpdateFlagDriveY = false;
		if(distanceY > 200){
			driveY.input = distanceY;
			driveY.error = driveY.SETPOINT - driveY.input;
			driveY.Iterm += driveY.ki * driveY.error;
			if(abs(driveY.Iterm) > 30){
				if(driveY.Iterm > 0)	driveY.Iterm = 30;
				if(driveY.Iterm < 0)	driveY.Iterm = -30;
			}
			if(abs(driveY.error) > 0){
				driveY.output = driveY.kp * driveY.error + driveY.Iterm - driveY.kd*(driveY.input - driveY.prevInput);
			}
			else{
				driveY.output = 0;
				stopDrive();
			}
			driveY.prevInput = driveY.input;
			if(abs(driveY.output) > 60){
				if(driveY.output >60)	driveY.output = 60;
				else						driveY.output = -60;
			}
			velocity_robot[1] = driveY.output;
		}
		else{
			velocity_robot[1] = 60 + (distanceY*0.65);
		}
	}
	if(direction == Front){
		velocity_robot[1] = velocity_robot[1];
	}
	else if(direction == Back){
		velocity_robot[1] = -abs(velocity_robot[1]);
	}
	velocity_robot[0] = 0;
	velocity_robot[2] = 0;
	calculatevel();
}

void movYForwardSlow(){
	velocity_robot[0] = 0;
	velocity_robot[1] = 30;
	velocity_robot[2] = 0;
	calculatevel();
}


#endif /* DRIVE_H_ */