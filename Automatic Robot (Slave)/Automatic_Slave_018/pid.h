/*
 * pid.h
 *
 * Created: 10/22/2017 5:38:07 PM
 *  Author: abheesh
 */ 


#ifndef PID_H_
#define PID_H_

#include "Motor.h"
#include "encoder.h"
#include "uart.h"
#include "Flags.h"

/////////////////////////////////////////////////
extern Motor m1,m2,m3,m4;
extern encoder e1,e2,e3,e4;
////////////////////////////////////////////////
bool PidUpdateFlagMotor = false;
extern int8_t rcvdata[4];

struct pid
{

	double kp,ki,kd;
	int input , output;
	int error;
	double Iterm;
	int prevError;
	int MOTOR_OCR_VALUE;
	
	inline void incrKP(void){kp += 0.05;}
	inline void incrKD(void){kd += 0.5;}
	inline void incrKI(void){ki += 0.05;}
	inline void dcrKP(void){kp -= 0.05;}
	inline void dcrKI(void){ki -= 0.05;}
	inline void dcrKD(void){kd -= 0.5;}
	inline void reset(void){kp = 0;ki=0;kd=0;}
	inline float getkp(void){return kp;}
	inline float getkd(void){return kd;}
	inline float getki(void){return ki;}
	void setpid(double KP, double KI, double KD)
	{
		kp = KP;
		ki = KI;
		kd = KD;
	}

};

pid ma,mb,mc,md;

void Set_Pid_Movx(void)
{
	ma.setpid(1,0,6);//(0,0,0);//(2.6,0,3);
	mb.setpid(1,0,6);//(0,0,0);//(2.2,0,4);
	mc.setpid(1.45,0,2);//(0,0,0);//(1.9,0,6);
	md.setpid(1,0,3);//(0,0,0);//(1.95,0,6);
}

void Set_Pid_Movy(void)
{
	ma.setpid(0.7,0,1);//(0,0,0);//(2.6,0,3);
	mb.setpid(0.81,0,1);//(0,0,0);//(2.2,0,4);
	mc.setpid(0.77,0,1);//(0,0,0);//(1.9,0,6);
	md.setpid(0.91,0,1);//(0,0,0);//(1.95,0,6);
}


void setTuningsM1(int8_t SETPOINT1)
{
	ma.input = e1.getspeed();
	
		ma.error = SETPOINT1 - ma.input;
	
		ma.Iterm += ma.ki * ma.error;
	
		ma.output = ma.kp * ma.error + ma.Iterm + ma.kd * (ma.error - ma.prevError) ;
	
		ma.prevError = ma.error;
		ma.MOTOR_OCR_VALUE = ma.MOTOR_OCR_VALUE + ma.output;
	
		if (abs(ma.MOTOR_OCR_VALUE) > ICR_TOP){
			if(ma.MOTOR_OCR_VALUE > 0)
				ma.MOTOR_OCR_VALUE = ICR_TOP;
			else 
				ma.MOTOR_OCR_VALUE = -ICR_TOP;
		}
		m1.SetOcrValue(ma.MOTOR_OCR_VALUE);
	
}
void setTuningsM2(int8_t SETPOINT2)
{
	mb.input = e2.getspeed();
		mb.error = SETPOINT2 - mb.input;

		mb.Iterm += mb.ki * mb.error;
	
		mb.output = mb.kp * mb.error + mb.Iterm + mb.kd * (mb.error - mb.prevError) ;
	
		mb.prevError = mb.error;
		mb.MOTOR_OCR_VALUE = mb.MOTOR_OCR_VALUE + mb.output;
	
		if (abs(mb.MOTOR_OCR_VALUE) > ICR_TOP){
			if(mb.MOTOR_OCR_VALUE > 0)
				mb.MOTOR_OCR_VALUE = ICR_TOP;
			else
				mb.MOTOR_OCR_VALUE = -ICR_TOP;
		}
		m2.SetOcrValue(mb.MOTOR_OCR_VALUE);

}

void setTuningsM3(int8_t SETPOINT3)
{
	mc.input = e3.getspeed();

		mc.error = SETPOINT3 - mc.input;
		mc.Iterm += mc.ki * mc.error;
	
		mc.output = mc.kp * mc.error + mc.Iterm + mc.kd * (mc.error - mc.prevError) ;
	
		mc.prevError = mc.error;
		mc.MOTOR_OCR_VALUE = mc.MOTOR_OCR_VALUE + mc.output;
	
		if (abs(mc.MOTOR_OCR_VALUE) > ICR_TOP){
			if(mc.MOTOR_OCR_VALUE > 0)
				mc.MOTOR_OCR_VALUE = ICR_TOP;
			else
				mc.MOTOR_OCR_VALUE = -ICR_TOP;
		}
		m3.SetOcrValue(mc.MOTOR_OCR_VALUE);
	
}
void setTuningsM4(int8_t SETPOINT4)
{
	
	md.input = e4.getspeed();

	
		md.error = SETPOINT4 - md.input;
		md.Iterm += md.ki * md.error;
	
		md.output = md.kp * md.error + md.Iterm + md.kd * (md.error - md.prevError) ;
	
		md.prevError = md.error;
		md.MOTOR_OCR_VALUE = md.MOTOR_OCR_VALUE + md.output;

		if (abs(md.MOTOR_OCR_VALUE) > ICR_TOP){
			if(md.MOTOR_OCR_VALUE > 0)
				md.MOTOR_OCR_VALUE = ICR_TOP;
			else
				md.MOTOR_OCR_VALUE = -ICR_TOP;
		}
		m4.SetOcrValue(md.MOTOR_OCR_VALUE);

}

void computePid()
{
	//if ((rcvdata[1] > 0 && rcvdata[2] > 0) || (rcvdata[1] < 0 && rcvdata[2] < 0) )
	//{
		//Set_Pid_Movy();
	//}
	//else
	//{
		//Set_Pid_Movx();
	//}
	
	if(MotorPidFlag){
		if(PidUpdateFlagMotor){
				setTuningsM1(rcvdata[0]);
				setTuningsM2(rcvdata[1]);
				setTuningsM3(rcvdata[2]);
				setTuningsM4(rcvdata[3]);
			PidUpdateFlagMotor = false;
		}
	}
	else{
		m1.SetOcrValue(rcvdata[0] * 5.4);
		m2.SetOcrValue(rcvdata[1] * 5.4);
		m3.SetOcrValue(rcvdata[2] * 5.4);
		m4.SetOcrValue(rcvdata[3] * 5.4);
	}
}

void stopDrive(){
	m1.StopMotor();
	m2.StopMotor();
	m3.StopMotor();
	m4.StopMotor();
	ma.prevError = mb.prevError = mc.prevError = md.prevError = 0;
	ma.Iterm = mb.Iterm = mc.Iterm = md.Iterm = 0;
	ma.output = mb.output = mc.output = md.output = 0;
	ma.MOTOR_OCR_VALUE = mb.MOTOR_OCR_VALUE = mc.MOTOR_OCR_VALUE = md.MOTOR_OCR_VALUE = 0;
	//clear all the rcvdata below
	rcvdata[0] = 0;
	rcvdata[1] = 0;
	rcvdata[2] = 0;
	rcvdata[3] = 0;
	//clear all the offset below
}


#endif /* PID_H_ */