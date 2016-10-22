// VEX Team 5485 Motor Control Types

// Only include this once per scope to avoid duplicate definition warnings
#ifndef MOTOR_CONTROL_TYPES_H
#define MOTOR_CONTROL_TYPES_H

#include "convenientMacros.h"

// Motor data sheet
// 						Output Stage  Output Stage	Output Speed 		Stall Torque 		IME Ticks
// 						Driving Gear	Driven Gear		(RPM)	Output 		(N*m)						per Revolution
//Standard 		10t						32t						100							1.67						627.2
//High Speed	14t						28t						160							1.04						392
//Turbo 			18t						24t						240							0.70						261.333

const int MAX_MOTOR_COMMAND = 127;
const float MAX_RPM = 100.0;	// Standard 393
const float TICKS_PER_RPM = (float) MAX_MOTOR_COMMAND / MAX_RPM;


// Encoder data sheet
//	Quadrature Encoders:	360 ticks per revolution of the output shaft
//	269 Motor IME:	      240.4 ticks per revolution of the output shaft
//	393 Motor IME:	      627.2 ticks per revolution of the output shaft
//	393 High Speed IME:	  392 ticks per revolution of the output shaft
//	393 Turbo Speed IME:	261.333 ticks per revolution of the output shaft

const float TICKS_PER_REVOLUTION = 360.0; // Quadrature (Shaft) Encoder
const float TICKS_PER_DEGREE = TICKS_PER_REVOLUTION / 360.0;
const float DEGREES_PER_TICK = 1.0 / TICKS_PER_DEGREE;

// The integrated encoders are convenient but in our case the ones we had
// were broken. The quadrature encoders can be used, but experiments with
// the integrated encoder library functions did not produce expected
// results; either just didn't work or produce unexpected answers.
//
// For this reason we will roll our own simple control structure and
// tasks to keep the arm motors well controlled.

// Declare a stuct that represent identification,
// control parameters, and status. Since RobotC
// is C and not C++, we will define separate functions
// rather than encapsulate the desired behavior

typedef struct {
	tMotor mId;
	tSensors sId;
	float commandDeg;
	float encoderDeg;
	float encoderRPM;
	float kp;
	float ki;
	float kd;
	float kb;
	int pid;
	float biasDeg;
	float prevError;
	float intError;
	int lastTime;
	int delta_t_0;
} motorControlType;

// Define operations for the motorControlType (similar to what we might have in C++ so the concepts will port
// easily if ROBOTC ever grows up
void constructMotorControl(motorControlType *this, tMotor mId, tSensors sId, float kp, float ki, float kd, float kb, float biasDeg) {
	this->mId = mId;
	this->sId = sId;
	this->commandDeg = 0.0;
	this->encoderDeg = 0.0;
	this->encoderRPM = 0.0;
	this->kp = kp;
	this->ki = ki;
	this->kd = kd;
	this->kb = kb;
	this->pid = 0;
	this->biasDeg = biasDeg;
	this->prevError = 0;
	this->intError = 0;
	this->lastTime = time1[timer1];
	this->delta_t_0 = 0;
}

// Each motor can be set to an independent position
// So we provide accessor functions to represent the interface
// This is a practice similar to what is seen in object oriented
// designs rather than allowing direct access to the elements
// This allows us to modify behavior globally instead of needing
// to find all uses

// For every "set" there is a "get" function
void setPosition(motorControlType *this, float positionDeg) {
	this->commandDeg = positionDeg;
}

// In this case the get function is not the inverse of the set
// Instead we will return the latest encoder position as read
// by the maintainPosition function
float getPosition(motorControlType *this) {
	return this->encoderDeg;
}

void resetPosition(motorControlType *this) {
		SensorValue[this->sId] = 0;	// Use the shaft encoder rather than integrated encoders
}

// A function to actually apply the control to each motor
void maintainPosition(motorControlType *this) {
	long encoder_tick = SensorValue[this->sId];		// Using shaft encoders for the moment
	this->encoderRPM  = 0.0;	  // TODO: No derivative control at this time

	this->encoderDeg = encoder_tick * DEGREES_PER_TICK;



	// values for exterpolation control

	float delta_t_0 = this->delta_t_0;
	float delta_t_1;
	 // get change in time since last measurement
	if (time1[timer1] < this->lastTime) {
		delta_t_1 = (1000 - this->lastTime) + time1[timer1];
	} else {
		delta_t_1 = time1[timer1] - this->lastTime;
	}
	float delta_t_2 = 2 * delta_t_1 - delta_t_0; // approximation of next change in time;



	float e_0 = this->prevError;
	float e_1 = this->commandDeg - this->encoderDeg;

	float e_2 = e_1 * (1 + delta_t_2 / delta_t_1) - e_0 * (delta_t_2 / delta_t_1);

	this->delta_t_0 = delta_t_1; // set previous change in time to change in time right now for use in cycle

	// for more information about how these equations/values were derived, go to
	// https://github.com/BitBucketsVEX/2016-17/blob/master/Derivative%20Control.pptx

	this->prevError = e_1;
	this->intError += e_1; // no integral control yet, but for the future!




	if (abs(e_1) > 0) {
		// make the speed proportional to the error, proportional to the cosine of the angle, and have exterpolation control
	  this->pid = (int) (this->kp * e_1);
	  // add
	  this->pid += (int) (this->kb * cos((this->encoderDeg - this->biasDeg) * PI / 180));
	  this->pid += (int) e_2 / delta_t_2; // exterpolation control
	 	//this->pid += (int) this->ki * this->intError; 							or integral control yet

	  //this->pid = BOUND(this->pid, -MAX_MOTOR_COMMAND, MAX_MOTOR_COMMAND); //MAX(this->pid, MAX_MOTOR_COMMAND);
	  //MIN(this->pid, -MAX_MOTOR_COMMAND);
	  this->pid = BOUND(this->pid, -MAX_MOTOR_COMMAND, MAX_MOTOR_COMMAND);
	  /*if (this->pid > MAX_MOTOR_COMMAND) {
	  	this->pid = MAX_MOTOR_COMMAND;
	  } else if (this->pid < -MAX_MOTOR_COMMAND) {
	  	this->pid = -MAX_MOTOR_COMMAND;
	  }*/
	  motor[this->mId] = this->pid;
	} else {
		// When within the target tolerance, stop.
		motor[this->mId] = 0;
	}
}

task timer() {
	for EVER {
		if (time1[timer1] >= 1000) {
			clearTimer(timer1);
		}
	}
}
#endif
