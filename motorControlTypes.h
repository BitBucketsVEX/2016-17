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
typedef struct
{
	tMotor mId;
	tSensors sId;
	bool enabled;
	float commandDeg;
	float encoderDeg;
	float encoderRPM;
	float kp;
	float ki;
	float kd;
	float kb;
	int pid;
	float biasDeg;

	float limitFactor;

	// elements to keep track of coupled stabilization
	bool stabilize;
	float startMasterAngle_deg;
	float startSlaveAngle_deg;

} motorControlType;

// Define operations for the motorControlType (similar to what we might have in C++ so the concepts will port
// easily if ROBOTC ever grows up
void constructMotorControl(motorControlType *this, tMotor mId, tSensors sId, float kp, float ki, float kd, float kb, float biasDeg, float limitFactor)
{
	this->mId = mId;
	this->sId = sId;
	this->enabled = true;
	this->commandDeg = 0.0;
	this->encoderDeg = 0.0;
	this->encoderRPM = 0.0;
	this->kp = kp;
	this->ki = ki;
	this->kd = kd;
	this->kb = kb;
	this->pid = 0;
	this->biasDeg = biasDeg;

	this->limitFactor = limitFactor;

	this->stabilize = false;
	this->startMasterAngle_deg = 0.0;
	this->startSlaveAngle_deg = 0.0;

}

// Each motor can be set to an independent position
// So we provide accessor functions to represent the interface
// This is a practice similar to what is seen in object oriented
// designs rather than allowing direct access to the elements
// This allows us to modify behavior globally instead of needing
// to find all uses

void setEnable(motorControlType *this, bool flag)
{
	this->enabled = flag;
}

bool getEnable(motorControlType *this)
{
	return this->enabled;
}

// For every "set" there is a "get" function
void setPosition(motorControlType *this, float positionDeg)
{
	this->commandDeg = positionDeg;
}

// In this case the get function is not the inverse of the set
// Instead we will return the latest encoder position as read
// by the maintainPosition function
float getPosition(motorControlType *this)
{
	return this->encoderDeg;
}
float getLastCommand(motorControlType *this)
{
	return this->commandDeg;
}

void enableStabilization(motorControlType *this, motorControlType *master)
{
	// Only initialize once; force disable to reinit
	if (! this->stabilize)
	{
		this->stabilize = true;
		this->startMasterAngle_deg = getPosition(master);
		this->startSlaveAngle_deg  = getPosition(this);
	}
}
void disableStabilization(motorControlType *this)
{
	this->stabilize = false;
}

void resetPosition(motorControlType *this)
{
		SensorValue[this->sId] = 0;	// Use the shaft encoder rather than integrated encoders
}

// A function to actually apply the control to each motor
void maintainPosition(motorControlType *this)
{
	long encoder_tick = SensorValue[this->sId];		// Using shaft encoders for the moment

	// Compute error between what we want and what we have
	this->encoderDeg = encoder_tick * DEGREES_PER_TICK;
	float error = this->commandDeg - this->encoderDeg;

	// No error accumulation at this time

	// No error rate at this time
	this->encoderRPM  = 0.0;	  // TODO: No derivative control at this time

	// Compute compensation bias
	float bias = this->kb * cos((this->encoderDeg - this->biasDeg) * PI / 180);

	if (fabs(error) > 0)
	{
		// make the speed proportional to the error
	  float stabilizeGain = 1.0;
	  if (this->stabilize)
	  {
	  	stabilizeGain = 2.0;
	  }
	  this->pid = (int) (this->kp * error);

	  // Add bias compensation (e.g., a gravity compensator when commanding 0 will not hold)
	  this->pid += (int) bias;

	  // Add in integral and derivitive control later if students really need it.
	  // NOTE: with sensors that read position directly this is almost never needed
	  // and students tend to defocus from the real problems.

	  if (this->pid > MAX_MOTOR_COMMAND)
	  {
	  	this->pid = MAX_MOTOR_COMMAND;
	  }
	  else if (this->pid < -MAX_MOTOR_COMMAND)
	  {
	  	this->pid = -MAX_MOTOR_COMMAND;
	  }
	  motor[this->mId] = (int)(this->limitFactor * this->pid);
	}
	else
	{
		// When within the target tolerance, stop.
		motor[this->mId] = (int)bias;
	}
}


#endif
