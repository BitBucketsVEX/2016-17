
// Only include this once per scope to avoid duplicate definition warnings
#ifndef HAND_CONTROL_H
#define HAND_CONTROL_H

#include "convenientMacros.h"
#include "motorControlTypes.h"

// -------------------------------------------------------------------------
// Hand Control
// -------------------------------------------------------------------------
// A set of the motors we want to position control during the handControl task
struct motorControlType handMotors[2];

// Functions to create and control hand motors as single call
bool handMotorsConstructed = false;
float handKp = 2.0;
float handKi = 0.0;
float handKd = 0.0;
float handKb = 0.0;
float handBiasAngle_deg = 0.0;

void constructHandMotorControls(void)
{
	if ( ! handMotorsConstructed)
	{
		constructMotorControl(&handMotors[0],handRight,     handEncoder, handKp, handKi, handKd, handKb, handBiasAngle_deg);
		constructMotorControl(&handMotors[1],handLeft,      handEncoder, handKp, handKi, handKd, handKb, handBiasAngle_deg);

		handMotorsConstructed = true;
	}
}

void resetHandPosition(void)
{
	for (int i = 0; i < LENGTH(handMotors); ++i)
	{
		resetPosition(&handMotors[i]);
	}
}

void setHandPosition(float angle_deg)
{
	// Hog the CPU while setting all the positions
  // to ensure they change atomically even for the high priority
  // task
	hogCPU();

	for (int i = 0; i < LENGTH(handMotors); ++i)
	{
		setPosition(&handMotors[i], angle_deg);
	}

	releaseCPU();
}

float getHandPosition(void)
{
	return getLastCommand(&handMotors[0]);
}


void maintainHandPosition(void)
{
	for (int i = 0; i < LENGTH(handMotors); ++i)
	{
		// Assume that hand position is controlled at high priority
		// eliminating the need to hog the CPU
		maintainPosition(&handMotors[i]);
	}
}



// Let the motor position control run as a concurrent activity
// to drive and joystick functions; this allows some control over
// priority to ensure position control is tight
bool handControlInitialized = false;
const long HAND_CONTROL_PERIOD_MSEC = 1;
#ifdef TEST_SIM
	unsigned int handCount = 0;
#endif

task handControl()
{
	if ( ! handControlInitialized)
	{
		resetHandPosition();
		handControlInitialized = true;
	}

	for EVER
	{
		maintainHandPosition();

#ifdef TEST_SIM
    // only display in emulator
		displayLCDNumber(0, 2, (handCount++)%100, 3);
#endif

		wait1Msec(HAND_CONTROL_PERIOD_MSEC);	// Let lower priority tasks execute before resuming control
	}
}


#endif
