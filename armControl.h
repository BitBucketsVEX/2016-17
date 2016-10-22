
// Only include this once per scope to avoid duplicate definition warnings
#ifndef ARM_CONTROL_H
#define ARM_CONTROL_H

#include "convenientMacros.h"

// -------------------------------------------------------------------------
// Arm Control
// -------------------------------------------------------------------------
// A set of the motors we want to position control during the armControl task
struct motorControlType armMotors[4];

// Functions to create and control arm motors as single call
bool armMotorsConstructed = false;
void constructArmMotorControls(void) {
	if (!armMotorsConstructed) {
		constructMotorControl(&armMotors[0], topRight, shaft, 2.0, 0.0, 0.0, 0.0, -59.0);	// Only proportional control at this time
		constructMotorControl(&armMotors[1], topLeft, shaft, 2.0, 0.0, 0.0, 0.0, -59.0);
		constructMotorControl(&armMotors[2], bottomRight, shaft, 2.0, 0.0, 0.0, 0.0, -59.0);
		constructMotorControl(&armMotors[3], bottomLeft, shaft, 2.0, 0.0, 0.0, 0.0, -59.0);

		armMotorsConstructed = true;
	}
}

void resetArmPosition(void) {
	for (int i = 0; i < LENGTH(armMotors); i++) {
		resetPosition(&armMotors[i]);
	}
}

void setArmPosition(float angleDeg) {
	// Hog the CPU while setting all the positions
  // to ensure they change atomically even for the high priority
  // task
	hogCPU();

	for (int i = 0; i < LENGTH(armMotors); i++) {
		setPosition(&armMotors[i], angleDeg);
	}

	releaseCPU();
}


void maintainArmPosition(void) {
	for (int i = 0; i < LENGTH(armMotors); i++) {
		// Assume that arm position is controlled at high priority
		// eliminating the need to hog the CPU
		maintainPosition(&armMotors[i]);
	}
}



// Let the motor position control run as a concurrent activity
// to drive and joystick functions; this allows some control over
// priority to ensure position control is tight
bool armControlInitialized = false;
// const long ARM_CONTROL_PERIOD_MSEC = 1; moved to motorControlTypes.h
#ifdef TEST_SIM
	unsigned int armCount = 0;
#endif

const long ARM_CONTROL_PERIOD_MSEC = 1;

task armControl() {
	if (!armControlInitialized) {
		resetArmPosition();
		armControlInitialized = true;
	}

	for EVER {
		maintainArmPosition();

#ifdef TEST_SIM
    // only display in emulator
		displayLCDNumber(0, 2, (armCount++) % 100, 3);
#endif

		wait1Msec(ARM_CONTROL_PERIOD_MSEC);	// Let lower priority tasks execute before resuming control
	}
}


#endif
