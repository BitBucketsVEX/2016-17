// VEX Team 5485 Drive Control

// Only include this once per scope to avoid duplicate definition warnings
#ifndef DRIVE_CONTROL_H
#define DRIVE_CONTROL_H

#include "convenientMacros.h"

// -----------------------------------------------------------------------------
// Drive Control
// -----------------------------------------------------------------------------

int linear[129] = {
	0, 0, 18, 18, 19, 19, 19, 19, 19, 19, 20, 20, 20, 20,
	21, 21, 21, 21, 21, 21, 22, 22, 22, 22, 23, 23, 24, 24, 24, 25, 25, 25,
	25, 26, 26, 26, 26, 27, 27, 27, 27, 28, 28, 29, 29, 29, 29, 30, 30, 30,
	31, 31, 31, 32, 32, 33, 33, 33, 34, 34, 35, 35, 36, 36, 36, 37, 37, 38,
	38, 39, 39, 40, 40, 41, 41, 41, 42, 43, 43, 44, 44, 45, 45, 46, 47, 48,
	49, 49, 50, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 61, 62, 64, 65, 66,
	67, 68, 69, 70, 72, 73, 75, 76, 77, 79, 81, 83, 84, 84, 85, 85, 86, 86,
	87, 87, 88, 90, 96, 105, 105
};

int linearize(int vel) {
	int pwm;

	if (vel > MAX_MOTOR_COMMAND) {
		vel = MAX_MOTOR_COMMAND;
	}
	if (vel < -MAX_MOTOR_COMMAND) {
		vel = -MAX_MOTOR_COMMAND;
	}
	if (vel < 0) {
		pwm = -linear[-vel];
	} else {
		pwm = linear[vel];
	}
	return pwm;
}

int deadband(int vel) {
	return ((abs(vel) < 24) ? 0 : vel);
}

// Drive control is a separate task to ensure that we can add additional
// control functions independently of the input commands from drive speed
// and turn coefficient
int const DRIVE_SPEED_CONTROL_PERIOD_MSEC = 20;  // number of milliseconds per each control loop
int const MAX_STEER = 50; // percent of drive to apply to steering

int driveSpeed = 0; //The forward drive speed.
int turnCoef = 0; //The turning amount.

#ifdef TEST_SIM
	unsigned int driveCount = 0;
#endif

task driveSpeedControl() {
	for EVER {
		// Linearizing will also limit the output
	  const int STEER = (MAX_STEER * turnCoef / 100);

	  // Minimize latency while hogging CPU
	  hogCPU();
	  motor[backLeft] = motor[frontLeft] = linearize(driveSpeed + STEER);
	  motor[backRight] = motor[frontRight] = linearize(driveSpeed - STEER);

	  // Hogging CPU here ensures that all 4 motors receive
	  // commands "atomically"

		releaseCPU();

		#ifdef TEST_SIM
    // only display in emulator
			displayLCDNumber(0, 8, (driveCount++)%100, 3);
		#endif

		wait1Msec(DRIVE_SPEED_CONTROL_PERIOD_MSEC);
	}
}

// drive position control is only used during autonomous and
// other similar demos to process command sequences maintaining the
// heading and field position
task drivePositionControl() {
	// TODO: Insert code similar to arm control but this time
  // we will keep track of (x,y) and heading, stopping only
  // when both are achieved
}

#endif
