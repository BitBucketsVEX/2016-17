// VEX Team 5485 Drive Control

// Only include this once per scope to avoid duplicate definition warnings
#ifndef DRIVE_CONTROL_H
#define DRIVE_CONTROL_H

#include "convenientMacros.h"

// -----------------------------------------------------------------------------
// Drive Control
// -----------------------------------------------------------------------------






// -----------------------------------------------------------------------------
// for autonomous.h
float DIRECTION = PI / 2; // radians
const float ROBOT_RADIUS = 9 * 2.54 / 100;
float RobotX = 0;
float RobotY = 0;
// -----------------------------------------------------------------------------


const float RADIUS_OF_WHEEL = 3.75 * 2.54 / 100; // in meters
const float WHEEL_VEL = MAX_RPM * PI * RADIUS_OF_WHEEL / 30; // Max wheel speed in m/s

const float MAT_LENGTH = 1.18956666667; // you guesses it! meters!




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

	//vel = BOUND(vel, -MAX_MOTOR_COMMAND, MAX_MOTOR_COMMAND); //MIN(vel, MAX(vel, MAX_MOTOR_COMMAND), -MAX_MOTOR_COMMAND);
	/*if (vel > MAX_MOTOR_COMMAND) {
		vel = MAX_MOTOR_COMMAND;
	}
	if (vel < -MAX_MOTOR_COMMAND) {
		vel = -MAX_MOTOR_COMMAND;
	}*/
	vel = BOUND(vel, -MAX_MOTOR_COMMAND, MAX_MOTOR_COMMAND);

	if (vel < 0) {
		pwm = -linear[-vel];
	} else {
		pwm = linear[vel];
	}
	return pwm;
	/*//MIN(vel, -MAX_MOTOR_COMMAND);

	/*if (vel > MAX_MOTOR_COMMAND) {
		vel = MAX_MOTOR_COMMAND;
	}
	if (vel < -MAX_MOTOR_COMMAND) {
		vel = -MAX_MOTOR_COMMAND;
	}

	pwm = SIGN(vel) * linear[abs(vel)];
	if (vel < 0) {
		pwm = -linear[-vel];
	} else {
		pwm = linear[vel];
	}
	return pwm;*/
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
	  int STEER = (MAX_STEER * turnCoef / 100);
	  int left = linearize(driveSpeed + STEER);
	  int right = linearize(driveSpeed - STEER);

	  // Minimize latency while hogging CPU
	  // Hogging CPU here ensures that all 4 motors receive
	  // commands "atomically", but we hold the state only
	  // as long as we absolutely need to so.
	  hogCPU();

	  /// TODO: Consider slaving motors
	  /// OR, at minimum put each motor assignmen on a separate line
	  // to speed it up slightly (eliminates an I/O read if command not shadowed)
	  motor[backLeft] = motor[frontLeft] = left;
	  motor[backRight] = motor[frontRight] = right;
		releaseCPU();

		/// TODO: This direction/position estimate is open loop based on
		/// assumed period and will not be able maintain accuracy in any deterministic way.
		/// We will need to use position encoders and keep track of
		/// the difference between the encoders for direction
		///
		/// IF we decide to use a time-based backup it will only start
		/// if a sensor fault is detected ... which for now is too much
		/// complexity for our little robot. But if we do, the time MUST
		/// be based on a measured time NOT the scheduled time.
		/// Measuring time is via the time1[] variable and simply needs to
		/// be retained each time we reach the top of the loop
	  float deltaDirection = 2 * STEER * DRIVE_SPEED_CONTROL_PERIOD_MSEC / ROBOT_RADIUS;
	  DIRECTION += deltaDirection;
	  while (DIRECTION >= 2 * PI) {
	  	DIRECTION -= 2 * PI;
	  }
	  while (DIRECTION < 0) {
		  DIRECTION += 2 * PI;
		}
		 // keep track of robot's X and Y positions
		RobotX += driveSpeed * cos(DIRECTION);
		RobotY += driveSpeed * sin(DIRECTION);



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
	/// TODO: Insert code similar to arm control but this time
  /// we will keep track of (x,y) and heading, stopping only
  /// when both are achieved

	/// TODO: The simplest thing to do is to:
	///       1) turn in direction of position target
	///       2) move to that position in a straight line
	///       3) turn to final heading
  /// HOWEVER, that protocol should NOT be here, but rather
  /// in the commands being ingested here.
  /// So, if both position and heading are commanded at the
  /// same time, the heading should be ignored until the
  /// position is achieved. There will be a little work
  /// to ensure that position is deadbanded properly to
  /// allow the turn to complete at the end... simplest
  /// thing is to just send positioning commands and
  /// heading as separate commands
}

#endif
