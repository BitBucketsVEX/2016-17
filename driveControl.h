// VEX Team 5485 Drive Control
// Drive control contains both manual and autonomous support functions

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

// ***************************************************************
// Drive Position Control
//
// Support functions for autonomous drive are
//		move     - Relative motion ( positive = forward, negative = backward)
//		moveTo   - field postion via turnTo, forward, turnTo
//		turn		 - Relative motion
//		turnTo   - field orientation
//
// Task responds to enable/disable and command from above funcions
//
// The global variables define the control for the task
// ***************************************************************
const float WHEEL_TRACK_m  = 15.25 * IN_2_M
const float TRACK_RADIUS_m = WHEEL_TRACK_m / 2.0;
const float WHEEL_DIAMETER_m = 4.0 * IN_2_M;
const float WHEEL_RADIUS_M   = WHEEL_DIAMETER_m / 2.0;

// A set of the motors we want to position control during the drivePositionControl task
struct motorControlType driveMotors[4];

// Functions to create and control drive motors as single call
bool driveMotorsConstructed = false;
float driveKp = 2.0;
float driveKi = 0.0;
float driveKd = 0.0;
float driveKb = 0.0;

// Unlike arm control, the left/right is important here
// Provide index to our structure to make mapping easier
enum DriveMotorIds
{
	DMI_FRONT_RIGHT,
	DMI_FRONT_LEFT,
	DMI_BACK_RIGHT,
	DMI_BACK_LEFT
};

void constructDriveMotorControls(void)
{
	if ( ! driveMotorsConstructed)
	{
		constructMotorControl(&driveMotors[DMI_FRONT_RIGHT], frontRight, rightEncoder, driveKp, driveKi, driveKd, driveKb, 0.0);
		constructMotorControl(&driveMotors[DMI_FRONT_LEFT],  frontLeft,  leftEncoder,  driveKp, driveKi, driveKd, driveKb, 0.0);
		constructMotorControl(&driveMotors[DMI_BACK_RIGHT],  backRight,  rightEncoder, driveKp, driveKi, driveKd, driveKb, 0.0);
		constructMotorControl(&driveMotors[DMI_BACK_LEFT],   backLeft,   leftEncoder,  driveKp, driveKi, driveKd, driveKb, 0.0);

		driveMotorsConstructed = true;
	}
}

void resetDrivePosition(void)
{
	for (int i = 0; i < LENGTH(driveMotors); ++i)
	{
		resetPosition(&driveMotors[i]);
	}
}

// Later, we will keep track of where we are
float RobotX_m = 0.0;
float RobotY_m = 0.0;
float RobotHeading_deg = 0.0;


// move - relative distance (meters) from current position and in current direction
void move(float dist_m)
{
	// Required wheel angle is distance / radius
  float commandAngle_deg = RAD_2_DEG * (dist_m / WHEEL_RADIUS_M);

	// Hog the CPU while setting all the positions
  // to ensure they change atomically even for the high priority
  // task
	hogCPU();

	// When moving we can set all of the motors the same
	for (int i = 0; i < LENGTH(driveMotors); ++i)
	{
		// Use the current encoder position associate with each motor
	  // to command the correct offset
	  float angle_deg = getPosition(&driveMotors[i]) + commandAngle_deg;
		setPosition(&driveMotors[i], angle_deg);
	}

	releaseCPU();

	// Wait for position to be reached
	// Two options
	//	1) Busy-wait by sleeping/polling
	//  2) Use a mutex or other notification

	// When the motion is complete we can update the location
	// I.e., Add the distance along a vector in the current heading direction
	// NOTE: This will be approximate since the position encoding and the
	// motor shutdown are not precise, but much more precise than a time-based
	// solution.
	// To improve accuracy later we would use other sensors to measure location
	// and correct our internal variables.

}

// turn - relative angle (degrees) from current direction
void turn(float angle_deg)
{
	/// TODO:
}

void turnTo(float deg)
{

}

void moveTo(float x, float y)
{

}

// drive position control is only used during autonomous and
// other similar demos to process command sequences maintaining the
// heading and field position
task drivePositionControl() {
	// TODO: Insert code similar to drive control but this time
  // we will keep track of (x,y) and heading, stopping only
  // when both are achieved
}

#endif
