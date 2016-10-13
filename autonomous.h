#include "convenientMacros.h"
#include "armControl.h"
#include "driveControl.h"
#include "motorControlTypes.h"

const float RADIUS_OF_WHEEL = 3.75 * 2.54 / 100; // in meters
const float WHEEL_VEL = MAX_RPM * PI * RADIUS_OF_WHEEL / 30; // Max wheel speed in m/s

const float MAT_LENGTH = 1.18956666667; // you guesses it! meters!


void forward(float dist, int t) {
	float mps = dist / t; // m / s
	// WHEEL_VEL = k * 127
	// k = WHEEL_VEL / 127;
	// <some rpm> = WHEEL_VEL * instructed vel / 127
	// instructed vel = 127 * <some rpm> / WHEEL_VEL
	int vel = (int) MAX_MOTOR_COMMAND * mps / WHEEL_VEL;
	vel = BOUND(vel, -MAX_MOTOR_COMMAND, MAX_MOTOR_COMMAND);
	motor[frontLeft] = motor[backLeft] = motor[frontRight] = motor[backRight] = vel;
	turnCoef = 0;
	wait1Msec(t * 1000);
	motor[frontLeft] = motor[backLeft] = motor[frontRight] = motor[backRight] = 0;
}
