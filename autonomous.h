#include "convenientMacros.h"
#include "armControl.h"
#include "driveControl.h"
#include "motorControlTypes.h"

const float RADIUS_OF_WHEEL = 3.75 * 2.54 / 100; // in meters
const float WHEEL_VEL = MAX_RPM * PI * RADIUS_OF_WHEEL / 30; // Max wheel RPM in m/s

const float MAT_LENGTH = 1.18956666667; // you guesses it! meters!


void forward(float dist, int t) {
	float mps = dist / t; // m / s
	// WHEEL_RPM = k * 127
	// k = WHEEL_RPM / 127;
	// <some rpm> = WHEEL_RPM * instructed vel / 127
	// instructed vel = 127 * <some rpm> / WHEEL_RPM
	int vel = (int) 127 * mps / WHEEL_VEL;
	motor[frontLeft] = motor[backLeft] = motor[frontRight] = motor[backRight] = vel;
	turnCoef = 0;
	wait1Msec(t * 1000);
	motor[frontLeft] = motor[backLeft] = motor[frontRight] = motor[backRight] = 0;
}
