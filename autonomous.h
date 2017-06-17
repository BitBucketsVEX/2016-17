#include "convenientMacros.h"
#include "armControl.h"
#include "driveControl.h"
#include "motorControlTypes.h"


/// TODO: All of these should send commands to a drivePositioningTask
/// to avoid the openloop timing... these functions should use a
/// wait-for semantic either by blocking on a semaphore or polling
/// at some reasonable rate to see if the drive tasking reached the
/// goal.

void forward(float dist, float t) {
	float mps = dist / t; // mps = m / s, aka velocity
	// WHEEL_VEL = k * 127
	// k = WHEEL_VEL / 127;
	// <some mps> = WHEEL_VEL * instructed vel / 127
	// instructed vel = 127 * <some mps> / WHEEL_VEL
	int vel = (int) MAX_MOTOR_COMMAND * mps / WHEEL_VEL; // m / s
	vel = BOUND(vel, -MAX_MOTOR_COMMAND, MAX_MOTOR_COMMAND);
	motor[frontLeft] = motor[backLeft] = motor[frontRight] = motor[backRight] = vel;
	turnCoef = 0;
	wait1Msec(t * 1000);
	motor[frontLeft] = motor[backLeft] = motor[frontRight] = motor[backRight] = 0;
}

void turn(float deg, float t) {
	// 2 * STEER * deltaT / (pi * r) = deltaAngle;
	// STEER = pi * r * deltaAngle / (deltaT * 2)
	deg *= PI / 180;
	float steer = PI * ROBOT_RADIUS * deg / (t * 2); // m/s
	steer *= MAX_MOTOR_COMMAND / WHEEL_VEL;
	driveSpeed = 0;
	turnCoef = 0;
	motor[frontLeft] = motor[backLeft] = (int) steer;
	motor[frontRight] = motor[backRight] = (int) -steer;
	wait1Msec(t * 1000);
	motor[frontLeft] = motor[backLeft] = motor[frontRight] = motor[backRight] = 0;
}

void turnTo(float deg, float t) {
	deg *= PI / 180;
	float deltaDeg = deg - DIRECTION;
	turn(deltaDeg, t);
}

void moveTo(float x, float y, int turnT, int t) {
	float deltaX = x - RobotX;
	float deltaY = y - RobotY;
	float angle;

	if (deltaX == 0) {
		angle = SIGN(deltaY) * 90;
	} else {
		if (deltaX >= 0 && deltaY >= 0) {
			angle = atan(deltaY / deltaX) * 180 / PI;
		} else if (deltaX <= 0 && deltaY >= 0) {
			angle = 180 - 180 * atan(-deltaY / deltaX) / PI;
		} else if (deltaX <= 0 && deltaY <= 0) {
			angle = 180 + 180 * atan(deltaY / deltaX) / PI;
		} else if (deltaX >= 0 && deltaY <= 0) {
			angle = 270 + 180 * atan(-deltaY / deltaX) / PI;
		}
	}
	turnTo(angle, turnT);
	forward(sqrt(deltaX * deltaX + deltaY * deltaY), t);
}
