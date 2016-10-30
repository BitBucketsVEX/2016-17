// VEX Team 5485 Drive Control
// Drive control contains both manual and autonomous support functions

// Only include this once per scope to avoid duplicate definition warnings
#ifndef DRIVE_CONTROL_H
#define DRIVE_CONTROL_H

#include "convenientMacros.h"

// -----------------------------------------------------------------------------
// Drive Control
// -----------------------------------------------------------------------------

int linear[129] =
{
  0, 0, 18, 18, 19, 19, 19, 19, 19, 19, 20, 20, 20, 20,
  21, 21, 21, 21, 21, 21, 22, 22, 22, 22, 23, 23, 24, 24, 24, 25, 25, 25,
  25, 26, 26, 26, 26, 27, 27, 27, 27, 28, 28, 29, 29, 29, 29, 30, 30, 30,
  31, 31, 31, 32, 32, 33, 33, 33, 34, 34, 35, 35, 36, 36, 36, 37, 37, 38,
  38, 39, 39, 40, 40, 41, 41, 41, 42, 43, 43, 44, 44, 45, 45, 46, 47, 48,
  49, 49, 50, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 61, 62, 64, 65, 66,
  67, 68, 69, 70, 72, 73, 75, 76, 77, 79, 81, 83, 84, 84, 85, 85, 86, 86,
  87, 87, 88, 90, 96, 105, 105
};

int linearize(int vel)
{
  int pwm;

  if (vel > MAX_MOTOR_COMMAND)
  {
    vel = MAX_MOTOR_COMMAND;
  }
  if (vel < -MAX_MOTOR_COMMAND)
  {
    vel = -MAX_MOTOR_COMMAND;
  }
  if (vel < 0)
  {
    pwm = -linear[-vel];
  }
  else
  {
    pwm = linear[vel];
  }
  return pwm;
}

int deadband(int vel)
{
  return ((abs(vel) < 24) ? 0 : vel);
}

// Drive control is a separate task to ensure that we can add additional
// control functions independently of the input commands from drive speed
// and turn coefficient
int const DRIVE_SPEED_CONTROL_PERIOD_MSEC = 20;  // number of milliseconds per each control loop
float MAX_STEER = 50; // percent of drive to apply to steering

int driveSpeed = 0; //The forward drive speed.
int turnCoef = 0; //The turning amount.

#ifdef TEST_SIM
unsigned int driveCount = 0;
#endif

task driveSpeedControl()
{
  for EVER
  {
    // Linearizing will also limit the output
    MAX_STEER = 100 - abs(driveSpeed) / (0.02 * MAX_MOTOR_COMMAND);
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
//    move     - Relative motion ( positive = forward, negative = backward)
//    moveTo   - field postion via turnTo, forward, turnTo
//    turn     - Relative motion
//    turnTo   - field orientation
//
// Task responds to enable/disable and command from above funcions
//
// The global variables define the control for the task
// ***************************************************************
const float WHEEL_TRACK_m  = 16.0 * IN_2_M;
const float WHEEL_DIAMETER_m = 4.0 * IN_2_M;
const float TRACK_WHEEL_RATIO = WHEEL_TRACK_m / WHEEL_DIAMETER_m;
const float WHEEL_RADIUS_M   = WHEEL_DIAMETER_m / 2.0;
const float WHEEL_TRACK_RATIO = WHEEL_TRACK_m / WHEEL_DIAMETER_m;

// A set of the motors we want to position control during the drivePositionControl task
struct motorControlType driveMotors[4];

// Functions to create and control drive motors as single call
bool driveMotorsConstructed = false;
float driveKp = 0.2;
float driveKi = 0.0;
float driveKd = 0.0;
float driveKb = 0.0;

// Later, we will keep track of where we are
bool moving = false;                  // keep track of commands in progress or incomplete
bool updated = true;

float RobotX_m = 0.0;                 // x is +right        (0 , 0) is the original robot position
float RobotY_m = 0.0;                 // y is +toward goal
float RobotOrientation_rad = PI / 2.0;  // 0 angle is along x-axis
float RobotHeading_deg = (RobotOrientation_rad - PI / 2.0) * RAD_2_DEG;         // 0 is toward the goal... keep in degrees for debuging

// Unlike arm control, the left/right is important here
// Provide index to our structure to make mapping easier
enum DriveMotorIds
{
  DMI_FRONT_RIGHT,
  DMI_FRONT_LEFT,
  DMI_BACK_RIGHT,
  DMI_BACK_LEFT
};

// This constructor function initializes the struct array
// This is necessary because RobotC does not support the C99 standard
// for initializer lists
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

// Always initialize to a known position
void resetDrivePosition(void)
{
  for (int i = 0; i < LENGTH(driveMotors); ++i)
  {
    resetPosition(&driveMotors[i]);
  }
}

void maintainDrivePosition(void)
{
  for (int i = 0; i < LENGTH(driveMotors); ++i)
  {
    // Assume that drive position is controlled at high priority
    // eliminating the need to hog the CPU
    maintainPosition(&driveMotors[i]);
  }
}

// getRemainingAngle_deg - returns the remaining angle to go
// based on the command and position of each motor
float getRemainingAngle_deg(void)
{
  // Average the distance to go based for all of the motors
  // even if the same encoder is used for multiple motors
  // This keeps the code simple and robust to changes in
  // the number of motors and encoders.
  float remainingAngle_deg = 0.0;
  int n = 0;
  for (int i = 0; i < LENGTH(driveMotors); ++i)
  {
    // Accumulate the angle remaining on enabled motors
    if (getEnable(&driveMotors[i])
    {
      remainingAngle_deg += abs(getLastCommand(&driveMotors[i]) - getPosition(&driveMotors[i]));
      ++n;
    }
  }

  // Average the angle of all of the motors (we assume they are moving in the same direction)
  remainingAngle_deg /= n;

  // Convert the remaining wheel angle to distance
  return remainingAngle_deg;
}

// getRemainingDistance_m - returns the remaining distance to go
// based on the command and position of each motor
float getRemainingDistance_m(void)
{
  // Convert the remaining wheel angle to distance
  return (getRemainingAngle_deg() * DEG_2_RAD) * WHEEL_RADIUS_M;
}

// Polling function to test for all motors reaching their desired position
bool isDriveStopped(void)
{
  bool isStopped = true;
  for (int i = 0; i < LENGTH(driveMotors); ++i)
  {
    // Consider a drive motor stopped when within 1 degree
    // Accumulate the status; if any one motor has not stopped we return false
    isStopped &= (abs(getLastCommand(driveMotors[i]) - getPosition(driveMotors[i])) > 1);
  }

  return isStopped;
}

// move - relative distance (meters) from current position and in current direction
void move(float dist_m)
{
  // Required wheel angle is distance / radius
  float commandAngle_deg = RAD_2_DEG * (dist_m / WHEEL_RADIUS_M);

  // Hog the CPU while setting all the positions
  // to ensure they change atomically even for the high priority
  // task
  hogCPU();
  updated = false;
  moving = true;

  // When moving we can set all of the motors the same
  for (int i = 0; i < LENGTH(driveMotors); ++i)
  {
    // Use the current encoder position associate with each motor
    // to command the correct offset
    float angle_deg = getPosition(&driveMotors[i]) + commandAngle_deg;
    setEnable(&driveMotors[i], true);
    setPosition(&driveMotors[i], angle_deg);
  }

  releaseCPU();

  // Wait for position to be reached
  // Two options
  //  1) Busy-wait by sleeping/polling
  //  2) Use a mutex or other notification
  //
  // For simplicity we will simply busy-wait for all of the motors to
  // reach their respective commands (to within a tolerance)
  // This wait is indefinitie, for now no timeout (could add later)
  // The point is that we will give up our timeslice as efficiently
  // as possible since we are not doing anything useful.
  //
  // There is one advantage to the polling; it allows us to monitor
  // the progress and use it to keep track of the vector, so far.
  // If we were to be disabled (task stops) we would have a pretty
  // good idea where we were at the moment we stopped... not perfect
  // but pretty good
  float distanceTraveled_m = 0.0;
  while ( ! isDriveStopped())
  {
    // Get the remaining distance to compute how far we moved so far
    // Keep doing this
    distanceTraveled_m = dist_m - getRemainingDistance_m();

    // TODO: We can add an abort here if we want
    // Just need to stop motors at current angle
    // and then recompute the distance traveled and break

    EndTimeSlice();
  }



  // When the motion is complete we can update the location
  // I.e., Add the distance along a vector in the current heading direction
  // NOTE: This will be approximate since the position encoding and the
  // motor shutdown are not precise, but much more precise than a time-based
  // solution.
  //
  // We will assume that the heading error caused by the asymmetric encoding
  // is negligible at the moment (i.e., we used a average via the getRemainingDistance_m
  // function, above)
  //
  // To improve accuracy later we would use other sensors to measure location
  // and correct our internal variables.
  moving = false;

  // Compute the vector that we will move
  float x_m = distanceTraveled_m * cos(RobotOrientation_rad);
  float y_m = distanceTraveled_m * sin(RobotOrientation_rad);

  hogCPU();

  RobotX_m += x_m;
  RobotY_m += y_m;

  updated = true;

  releaseCPU();
}

// turn - relative angle (degrees) from current direction
void turn(float angle_deg)
{
  float commandAngle_deg = angle_deg * TRACK_WHEEL_RATIO;

  // Hog the CPU while setting all the positions
  // to ensure they change atomically even for the high priority
  // task
  hogCPU();

  updated = false;
  moving = true;

  // When turning we set right motors positive and left motor negative
  float finalAngle_deg = getPosition(&driveMotors[DMI_FRONT_RIGHT]) + commandAngle_deg;
  setPosition(&driveMotors[DMI_FRONT_RIGHT], finalAngle_deg);

  finalAngle_deg = getPosition(&driveMotors[DMI_BACK_RIGHT]) + commandAngle_deg;
  setPosition(&driveMotors[DMI_BACK_RIGHT], finalAngle_deg);

  finalAngle_deg = getPosition(&driveMotors[DMI_FRONT_LEFT]) - commandAngle_deg;
  setPosition(&driveMotors[DMI_FRONT_LEFT], finalAngle_deg);

  finalAngle_deg = getPosition(&driveMotors[DMI_BACK_LEFT]) - commandAngle_deg;
  setPosition(&driveMotors[DMI_BACK_LEFT], finalAngle_deg);

  releaseCPU();

  // Wait for angle to be reached
  // Two options
  //  1) Busy-wait by sleeping/polling
  //  2) Use a mutex or other notification
  //
  // For simplicity we will simply busy-wait for all of the motors to
  // reach their respective commands (to within a tolerance)
  // This wait is indefinitie, for now no timeout (could add later)
  // The point is that we will give up our timeslice as efficiently
  // as possible since we are not doing anything useful.
  //
  // There is one advantage to the polling; it allows us to monitor
  // the progress and use it to keep track of the vector, so far.
  // If we were to be disabled (task stops) we would have a pretty
  // good idea where we were at the moment we stopped... not perfect
  // but pretty good
  float angleTraveled_deg = 0.0;
  while ( ! isDriveStopped())
  {
    // Get the remaining distance to compute how far we moved so far
    // Keep doing this
    angleTraveled_deg = angle_deg - getRemainingAngle_deg();

    // TODO: We can add an abort here if we want
    // Just need to stop motors at current angle
    // and then recompute the distance traveled and break

    EndTimeSlice();
  }

  // When the motion is complete we can update the heading
  //
  // We will assume that the heading error caused by the asymmetric encoding
  // is negligible at the moment (i.e., we used a average via the getRemainingAngle_deg
  // function, above)
  //
  // To improve accuracy later we would use other sensors to measure location
  // and correct our internal variables.

  moving = false;

  hogCPU();

  RobotHeading_deg += angleTraveled_deg;

  updated = true;

  releaseCPU();
}

  hogCPU();
  float angle_deg = getPosition(&driveMotors[DMI_FRONT_RIGHT]) + delta_wheel_deg;
  setPosition(&driveMotors[DMI_FRONT_RIGHT], angle_deg);
  setPosition(&driveMotors[DMI_BACK_RIGHT], angle_deg);

  angle_deg = getPosition(&driveMotors[DMI_FRONT_LEFT]) - delta_wheel_deg;
  setPosition(&driveMotors[DMI_FRONT_LEFT], angle_deg);
  setPosition(&driveMotors[DMI_BACK_LEFT], angle_deg);
  releaseCPU();


  while (!isDriveStopped()) {
    EndTimeSlice();
  }
}

// Turn to a specific heading (0 is toward goal)
void turnTo(float deg) {
  float delta_angle = deg - RobotOrientation_rad;
  turn(delta_angle);
}

void moveTo(float x, float y)
{
  // Compute vector from here to there

  // Compute angle between current orientation and desired vector

  // Turn the required angle or turn to the required heading

  // Move the distance equal to the vector magnitude
}

// drive position control is only used during autonomous and
// other similar demos to process command sequences maintaining the
// heading and field position

bool drivePositionControlInitialized = false;
const long DRIVE_POSITION_CONTROL_PERIOD_MSEC = 20;

task drivePositionControl()
{
  // TODO: Insert code similar to drive control but this time
  // we will keep track of (x,y) and heading, stopping only
  // when both are achieved
  if ( ! drivePositionControlInitialized)
  {
    resetDrivePosition();
    drivePositionControlInitialized = true;
  }

  for EVER
  {
    maintainDrivePosition();

#ifdef TEST_SIM
    // only display in emulator
    displayLCDNumber(1, 2, (driveCount++)%100, 3);
#endif

    wait1Msec(DRIVE_POSITION_CONTROL_PERIOD_MSEC);  // Let lower priority tasks execute before resuming control
  }
}

#endif
