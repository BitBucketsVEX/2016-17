#pragma config(Sensor, dgtl1,  armEncoder,     sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  leftEncoder,    sensorQuadEncoder)
#pragma config(Sensor, dgtl5,  rightEncoder,   sensorQuadEncoder)
#pragma config(Sensor, dgtl7,  handEncoder,    sensorQuadEncoder)
#pragma config(Motor,  port2,           frontRight,    tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port3,           frontLeft,     tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port4,           handRight,     tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           handLeft,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           topLeft,       tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           topRight,      tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port8,           bottomLeft,    tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           bottomRight,   tmotorVex393_MC29, openLoop, reversed)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#pragma platform(VEX)

//Competition Control and Duration Settings
#pragma competitionControl(Competition)
#pragma autonomousDuration(20)
#pragma userControlDuration(240)


// Comment/uncomment this TEST_SIM flag to switch between real and emulation
//#define TEST_SIM

#ifndef TEST_SIM
	#include "Vex_Competition_Includes.c"
#else
  bool bStopTasksBetweenModes = true;
#endif

/*
     ___          __        ______   .__   __.   _______    .___________. __  .___  ___.  _______         ___       _______   ______
    /   \        |  |      /  __  \  |  \ |  |  /  _____|   |           ||  | |   \/   | |   ____|       /   \     /  _____| /  __  \
   /  ^  \       |  |     |  |  |  | |   \|  | |  |  __     `---|  |----`|  | |  \  /  | |  |__         /  ^  \   |  |  __  |  |  |  |
  /  /_\  \      |  |     |  |  |  | |  . `  | |  | |_ |        |  |     |  | |  |\/|  | |   __|       /  /_\  \  |  | |_ | |  |  |  |
 /  _____  \     |  `----.|  `--'  | |  |\   | |  |__| |        |  |     |  | |  |  |  | |  |____     /  _____  \ |  |__| | |  `--'  |  __ __ __
/__/     \__\    |_______| \______/  |__| \__|  \______|        |__|     |__| |__|  |__| |_______|   /__/     \__\ \______|  \______/  (__|__|__)
 __  .__   __.         ___           _______      ___       __          ___      ___   ___ ____    ____     _______    ___      .______
|  | |  \ |  |        /   \         /  _____|    /   \     |  |        /   \     \  \ /  / \   \  /   /    |   ____|  /   \     |   _  \
|  | |   \|  |       /  ^  \       |  |  __     /  ^  \    |  |       /  ^  \     \  V  /   \   \/   /     |  |__    /  ^  \    |  |_)  |
|  | |  . `  |      /  /_\  \      |  | |_ |   /  /_\  \   |  |      /  /_\  \     >   <     \_    _/      |   __|  /  /_\  \   |      /
|  | |  |\   |     /  _____  \     |  |__| |  /  _____  \  |  `----./  _____  \   /  .  \      |  |        |  |    /  _____  \  |  |\  \----.__
|__| |__| \__|    /__/     \__\     \______| /__/     \__\ |_______/__/     \__\ /__/ \__\     |__|        |__|   /__/     \__\ | _| `._____(_ )
                                                                                                                                             |/
 _______    ___      .______                 ___   ____    __    ____  ___   ____    ____
|   ____|  /   \     |   _  \               /   \  \   \  /  \  /   / /   \  \   \  /   /
|  |__    /  ^  \    |  |_)  |             /  ^  \  \   \/    \/   / /  ^  \  \   \/   /
|   __|  /  /_\  \   |      /             /  /_\  \  \            / /  /_\  \  \_    _/
|  |    /  _____  \  |  |\  \----.__     /  _____  \  \    /\    / /  _____  \   |  |  __ __ __
|__|   /__/     \__\ | _| `._____(_ )   /__/     \__\  \__/  \__/ /__/     \__\  |__| (__|__|__)
                                  |/
     _______.___________.    ___      .______              _______.___________..______       __    __    ______  __  ___
    /       |           |   /   \     |   _  \            /       |           ||   _  \     |  |  |  |  /      ||  |/  /
   |   (----`---|  |----`  /  ^  \    |  |_)  |          |   (----`---|  |----`|  |_)  |    |  |  |  | |  ,----'|  '  /
    \   \       |  |      /  /_\  \   |      /            \   \       |  |     |      /     |  |  |  | |  |     |    <
.----)   |      |  |     /  _____  \  |  |\  \----.   .----)   |      |  |     |  |\  \----.|  `--'  | |  `----.|  .  \
|_______/       |__|    /__/     \__\ | _| `._____|   |_______/       |__|     | _| `._____| \______/   \______||__|\__\
                _______ .______    __       _______.  ______    _______   _______     __   __
               |   ____||   _  \  |  |     /       | /  __  \  |       \ |   ____|   |  | |  |  _
               |  |__   |  |_)  | |  |    |   (----`|  |  |  | |  .--.  ||  |__      |  | |  | (_)
               |   __|  |   ___/  |  |     \   \    |  |  |  | |  |  |  ||   __|     |  | |  |
               |  |____ |  |      |  | .----)   |   |  `--'  | |  '--'  ||  |____    |  | |  |  _
               |_______|| _|      |__| |_______/     \______/  |_______/ |_______|   |__| |__| (_)
      .______    __  .___________.   .______    __    __    ______  __  ___  _______ .___________.    _______.
      |   _  \  |  | |           |   |   _  \  |  |  |  |  /      ||  |/  / |   ____||           |   /       |
      |  |_)  | |  | `---|  |----`   |  |_)  | |  |  |  | |  ,----'|  '  /  |  |__   `---|  |----`  |   (----`
      |   _  <  |  |     |  |        |   _  <  |  |  |  | |  |     |    <   |   __|      |  |        \   \
      |  |_)  | |  |     |  |        |  |_)  | |  `--'  | |  `----.|  .  \  |  |____     |  |    .----)   |
      |______/  |__|     |__|        |______/   \______/   \______||__|\__\ |_______|    |__|    |_______/
     _______.___________..______       __   __  ___  _______     _______.   .______        ___       ______  __  ___
    /       |           ||   _  \     |  | |  |/  / |   ____|   /       |   |   _  \      /   \     /      ||  |/  /
   |   (----`---|  |----`|  |_)  |    |  | |  '  /  |  |__     |   (----`   |  |_)  |    /  ^  \   |  ,----'|  '  /
    \   \       |  |     |      /     |  | |    <   |   __|     \   \       |   _  <    /  /_\  \  |  |     |    <
.----)   |      |  |     |  |\  \----.|  | |  .  \  |  |____.----)   |      |  |_)  |  /  _____  \ |  `----.|  .  \
|_______/       |__|     | _| `._____||__| |__|\__\ |_______|_______/       |______/  /__/     \__\ \______||__|\__\
     _______.___________.    ___      .______              _______.___________..______       __    __    ______  __  ___
    /       |           |   /   \     |   _  \            /       |           ||   _  \     |  |  |  |  /      ||  |/  /
   |   (----`---|  |----`  /  ^  \    |  |_)  |          |   (----`---|  |----`|  |_)  |    |  |  |  | |  ,----'|  '  /
    \   \       |  |      /  /_\  \   |      /            \   \       |  |     |      /     |  |  |  | |  |     |    <
.----)   |      |  |     /  _____  \  |  |\  \----.   .----)   |      |  |     |  |\  \----.|  `--'  | |  `----.|  .  \
|_______/       |__|    /__/     \__\ | _| `._____|   |_______/       |__|     | _| `._____| \______/   \______||__|\__\
                         ___     ___    __     __                  ___     ___    __   ______
                        |__ \   / _ \  /_ |   / /                 |__ \   / _ \  /_ | |____  |
                           ) | | | | |  | |  / /_       ______       ) | | | | |  | |     / /
                          / /  | | | |  | | | '_ \     |______|     / /  | | | |  | |    / /
                         / /_  | |_| |  | | | (_) |                / /_  | |_| |  | |   / /
                        |____|  \___/   |_|  \___/                |____|  \___/   |_|  /_/
*/

#include "convenientMacros.h"
#include "motorControlTypes.h"
#include "armControl.h"
#include "handControl.h"
#include "driveControl.h"


////////////////////////////////////////////////////////////////////////////////////////
//                                                                                    //
//                          Pre-Autonomous Functions                                  //
//                                                                                    //
// You may want to perform some actions before the competition starts. Do them in the //
// following function.                                                                //
//                                                                                    //
////////////////////////////////////////////////////////////////////////////////////////

void pre_auton()
{
	bStopTasksBetweenModes = true;

	constructArmMotorControls();
	constructHandMotorControls();
	constructDriveMotorControls();
}

/////////////////////////////////////////////////////////////////////////////////////////
//
//                                 Autonomous Task
//
// This task is used to control your robot during the autonomous phase of a VEX Competition.
// You must modify the code to add your own robot specific commands here.
//
/////////////////////////////////////////////////////////////////////////////////////////

bool autonomousComplete = false;

bool holdHandRelative = false;		// When true, hand stays relative to ground regardless of arm position
float armAngleStart_deg = 0.0;
float handAngleStart_deg = 0.0;


task autonomous()
{
	autonomousComplete = false;

	// Assume arm control and drive controls have been constructed

	// Place the motor control loops at higher priority than the main loop
	// so we can ensure more stable application of time. The motor control
	// loop must sleep long enough for other tasks to get execution time
	short controlPriority = getTaskPriority(autonomous) + 1;

	// Need arm control during autonomous mode
	startTask(armControl, controlPriority);
	startTask(handControl, controlPriority);

	// Need drive position control during autonomous mode
	startTask(drivePositionControl, controlPriority);

	// Move forward most of the way to the fence
	move(-44.0 * IN_2_M);

	wait1Msec(1000);

	// Maintain the relative orientation of the hand
	// Command only once until reset
	holdHandRelative = true;
	enableHandStabilization();

	setArmPosition(135.0);

	wait1Msec(3000);


	setArmPosition(0.0);

	wait1Msec(1000);

	holdHandRelative = false;
	disableHandStabilization();
	setHandPosition(0.0);

	wait1Msec(1000);

	move(12.0 * IN_2_M);

	wait1Msec(1000);

	turn(180.0);

	wait1Msec(1000);

	move(12.0 * IN_2_M);

	wait1Msec(1000);

	//// Maintain the relative orientation of the hand
	//// Command only once until reset
	//holdHandRelative = true;
	//enableHandStabilization();

	//setArmPosition(135.0);

	//wait1Msec(1000);

	autonomousComplete = true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                             //
//                                 User Control Task                                           //
//                                                                                             //
// This task is used to control your robot during the user control phase of a VEX Competition. //
// You must modify the code to add your own robot specific commands here.                      //
//                                                                                             //
/////////////////////////////////////////////////////////////////////////////////////////////////

int frontback = 1;
float driveFactor = 1.0;
bool manualControl = false;
bool climb = false;
bool climbing = false;
bool abort = false;

bool lifting = false;
bool dumping = false;
bool resetDump = false;

const TVexJoysticks HAND_CONTROL  = Ch1;
const TVexJoysticks DRIVE_CONTROL = Ch3;
const TVexJoysticks TURN_CONTROL  = Ch4;

const int MAX_JOYSTICK_COMMAND = 127;

const TVexJoysticks ALL_DOWN = Btn6D;
const TVexJoysticks HAND_UP  = Btn6U;
const TVexJoysticks LIFT     = Btn5U;
const TVexJoysticks DUMP     = Btn5D;

const TVexJoysticks BACK_IS_FRONT  = Btn7D;
const TVexJoysticks FRONT_IS_FRONT = Btn7U;

const TVexJoysticks CLIMB_A = Btn7L;
const TVexJoysticks CLIMB_B = Btn8R;	// Reused

const TVexJoysticks RESET_ARM_HAND = Btn8D;
const TVexJoysticks ENABLE_HAND_CONTROL = Btn8L;
const TVexJoysticks DISABLE_HAND_CONTROL = Btn8R;

task usercontrol()
{
    // Uncomment for auto test
		//startTask(autonomous);
		//EndTimeSlice();
		//while (! autonomousComplete)
		//{
		//	EndTimeSlice();
		//}
		//stopTask(drivePositionControl);
		//stopTask(autonomous);


	// If (for some reason during testing and such) the autonomous task and subtasks
  // are still running we will need to stop them before continuing
  stopTask(armControl);
  stopTask(handControl);
  stopTask(drivePositionControl);
  stopTask(autonomous);

	// Assume arm controls and drive controls have been constructed

	// Place the motor control loop at higher priority than the main loop
	// so we can ensure more stable application of time. The motor control
	// loop must sleep long enough for other tasks to get execution time
	short controlPriority = getTaskPriority(usercontrol) + 1;
	startTask(armControl, controlPriority);
	startTask(handControl, controlPriority);

	// Only need drive speed control in user control mode
	startTask(driveSpeedControl);

	for EVER
	{

		// Create two-button latching climb command
		// Once triggered there is no going back
		if ((vexRT[CLIMB_A] == 1) &&
			  (vexRT[CLIMB_B] == 1))
		{
			climb = true;
		}
		else
		{
			if (climb == true)
			{
				// Abort and stop
				climb = false;
				climbing = false;
				abort = true;
				setArmPosition(getArmPosition());
				setHandPosition(getHandPosition());
			}

		}

		// Create toggle to switch front and back
	  if (vexRT[BACK_IS_FRONT] == 1)
	  {
	  	frontback = -1;
	  }
	  else if (vexRT[FRONT_IS_FRONT] == 1)
	  {
	  	frontback = 1;
	  }

	  // Create toggle to enable/disable manual arm/hand control
	  if (vexRT[ENABLE_HAND_CONTROL] == 1)
	  {
	  	manualControl = true;
			holdHandRelative = true;
			enableHandStabilization();
			driveFactor = 0.5;
	  }
	  else if (vexRT[DISABLE_HAND_CONTROL] == 1)
	  {
	  	if (climb == false)
	  	{
		  	manualControl = false;
				holdHandRelative = false;
				disableHandStabilization();
				driveFactor = 1.0;
			}
		}

		// Read the joysticks for drive control
	  // passing the latest command for the drive speed task
	  // to pick up on next cycle
		driveSpeed = (int)(driveFactor * (float)(frontback * deadband(vexRT[DRIVE_CONTROL])));
		turnCoef   = (int)(driveFactor * (float)(deadband(vexRT[TURN_CONTROL])));

		// Preset commands
		if (climb && ! abort)
		{
			holdHandRelative = false;		// Just to see it in debug
			disableHandStabilization();
			setHandPosition(getHandPosition() - 45.0);
			if (climbing == false)
			{
				wait1Msec(1000);
				climbing = true;
			}
			setArmPosition(0.0);
		}
		else if (manualControl)
		{
			// Use joystick for relative hand control
			int manualCoef = deadband(vexRT[HAND_CONTROL]);
			setArmSpeed(manualCoef / 2);
		}
		else if (resetDump)
		{
			resetDump = (abs(getHandPosition() - getHandCommand()) > 20.0);
			if (! resetDump)
			{
				 // Finish the reset now that the hand is in position
					setArmPosition(0.0);
					setHandPosition(0.0);
			}
		}
		else
		{
			if (vexRT[RESET_ARM_HAND])
			{
				holdHandRelative = false;
				disableHandStabilization();
				if (dumping == true)
				{
					resetDump = true;
					dumping = false;
					setHandPosition(getHandPosition() - 100.0);
				}
				else if (lifting == true)
				{
					resetDump = true;
					lifting = false;
					setHandPosition(getHandPosition() - 50.0);
				}
			}
			else if (vexRT[ALL_DOWN])
			{
				if (dumping == false)
				{
					holdHandRelative = false;
					disableHandStabilization();
					setArmPosition(0.0);
					setHandPosition(-90.0);
				}
			}
			else if (vexRT[HAND_UP])
			{
				// Ignore command until reset issued
			  // after dumping
			  if (dumping == false)
			  {
					holdHandRelative = false;
					disableHandStabilization();
					setHandPosition(0.0);
				}
			}
			else if (vexRT[LIFT])
			{
				// Ignore command until reset issued
			  // after dumping
			  if (dumping == false)
			  {
			  	lifting = true;
					if (holdHandRelative == false)
					{
						// Maintain the relative orientation of the hand
						// Command only once until reset
						holdHandRelative = true;
						enableHandStabilization();

						setArmPosition(135.0);
					}
				}
			}
			else if (vexRT[DUMP])
			{
				if ((dumping == false)&&(lifting==true))
				{
					dumping = true;
					holdHandRelative = false;
					disableHandStabilization();
					float targetPosition_deg = getHandPosition() + 70.0;		// Based on geometry of fence if hand starts from vertical
					if (targetPosition_deg > 0.0)
					{
						targetPosition_deg = 0.0;		// Don't go past the arm
					}
					setHandPosition(targetPosition_deg);
				}
			}
		} // end if handControl

		EndTimeSlice();
	}
}

// Take over for VEX main task when testing in emulator
#ifdef TEST_SIM

task main() {
	//Setup the VEX LCD for displaying tasks
	clearLCDLine(0);
	clearLCDLine(1);
	displayLCDString(0, 0, "0: xx 1: xx");

	pre_auton();

	startTask(autonomous);

	wait1Msec(1000);
	stopTask(autonomous);

	startTask(usercontrol);

	for EVER
	{
		wait10Msec(100);
	}

}

#endif
