#pragma config(Sensor, dgtl1,  armEncoder,     sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  leftEncoder,    sensorQuadEncoder)
#pragma config(Sensor, dgtl5,  rightEncoder,   sensorQuadEncoder)
#pragma config(Motor,  port2,           frontRight,    tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port3,           frontLeft,     tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port4,           backRight,     tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           backLeft,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           topLeft,       tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           topRight,      tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port8,           bottomLeft,    tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           bottomRight,   tmotorVex393_MC29, openLoop, reversed)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#pragma platform(VEX)

//Competition Control and Duration Settings
#pragma competitionControl(Competition)
#pragma autonomousDuration(20)
#pragma userControlDuration(120)


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
#include "driveControl.h"


////////////////////////////////////////////////////////////////////////////////////////
//                                                                                    //
//                          Pre-Autonomous Functions                                  //
//                                                                                    //
// You may want to perform some actions before the competition starts. Do them in the //
// following function.                                                                //
//                                                                                    //
////////////////////////////////////////////////////////////////////////////////////////

void pre_auton() {
	bStopTasksBetweenModes = true;

	constructArmMotorControls();
}

/////////////////////////////////////////////////////////////////////////////////////////
//
//                                 Autonomous Task
//
// This task is used to control your robot during the autonomous phase of a VEX Competition.
// You must modify the code to add your own robot specific commands here.
//
/////////////////////////////////////////////////////////////////////////////////////////

task autonomous() {
	// Assume arm control and drive controls have been constructed

	// Place the motor control loop at higher priority than the main loop
	// so we can ensure more stable application of time. The motor control
	// loop must sleep long enough for other tasks to get execution time
	//short armControlPriority = getTaskPriority(autonomous) + 1;
	//startTask(armControl, armControlPriority);

	//// Need both drive speed and drive position control during autonomous mode
	//startTask(driveSpeedControl);
	//startTask(drivePositionControl);

}

/////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                             //
//                                 User Control Task                                           //
//                                                                                             //
// This task is used to control your robot during the user control phase of a VEX Competition. //
// You must modify the code to add your own robot specific commands here.                      //
//                                                                                             //
/////////////////////////////////////////////////////////////////////////////////////////////////

bool armUp = false;
bool partialArmUp = false;
int frontback = 1;
TVexJoysticks turnControl = Ch4;

task usercontrol() {

	// Assume arm controls and drive controls have been constructed

	// Place the motor control loop at higher priority than the main loop
	// so we can ensure more stable application of time. The motor control
	// loop must sleep long enough for other tasks to get execution time
	short armControlPriority = getTaskPriority(usercontrol) + 1;
	startTask(armControl, armControlPriority);

	// Only need drive speed control in user control mode
	startTask(driveSpeedControl);

	for EVER
	{
		// Create toggle to switch front and back
	  if (vexRT[Btn8D] == 1)
	  {
	  	frontback = -1;
	  }
	  else if (vexRT[Btn8U] == 1)
	  {
	  	frontback = 1;
	  }

	  if (vexRT[Btn8R] == 1)
	  {
	  	turnControl = Ch1;
	  }
	  else if (vexRT[Btn8L] == 1)
	  {
	  	turnControl = Ch4;
	  }

		// Read the joysticks for drive control
	  // passing the latest command for the drive speed task
	  // to pick up on next cycle
		driveSpeed = frontback * deadband(vexRT[Ch3]);
		turnCoef   = deadband(vexRT[turnControl]);

	  //int cmd = abs(deadband(vexRT[Ch1]));
	  //motor[topRight] = MAX(cmd,MAX_MOTOR_COMMAND);
	  //motor[topLeft] = MAX(cmd,MAX_MOTOR_COMMAND);
	  //motor[bottomRight] = MAX(cmd,MAX_MOTOR_COMMAND);
	  //motor[bottomLeft] = MAX(cmd,MAX_MOTOR_COMMAND);

		// The following is VERY EARLY TEST CODE ONLY
		if (vexRT[Btn5U] == 1)
		{
			if ( ! armUp)
			{
				setArmPosition(120.0);
				armUp = true;
			}
		}
		else if (vexRT[Btn5D] == 1)
		{
			if (armUp)
			{
				setArmPosition(0.0);
				armUp = false;
			}
		}

		if (vexRT[Btn6U] == 1)
		{
			if (!partialArmUp)
			{
				setArmPosition(60.0);
				partialArmUp = true;
			}
		} else if (vexRT[Btn6D] == 1)
		{
			if  (partialArmUp)
			{
				setArmPosition(0.0);
				partialArmUp = false;
			}
		}

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

	for EVER {
		wait10Msec(100);
	}

}

#endif
