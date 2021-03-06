#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, I2C_1,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_2,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Motor,  port2,           blah,          tmotorVex393TurboSpeed_MC29, PIDControl, encoderPort, I2C_1)
#pragma config(Motor,  port9,           blec,          tmotorVex393TurboSpeed_MC29, PIDControl, encoderPort, I2C_2)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#define EVER (;;)
#define DIM(x) (sizeof(x)/sizeof(x[0]))


//	Quadrature Encoders:	360 ticks per revolution of the output shaft
//	269 Motor IME:	240.4 ticks per revolution of the output shaft
//	393 Motor IME:	627.2 ticks per revolution of the output shaft
//	393 High Speed IME:	392 ticks per revolution of the output shaft
//	393 Turbo Speed IME:	261 ticks per revolution of the output shaft

// 						Output Stage  Output Stage	Output Speed 		Stall Torque 		IME Ticks
// 						Driving Gear	Driven Gear		(RPM)	Output 		(N*m)						per Revolution
//Standard 		10t						32t						100							1.67						627.2
//High Speed	14t						28t						160							1.04						392
//Turbo 			18t						24t						240							0.70						261.333

const float TICKS_PER_REVOLUTION = 627.2; // Standard 393
const float TICKS_PER_DEGREE = TICKS_PER_REVOLUTION / 360.0;
const float DEGREES_PER_TICK = 1.0 / TICKS_PER_DEGREE;

const int MAX_MOTOR_RATE = 127;
const float MAX_RPM = 100.0;							// Standard 393
const float TICKS_PER_RPM = (float)MAX_MOTOR_RATE / MAX_RPM;


// Declare a stuct that represent identification,
// control parameters, and status. Since RobotC
// is C and not C++, we will define separate functions
// rather than encapsulate the desired behavior
typedef struct
{
	tMotor id;
	float command_deg;
	float encoder_deg;
	float encoder_rpm;
	float kp;
	float ki;
	float kd;
	int pid;
} motorControlType;

// A list of the motors we want to control during the motorControl task
struct motorControlType controlledMotors[2];

void constructMotorControlType(unsigned int index, tMotor id, float kp, float ki, float kd)
{
	if (index < DIM(controlledMotors))
	{
		controlledMotors[index].id = id;
		controlledMotors[index].command_deg = 0.0;
		controlledMotors[index].encoder_deg = 0.0;
		controlledMotors[index].encoder_rpm = 0.0;
		controlledMotors[index].kp = kp;
		controlledMotors[index].ki = ki;
		controlledMotors[index].kd = kd;
		controlledMotors[index].pid = 0;
	}
}

void setMotorCommand(const unsigned int index, float aPosition_deg)
{
	if (index < DIM(controlledMotors))
	{
		controlledMotors[index].command_deg = aPosition_deg;
	}
}

float getMotorCommand(const unsigned int index)
{
	float retval = 0.0;

	if (index < DIM(controlledMotors))
	{
		retval = controlledMotors[index].command_deg;
	}

	return retval;
}

void resetControlledMotors(void)
{
	for (unsigned int i = 0; i < DIM(controlledMotors); ++i)
	{
		resetMotorEncoder(controlledMotors[i].id);
	}
}

// A function to actually apply the control to each motor in the control list
//
// TODO: It would be better to slave motors on command and occassionally correct
// for positioning errors
void maintainMotorPosition(void)
{
	for (unsigned int i = 0; i < DIM(controlledMotors); ++i)
	{
		struct motorControlType *pSomeMotorControl = &controlledMotors[i];

		long encoder_tick = getMotorEncoder(controlledMotors[i].id);
		controlledMotors[i].encoder_rpm  = (float)getMotorVelocity(controlledMotors[i].id);


		pSomeMotorControl->encoder_deg = encoder_tick * DEGREES_PER_TICK;
		float error_deg = pSomeMotorControl->command_deg - pSomeMotorControl->encoder_deg;
		if (abs(error_deg) > 0)
		{
			// make the speed proportional to the error
		  int lastPidSign = (pSomeMotorControl->pid != 0)? pSomeMotorControl->pid/abs(pSomeMotorControl->pid) : 1;
		  pSomeMotorControl->pid = (int)(pSomeMotorControl->kp * error_deg) + lastPidSign*(int)(pSomeMotorControl->kd * pSomeMotorControl->encoder_rpm); // truncate to integer
		  if (pSomeMotorControl->pid > MAX_MOTOR_RATE)
		  {
		  	pSomeMotorControl->pid = MAX_MOTOR_RATE;
		  }
		  else if (pSomeMotorControl->pid < -MAX_MOTOR_RATE)
		  {
		  	pSomeMotorControl->pid = -MAX_MOTOR_RATE;
		  }
		  motor[pSomeMotorControl->id] = pSomeMotorControl->pid;
		}
		else
		{
			// When within the target tolerance, stop.
			motor[pSomeMotorControl->id] = 0;
		}

	}
}

task motorControl()
{
	resetControlledMotors();

	for EVER
	{
		maintainMotorPosition();
		sleep(1);	// Let lower priority tasks execute before resuming control
	}
}

task main()
{
	constructMotorControlType(0,blah,0.8,0.0,0.0);
	constructMotorControlType(1,blec,0.8,0.0,0.0);

	wait1Msec(1000);

	// Place the motor control loop at higher priority than the main loop
	// so we can ensure more stable application of time. The motor control
	// loop must sleep long enough for other tasks to get execution time
	short motorControlPriority = getTaskPriority(main) + 1;
	startTask(motorControl, motorControlPriority);

	//Setup the VEX LCD for displaying encoder values
	clearLCDLine(0);
	clearLCDLine(1);
	displayLCDString(0, 0, "0: xxxx 1: xxxx");

	// Ensure that both motor commands change at the same
	// time relative to the control loop; we declare these
	// lines as a "critical section"
	hogCPU();
	setMotorCommand(0, 90.0);
	setMotorCommand(1, 90.0);
	releaseCPU();

	long switchTime = nSysTime + 1000;

	for EVER
	{
		long sysTime = nSysTime;

		if (sysTime > switchTime)
		{
			switchTime += 2000;
			hogCPU();
			setMotorCommand(0,-getMotorCommand(0));
			setMotorCommand(1,-getMotorCommand(1));
			releaseCPU();
		}

		//Display the control loop values
		displayLCDNumber(0, 3, (int)controlledMotors[0].encoder_deg, 4);
		displayLCDNumber(0, 11, (int)controlledMotors[1].encoder_deg, 4);
	  displayLCDString(1, 0, "Time: ");
	  displayLCDNumber(1, 6, sysTime, 10);
	  EndTimeSlice();		// Allow equal priority tasks to have the processor
	}

}
