#pragma config(Sensor, dgtl1,  blah,           sensorQuadEncoder)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#define EVER (;;)
#define DIM(x) (sizeof(x)/sizeof(x[0]))

// Shaft Encoder Constants
const int SHAFT_DEGREES_PER_TICK = 1.0


task main()
{
	//Setup the VEX LCD for displaying encoder values
	clearLCDLine(0);
	clearLCDLine(1);
	displayLCDString(0, 0, "0: xxxx 1: xxxx");

	SensorValue[blah] = 0;

	for EVER
	{
		long sysTime = nSysTime;

		//Display the control loop values
		displayLCDNumber(0, 3, (int)SensorValue[blah], 4);
	  displayLCDString(1, 0, "Time: ");
	  displayLCDNumber(1, 6, sysTime, 10);
	  EndTimeSlice();		// Allow equal priority tasks to have the processor
	}

}
