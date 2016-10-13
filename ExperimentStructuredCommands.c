// The example shows how to build up a command processor to make developing autonomous
// sequences a little easier to maintain

// Some convenient macros
#define EVER (;;)
#define DIM(x) (sizeof(x)/sizeof(x[0]))

// Declare an enumeration of meaningful, simple commands
enum Commands
{
	STOP,
	FORWARD,		// Relative position forward (meters)
	BACKWARD,		// Relative position backwards (meters)
	POSITION,		// Absolute field coordinate (meters); (0,0) is back/left corner
	RIGHT,			// Relative heading (degrees)
	LEFT,				// Relative heading (degrees)
	HEADING,		// Absolute heading (degrees); 0 is toward opposing side, +90 is right, -90 is left
	ARM_UP,			// Arm at full up position
	ARN_DOWN,		// Arm at full down position
	ARM_AT,			// Arm at angle relative to level (0 degrees is horizontal hold, < 0 is down toward ground, > 0 is above level)

	BOGUS
};

// Declare a structure to contain command tokens and parameters
// so we can represent a command sequence as a simple array
// of commands
typedef struct
{
	Commands command;
	float    argument[2];		// Two arguments is all we need for the moment
} CommandToken;

// Define a command sequence
//
// NOTE: ROBOTC does not follow the C99 standard, thus does not
// allow struct initializers, for example:
//		struct CommandToken	commandSequence[8] =
//		{
//			// Example of drawing a square
//			{FORWARD, 	1.0, 	0.0},
//			{RIGHT,			90.0, 0.0},
//			{FORWARD,		1.0, 	0.0},
//			{RIGHT,			90.0, 0.0},
//			{FORWARD,		1.0, 	0.0},
//			{RIGHT,			90.0, 0.0},
//			{FORWARD,		1.0, 	0.0},
//			{RIGHT,			90.0, 0.0}
//		};
//
// INSTEAD, we will synthesize the initialization via a function call
// initializing an array of up to some maximum depth (e.g., 256 commands)
// For user-friendliness we will hang with a message on the LCD if the
// maximum number of commands has been exceeded.

/// TODO:

void processCommand(const CommandToken *pCommand)
{
	// Only process known commands
	//if (pCommand->
}

// ----------------------------------------------------------------------------------
// Like some competition environments there is an autonomous and user controlled
// mode driven from some main task
// ----------------------------------------------------------------------------------
task autonomous()
{
	for (unsigned int i = 0; i < DIM(commandSequence); ++i)
	{
		processCommand(&commandSequence[i]);
	}
}

// ----------------------------------------------------------------------------------
// ROBOTC always starts at main
// ----------------------------------------------------------------------------------
task main()
{

	// In this case we just start the autonomous task since we are not creating
  // from a competition template
	startTask(autonomous);


}
