//Michael Kerr

#include "RobotController.h"
#include "testing.h"
#include <windows.h>
#include <math.h>

/*
Creates an instance of RobotController
*/
RobotController::RobotController()
{
	if( TESTING ) cout << "RobotController constructor" << endl;

	//initialize class variables to 0
	this->currentHeading = 0;
	this->current_x = 0;
	this->current_y = 0;
	this->target_x = 0;
	this->target_y = 0;
}

/*
Initializes the robot, and the sonar on the Robot
*/
boolean RobotController::init(int argc, char** argv)
{
	if( TESTING ) cout << "RobotController initializing" << endl;

	// Initialize some global data
	Aria::init();
	
	// This object parses program options from the command line
	ArArgumentParser parser(&argc, argv);

	// (Linux) or the ARIAARGS environment variable.
	parser.loadDefaultArguments();
	
	// Object that connects to the robot or simulator using program options
	robotConnector = new ArRobotConnector(&parser, &robot);

	// Connect to the robot, get some initial data from it such as type and name,
	// and then load parameter files for this robot.
	if (!robotConnector->connectRobot())
	{
		if( TESTING ) cout << "An error has occured connecting to the robot" << endl;
		// Error connecting:
		// if the user gave the -help argumentp, then just print out what happened,
		// and continue so options can be displayed later.
		if (!parser.checkHelpAndWarnUnparsed())
		{
			if( TESTING ) cout << "Error connecting - log error" << endl;
			ArLog::log(ArLog::Terse, "Could not connect to robot, will not have parameter file so options displayed later may not include everything");
		}
		// otherwise abort
		else
		{
			if( TESTING ) cout << "Error connecting - abort" << endl;
			ArLog::log(ArLog::Terse, "Error, could not connect to robot.");
			Aria::logOptions();
			Aria::exit(1);
		}
	}

	//ensure the robot is connected
	if(!robot.isConnected())
	{
		if( TESTING ) cout << "Internal error: robot connector succeeded but ArRobot::isConnected() is false!" << endl;
		ArLog::log(ArLog::Terse, "Internal error: robot connector succeeded but ArRobot::isConnected() is false!");
	}

	// Parse the command line options. Fail and print the help message if the parsing fails
	// or if the help was requested with the -help option
	if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
	{
		if( TESTING ) cout << "Error parsing command line options" << endl;
		Aria::logOptions();
		Aria::exit(1);
		return false;
	}

	// Start the robot task loop running in a new background thread. The 'true' argument means if it loses
	// connection the task loop stops and the thread exits.
	robot.runAsync(true);
  
	// Sleep for a second so some messages from the initial responses
	// from robots and cameras and such can catch up
	ArUtil::sleep(1000);

	//Initialize sonar
	robot.addRangeDevice(&sonar);
	robot.enableSonar();

	// turn on the motors
	robot.comInt(ArCommands::ENABLE, 1);

	return true;
}

/*
Moves the robot forward by "distance" in milimeters
returns TRUE if successful
*/
boolean RobotController::moveForward(double distance)
{
	if( TESTING ) cout << "MOVE STARTED" << endl;

	//Make sure the robot is connected before issuing a move command
	if(robot.isConnected())
	{
		if( TESTING ) cout << "Robot is connected - move command is valid" << endl;

		//keep track of distance traveled in the x and y directions
		current_x += distance*sin(currentHeading / -57.2957795);
		current_y += distance*cos(currentHeading / -57.2957795);

		//issue the move command
		robot.move(distance);

		//wait until the robot is done moving
		while( !robot.isMoveDone() )
		{
			if( TESTING ) cout << "Waiting for robot to finish move command..." << endl;
			Sleep(10);
		}

		//move was successful, return
		return true;
	}

	if( TESTING ) cout << "Robot not connected - move command invalid" << endl;
	return false;
}

/*
Sets the robots current heading to a heading of theta, returns true is the headding change is successful
*/
boolean RobotController::setHeading(double theta)
{
	if( TESTING ) cout << "SET HEADING STARTED" << endl;

	//make sure robot is connected
	if(robot.isConnected())
	{
		if( TESTING ) cout << "Robot is connected - heading change is valid" << endl;

		//update the current heading of the robot
		currentHeading = theta;
		robot.setHeading(currentHeading);

		//wait for the robot to finish changing directions
		while( !robot.isHeadingDone() )
		{
			if( TESTING ) cout << "Waiting for robot to finish turning.." << endl;
			Sleep(10);
		}

		//heading change successful - return
		return true;
		
	}

	if( TESTING ) cout << "Robot is not connected - heading change invalid" << endl;
	return false;
}

/*
Turns the robot by the specified angle (+90 is right, -90 is left) with respect to the robots current direction
returns TRUE if successful
*/
boolean RobotController::moveTurn(boolean right)
{
	static const double TURN_ERROR_ADJUSTMENT = 0.98;
	double angle;
	if( right ) angle = -90;
	else angle = 90;

	if(right)
		if( TESTING ) cout << "Turning RIGHT" << endl;
	else
		if( TESTING ) cout << "Turning LEFT" << endl;

	//make sure robot is connected
	if(robot.isConnected())
	{
		if( TESTING ) cout << "TURN STARTED" << endl;

		//update the current heading by the calculated amount
		currentHeading += angle * TURN_ERROR_ADJUSTMENT;
		robot.setHeading(currentHeading);

		//wait for the robot to finish turning
		while( !robot.isHeadingDone() )
		{
			if( TESTING ) cout << "Waiting for robot to finish turning.." << endl;
			Sleep(10);
		}

		//turn was successful, return
		return true;
		
	}

	if( TESTING ) cout << "Robot not connected - turn invalid" << endl;
	return false;
}

/*
Uses the sonar to check an arc from -avoidAngle to avoidAngle directly infront of the robot
returns TRUE if an object has been detected
*/
boolean RobotController::sonarDetectFront()
{
	if( TESTING ) cout << "Checking front sonar for obstructions" << endl;

	//get readings from all relevant sonar
	int one   = robot.getSonarReading(1)->getRange();
	int two   = robot.getSonarReading(2)->getRange();
	int three = robot.getSonarReading(3)->getRange();
	int four  = robot.getSonarReading(4)->getRange();
	int five  = robot.getSonarReading(5)->getRange();
	int six   = robot.getSonarReading(6)->getRange();

	//check each sonar to see if an obstacle is too close
	//if( one   < 200 || six  < 200 ){ if( TESTING ) cout << "DETECTED 1-6" << endl; return true; }
	if( two   < 300 || five < 300 ){ if( TESTING ) cout << "DETECTED 2-5" << endl; return true; }
	if( three < 400 || four < 400 ){ if( TESTING ) cout << "DETECTED 3-4" << endl; return true; }

	if( TESTING ) cout << "Nothing detected - path is clear" << endl;
	return false;
}

/*
Check the side of the robot using sonar to see if it is safe to turn that way
*/
boolean RobotController::sonarDetectSide(boolean right)
{
	if( TESTING ) cout << "DETECT SIDE STARTED" << endl;

	//detect right side
	if( right )
	{
		if( TESTING ) cout << "Checking right side" << endl;

		//get all readings from the right side of teh robot
		int seven   = robot.getSonarReading(7)->getRange();
		int eight   = robot.getSonarReading(8)->getRange();
		int nine    = robot.getSonarReading(9)->getRange();

		//see if robot is clear right
		if( seven < 500 || eight < 500 || nine < 200)
		{
			if( TESTING ) cout << "RIGHT DETECTED" << endl;
			return true;
		}
	}
	//detect left side
	else
	{
		if( TESTING ) cout << "Checking left side" << endl;

		//get all readings from the left side of the robot
		int zero    = robot.getSonarReading(0)->getRange();
		int fifteen = robot.getSonarReading(15)->getRange();
		int fourteen = robot.getSonarReading(14)->getRange();

		//see if robot is clear on left
		if( zero < 500 || fifteen < 500 || fourteen < 200)
		{
			if( TESTING ) cout << "LEFT DETECTED" << endl;
			return true;
		}
	}

	if( TESTING ) cout << "Side detection is clear - no obstructions" << endl;
	return false;
}


/*
Creates an array of distances, one for each sonar value that was read
*/
unsigned int* RobotController::sonarDetect()
{
	if( TESTING ) cout << "Get readings from all sonar on the robot" << endl;
	int i, numSonar;
	
	//get the number of sonar devices on the robot
	numSonar = robot.getNumSonar();
	unsigned int* distances = new unsigned int[numSonar];

	//get distances from each sonar device
	for (i = 0; i < numSonar; i++)
	{
		if( TESTING ) cout << "Reading from sonar" << endl;
		//add that distance to the list
		distances[i] = robot.getSonarReading(i)->getRange();
	}

	//return distances
	return distances;
}