//Michael Kerr

#include "RobotController.h"
/*
Creates an instance of RobotController
*/
RobotController::RobotController()
{
    this->sonarAvoidDistance = 375;
    this->sonarAvoidAngle = 10;
	this->currentHeading = 0;
}

/*
Initializes the robot, and the sonar on the Robot
*/
boolean RobotController::init(int argc, char** argv)
{
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
		// Error connecting:
		// if the user gave the -help argumentp, then just print out what happened,
		// and continue so options can be displayed later.
		if (!parser.checkHelpAndWarnUnparsed())
		{
			ArLog::log(ArLog::Terse, "Could not connect to robot, will not have parameter file so options displayed later may not include everything");
		}
		// otherwise abort
		else
		{
			ArLog::log(ArLog::Terse, "Error, could not connect to robot.");
			Aria::logOptions();
			Aria::exit(1);
		}
	}

	if(!robot.isConnected())
	{
		ArLog::log(ArLog::Terse, "Internal error: robot connector succeeded but ArRobot::isConnected() is false!");
	}

	// Parse the command line options. Fail and print the help message if the parsing fails
	// or if the help was requested with the -help option
	if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
	{    
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
	if(robot.isConnected())
	{
		robot.move(distance);
		return true;
	}

	return false;
}

/*
Turns the robot by the specified angle (+90 is right, -90 is left) with respect to the robots current direction
returns TRUE if successful
*/
boolean RobotController::moveTurn(double angle)
{
	//make sure robot is connected and it is not currently changing heading
	if(robot.isConnected() && robot.isHeadingDone())
	{
		cout << "TURN STARTED" << endl;

		currentHeading += angle;
		robot.setHeading(currentHeading);

		while( !robot.isHeadingDone() )
		{
			Sleep(10);
		}

		cout << "TURN ENDED" << endl;
		return true;
		
	}

	return false;
}

/*
Uses the sonar to check an arc from -avoidAngle to avoidAngle directly infront of the robot
returns TRUE if an object has been detected
*/
boolean RobotController::sonarDetectFront()
{
    //gets the closest detected object range in front of the robot
    int range = robot.getClosestSonarRange(-sonarAvoidAngle, sonarAvoidAngle);
    
	//an object has been detected
    if ((range > 0) && (range < sonarAvoidDistance)) return true;
    //an object has not been detected
    return false;
}


/*
Creates an array of distances, one for each sonar value that was read
*/
unsigned int* RobotController::sonarDetect()
{
	int i, numSonar;
	
	//get the number of sonar devices on the robot
	numSonar = robot.getNumSonar();
	unsigned int* distances = new unsigned int[numSonar];

	//get distances from each sonar device
	for (i = 0; i < numSonar; i++)
	{
		distances[i] = robot.getSonarReading(i)->getRange();
	}

	return distances;
}