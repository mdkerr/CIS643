#include "Aria.h"
#include "Sonar.h"
#include "KinectSensor.h"
#include <iostream>
using namespace std;

int main(int argc, char** argv)
{
	// Initialize some global data
	Aria::init();

	// This object parses program options from the command line
	ArArgumentParser parser(&argc, argv);

	// Load some default values for command line arguments from /etc/Aria.args
	// (Linux) or the ARIAARGS environment variable.
	parser.loadDefaultArguments();

	// Central object that is an interface to the robot and its integrated
	// devices, and which manages control of the robot by the rest of the program.
	ArRobot robot;

	// Object that connects to the robot or simulator using program options
	ArRobotConnector robotConnector(&parser, &robot);

	// Connect to the robot, get some initial data from it such as type and name,
	// and then load parameter files for this robot.
	if (!robotConnector.connectRobot())
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
		return 1;
	}

	// Start the robot task loop running in a new background thread. The 'true' argument means if it loses
	// connection the task loop stops and the thread exits.
	robot.runAsync(true);
  
	// Sleep for a second so some messages from the initial responses
	// from robots and cameras and such can catch up
	ArUtil::sleep(1000);

	// Used to access sonar range data
	Sonar sonar;
	sonar.init(&robot);

	// turn on the motors
	robot.comInt(ArCommands::ENABLE, 1);

	KinectSensor kinect;
	while(!kinect.Connect())
	{
		cout << "KINECT FAILED TO CONNECT - RETRYING IN 1 SECOND" << endl;
		Sleep(1000);
	}

	//temporary
	while(true)
	{
		boolean sonarClear = sonar.checkFront();

		boolean kinectObstructed;
		if(!kinect.CenterDetect(&kinectObstructed))
		{
			cout << "======================" << endl;
			cout << "KINECT BROKE" << endl;
			cout << "======================" << endl;
		}

		if(!sonarClear)
		{
			cout << "Sonar detected obstacle" << endl;
		}

		if(kinectObstructed)
		{
			cout << "Kinect detected obstacle" << endl;
		}

		if ( sonarClear && !kinectObstructed )
		{
		robot.move(10);
		}
	}
  
	// Block execution of the main thread here and wait for the robot's task loop
	// thread to exit (e.g. by robot disconnecting, escape key pressed, or OS
	// signal)
	robot.waitForRunExit();

	Aria::exit(0);
	return 0;

}

