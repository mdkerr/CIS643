//Michael Kerr

#include "AI.h"

AI::AI(int argc, char** argv)
{
	rc.init(argc, argv);
	dm.init(&rc);
}

void AI::start()
{
	boolean obstruction;
	double angle;
	maneuver = false;
	
	while(true)
	{
		if(rc.currentHeading == 0) maneuver = false;

		if(maneuver)
		{
			//we need to go back to the left
			if(rc.currentHeading > 0) angle = -90;
			//we need to go back to the right
			else angle = 90;
				
			if(!rc.sonarDetectAngle(angle))
			{
				rc.moveForward(50);
				rc.moveTurn(angle);
				continue;
			}
		}

		//detect
		obstruction = dm.detectCloseFront();

		//no detection
		if(!obstruction)
		{
			rc.moveForward(50);
		}
		//detection
		else
		{
			double angle;
			maneuver = true;

			//we need to go left
			if(rc.currentHeading > 0) angle = -90;
			//we need to go right
			else angle = 90;

			//try going the correct way first
			if(!rc.sonarDetectAngle(angle))
			{
				rc.moveTurn(angle);
			}
			//if its not clear go the other way
			else
			{
				rc.moveTurn(-angle);
			}
		}
		
	}
  
	// Block execution of the main thread here and wait for the robot's task loop
	// thread to exit (e.g. by robot disconnecting, escape key pressed, or OS
	// signal)
	//robot.waitForRunExit();

	//Aria::exit(0);
	//return 0;
}