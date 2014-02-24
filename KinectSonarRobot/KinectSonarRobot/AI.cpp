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
	maneuver = false;
	//temporary
	while(true)
	{
		obstruction = dm.detectSimple();

		if(!obstruction)
		{
			rc.moveForward(75);
		}
		else
		{
			
			if(maneuver)
			{
				rc.moveTurn(90);
			}
			else
			{
				rc.moveTurn(0);
			}
			maneuver = !maneuver;
		}
		
	}
  
	// Block execution of the main thread here and wait for the robot's task loop
	// thread to exit (e.g. by robot disconnecting, escape key pressed, or OS
	// signal)
	//robot.waitForRunExit();

	//Aria::exit(0);
	//return 0;
}