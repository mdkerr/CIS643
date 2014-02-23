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
	//temporary
	while(true)
	{
		obstruction = dm.detectSimple();

		if(!obstruction)
		{
			rc.moveForward(10);
		}
	}
  
	// Block execution of the main thread here and wait for the robot's task loop
	// thread to exit (e.g. by robot disconnecting, escape key pressed, or OS
	// signal)
	//robot.waitForRunExit();

	//Aria::exit(0);
	//return 0;
}