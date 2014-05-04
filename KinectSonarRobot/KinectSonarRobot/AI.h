//Michael Kerr

#include <iostream>
#include "DataManager.h"
#include "RobotController.h"

using namespace std;

class AI
{
	public:
		AI(int, char**);
		void start();
		void goAround(boolean right);
		void DestinationCheck();
		void TurnTowardsTarget();
	private:
		RobotController rc;
		DataManager		dm;
		boolean			maneuver;

		double			theta;
};