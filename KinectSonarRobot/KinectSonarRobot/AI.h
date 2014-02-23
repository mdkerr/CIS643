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
	private:
		RobotController rc;
		DataManager		dm;
};