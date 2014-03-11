//Michael Kerr

#include "RobotController.h"
#include "KinectSensor.h"
#include <iostream>

using namespace std;

class DataManager
{
	public:
		DataManager();
		boolean init(RobotController*);
		boolean detectSimple();
		boolean detectCloseFront();
	private:
		RobotController* rc;
		KinectSensor	 ks;
};