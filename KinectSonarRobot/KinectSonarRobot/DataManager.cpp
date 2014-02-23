//Michael Kerr

#include "DataManager.h"

DataManager::DataManager()
{

}

boolean DataManager::init(RobotController* rc)
{
	this->rc = rc;

	while(!ks.Connect())
	{
		cout << "KINECT FAILED TO CONNECT - RETRYING IN 1 SECOND" << endl;
		Sleep(1000);
	}

	return true;
}

boolean DataManager::detectSimple()
{
	boolean sonarObstructed, kinectObstructed;

	sonarObstructed = rc->sonarDetectFront();
	ks.CenterDetect(&kinectObstructed);

	if(sonarObstructed)
	{
		cout << "Sonar detected obstacle" << endl;
	}

	if(kinectObstructed)
	{
		cout << "Kinect detected obstacle" << endl;
	}

	return (sonarObstructed || kinectObstructed);
}