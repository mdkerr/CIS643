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
	//ks.CenterDetect(&kinectObstructed);

	if(sonarObstructed)
	{
		cout << "Sonar detected obstacle" << endl;
	}

	//if(kinectObstructed)
	//{
	//	cout << "Kinect detected obstacle" << endl;
	//}

	//return (sonarObstructed || kinectObstructed);
	return sonarObstructed;
}

boolean DataManager::detectCloseFront()
{
	boolean sonarObstructed, kinectObstructed;

	//check Kinect
	float		detectionAreaWidth = 0.2f;
	float		detectionAreaDepth = 0.6f;
	Vector4*	kinectPoints;
	int			kinectPointsSize;
	
	boolean kinectValid = ks.GetObstacleData(&kinectPoints, &kinectPointsSize);
	kinectObstructed = FALSE;

	if(kinectValid)
	{
		for(int i = 0; i < kinectPointsSize; i++)
		{
			Vector4 point = kinectPoints[i];
			if(point.z != 0 && (abs(point.x) <= detectionAreaWidth) && (point.z <= detectionAreaDepth))
			{
				kinectObstructed = TRUE;
				break;
			}
		}
	}
	else
	{
		//leave kinectObstructed as FALSE
		cout << "Kinect detection encountered an error." << endl;
	}

	//check Sonar
	sonarObstructed = rc->sonarDetectFront();

	if(kinectObstructed)
		cout << "KINECT DETECT" << endl;
	else
		cout <<"KINECT CLEAR" << endl;

	return sonarObstructed || kinectObstructed;
}