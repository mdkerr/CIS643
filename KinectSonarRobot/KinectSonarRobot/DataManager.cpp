//Michael Kerr

#include "testing.h"
#include "DataManager.h"

DataManager::DataManager()
{
	if( TESTING ) cout << "DataManager contructor" << endl;
}

//connects to kinect and saves robot controller reference for later use
boolean DataManager::init(RobotController* rc)
{
	if( TESTING ) cout << "Beginning data manager initialization" << endl;
	this->rc = rc;

	while(!ks.Connect())
	{
		if( TESTING ) cout << "KINECT FAILED TO CONNECT - RETRYING IN 1 SECOND" << endl;
		Sleep(1000);
	}

	if( TESTING ) cout << "Kinect connected!" << endl;

	return true;
}

//detect obstacles in front using sonar and a single center point in the kinect image
boolean DataManager::detectSimple()
{
	if( TESTING ) cout << "Starting simple detection" << endl;
	boolean sonarObstructed, kinectObstructed;

	//call sonar/kinect functions to detect
	sonarObstructed = rc->sonarDetectFront();
	ks.CenterDetect(&kinectObstructed);

	if(sonarObstructed)
	{
		if( TESTING ) cout << "Sonar detected obstacle" << endl;
	}

	if(kinectObstructed)
	{
		if( TESTING ) cout << "Kinect detected obstacle" << endl;
	}

	return (sonarObstructed || kinectObstructed);
}

//detect obstacles in front using sonar and wide kinect image analysis
boolean DataManager::detectCloseFront()
{
	if( TESTING ) cout << "starting detect close front" << endl;

	boolean sonarObstructed, kinectObstructed;

	//check Kinect
	float		detectionAreaWidth = 0.2f;
	float		detectionAreaDepth = 0.6f;
	Vector4*	kinectPoints;
	int			kinectPointsSize;
	
	//check kinect
	if( TESTING ) cout << "Checking kinect" << endl;
	boolean kinectValid = ks.GetObstacleData(&kinectPoints, &kinectPointsSize);
	kinectObstructed = FALSE;

	//ensure kinect data is valid
	if(kinectValid)
	{
		if( TESTING ) cout << "Got kinect data, analyzing image" << endl;
		
		//analyze each point in the horizon line of kinect image
		int numPointsDetected = 0;
		for(int i = 0; i < kinectPointsSize; i++)
		{
			if( TESTING ) cout << "Analyzing point in kinect image" << endl;
			
			//get point from kinect image
			Vector4 point = kinectPoints[i];
			
			//check if point is obstructed
			if(point.z != 0 && (abs(point.x) <= detectionAreaWidth) && (point.z <= detectionAreaDepth))
			{
				//add 1 to points detected
				if( TESTING ) cout << "Point in kinect image was obstructed" << endl;
				numPointsDetected++;
			}

			//check if a threshold amount of points were obstructed
			if(numPointsDetected > 3)
			{
				if( TESTING ) cout << "Kinect detected an obstruction" << endl;
				kinectObstructed = TRUE;
				break;
			}
		}
		if( TESTING ) cout << "Finished kinect analysis" << endl;
	}
	else
	{
		//leave kinectObstructed as FALSE
		if( TESTING ) cout << "Kinect detection encountered an error." << endl;
	}

	//check Sonar
	if( TESTING ) cout << "Checking sonar" << endl;
	sonarObstructed = rc->sonarDetectFront();

	if(kinectObstructed)
		if( TESTING ) cout << "KINECT DETECTED" << endl;

	if(sonarObstructed)
		if( TESTING ) cout << "SONAR DETECTED" << endl;

	//return true if either of sonar or kinect were obstructed
	if( TESTING ) cout << "detect close front finished" << endl;
	return sonarObstructed || kinectObstructed;
}

//use sonar to detect an obstacle to the side of the robot
boolean DataManager::detectSide(boolean right)
{
	if( TESTING ) cout << "Checking if side is obstructed using sonar" << endl;

	//make 2 calls to account for errors
	boolean first = rc->sonarDetectSide(right);
	//Sleep(310);
	boolean second = rc->sonarDetectSide(right);
	
	//log when there is an inconsistency in 2 sequential sonar detections
	if(first != second)
		if( TESTING ) cout << "===================SONAR MISMATCH==================" << endl;

	if( TESTING ) cout << "detect side finished" << endl;
	//return true if both detections showed an obstruction
	return first && second;	
}