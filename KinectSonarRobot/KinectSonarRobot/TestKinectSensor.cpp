//Samuel Fike
//Chuohao Jiang

#include <iostream>
#include "KinectSensor.h"

using namespace std;


int mainA()
{
	KinectSensor kinect;

	cout << "Attempting Kinect connection" << endl;

	//try connecting
	while(!kinect.Connect()) 
	{
		cout << "Kinect connection failed - Retrying in 1 second" << endl;
		Sleep(1000);
	}
	
	cout << endl << "Kinect connected" << endl;

	boolean obstaclesDetected;

	while(true)
	{
		//test center detect
		if(kinect.CenterDetect(&obstaclesDetected))
		{
			cout << (obstaclesDetected ? "Obstructed" : "Clear") << endl;
		}
		else
		{
			cout << "Center detection failed" << endl;
		}

		/*
		//function isn't using out values yet, so just use null
		if(kinect.GetObstacleData(NULL, NULL))
		{
			//do nothing and let it print debug/testing data from inside function
		}
		else
		{
			cout << "Kinect detection failed" << endl;
		}
		*/

		cout << "\nPress enter to run again\n";
		cin.get();
	}
}

