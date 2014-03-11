//Samuel Fike
//Chuohao Jiang

#include <iostream>
#include "KinectSensor.h"

using namespace std;

/*
int mainDisabled()
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

	while(true)
	{
		Vector4*	obstacleData = NULL;
		int			obstacleDataSize;

		//function isn't using out values yet, so just use null
		if(kinect.GetObstacleData(obstacleData, &obstacleDataSize))
		{
			for(int i = 0; i < obstacleDataSize; i+=30)
			{
				//cout << (int) obstacleData[i].z << " ";
			}
			//cout << endl;
		}
		else
		{
			cout << "Kinect detection failed!" << endl;
		}
		

		cout << "\nPress enter to run again\n";
		cin.get();
	}
}

*/