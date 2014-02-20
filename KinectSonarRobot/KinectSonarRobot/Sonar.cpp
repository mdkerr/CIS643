//Michael Kerr

#include "Sonar.h"
/*
Creates an instance of Sonar with default avoidDistance and avoidAngle
*/
Sonar::Sonar()
{
    this->avoidDistance = 250;
    this->avoidAngle = 15;
}

/*
Initializes sonar on the robot
returns true if robot has not been set
*/
bool Sonar::init(ArRobot* r)
{
	this->robot = r;

    if (robot != NULL)
    {
        //add sonar device to the robot
        robot->addRangeDevice(&sonar);
		robot->enableSonar();
		robot->getNumSonar();
        return true;
    }
    return false;
}

/*
Uses the sonar to check an arc from -avoidAngle to avoidAngle directly infront of the robot
returns FALSE if an object is detected, TRUE otherwise
*/
bool Sonar::checkFront()
{
    //gets the closest detected object range in front of the robot
    int range = robot->getClosestSonarRange(-avoidAngle, avoidAngle);
    //returns false if object is detected within range
    if ((range > 0) && (range < avoidDistance)) return false;
    
    return true;
}

/*
Creates an array of distances, one for each sonar value that was read
*/
unsigned int* Sonar::detect()
{
	int i, numSonar;
	
	//get the number of sonar devices on the robot
	numSonar = robot->getNumSonar();
	unsigned int* distances = new unsigned int[numSonar];

	//get distances from each sonar device
	for (i = 0; i < numSonar; i++)
	{
		distances[i] = robot->getSonarReading(i)->getRange();
	}

	return distances;
}