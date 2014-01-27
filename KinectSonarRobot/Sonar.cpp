//Michael Kerr

#include "Sonar.h"

Sonar::Sonar()
{
    this->avoidDistance = 250;
    this->avoidAngle = 15;
}

Sonar::Sonar(int avoidDistance, int avoidAngle)
{
    this->avoidDistance = avoidDistance;
    this->avoidAngle = avoidAngle;
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
returns false if an object is detected within that range
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
	
	numSonar = robot->getNumSonar();
	unsigned int* distances = new unsigned int[numSonar];

	for (i = 0; i < numSonar; i++)
	{
		distances[i] = robot->getSonarReading(i)->getRange();
	}

	return distances;
}