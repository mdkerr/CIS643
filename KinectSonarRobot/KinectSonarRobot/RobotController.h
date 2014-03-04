//Michael Kerr
#pragma once
#include "Aria.h"
#include "ArRobot.h"
#include "ArActionAvoidFront.h"
#include <iostream>

using namespace std;

class RobotController
{
    public:
        RobotController();

		boolean			init(int argc, char** argv);
		boolean			moveForward(double);
		boolean			moveTurn(double);
        boolean			sonarDetectFront();
		unsigned int*	sonarDetect();

        int				sonarAvoidDistance;
        int				sonarAvoidAngle;
    
    private:
        ArRobot robot;
		ArRobotConnector* robotConnector;
        ArSonarDevice sonar;
		double currentHeading;
};