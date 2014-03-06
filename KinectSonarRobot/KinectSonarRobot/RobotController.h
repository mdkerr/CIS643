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
		boolean			sonarDetectAngle(double angle);
		unsigned int*	sonarDetect();

        int				sonarAvoidDistance;
        double			sonarAvoidAngle;
		double			currentHeading;
    
    private:
        ArRobot robot;
		ArRobotConnector* robotConnector;
        ArSonarDevice sonar;
};