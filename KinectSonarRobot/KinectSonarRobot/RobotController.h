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
		boolean			setHeading(double);
		boolean			moveForward(double);
		boolean			moveTurn(boolean right);
        boolean			sonarDetectFront();
		boolean			sonarDetectSide(boolean right);
		unsigned int*	sonarDetect();

		double			currentHeading;
		double			target_x;
		double			target_y;
		double			current_x;
		double			current_y;
    
    private:
        ArRobot robot;
		ArRobotConnector* robotConnector;
        ArSonarDevice sonar;
};