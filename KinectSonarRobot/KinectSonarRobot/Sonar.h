//Michael Kerr

#include "Aria.h"
#include "ArRobot.h"
#include "ArActionAvoidFront.h"

class Sonar
{
    public:
        Sonar();
        bool init(ArRobot*);
        bool checkFront();
		unsigned int* detect();

        int avoidDistance;
        int avoidAngle;
    
    private:
        ArRobot* robot;
        ArSonarDevice sonar;
};