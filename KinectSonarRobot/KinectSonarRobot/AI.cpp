//Michael Kerr

#include "AI.h"
#include "testing.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

//initializes robot controller and data manager
AI::AI(int argc, char** argv)
{
	if( TESTING ) cout << "Initializing robot controller" << endl;
	rc.init(argc, argv);

	if( TESTING ) cout << "Initializing data manager" << endl;
	dm.init(&rc);
}

//gets user input and starts main control loop
void AI::start()
{
	if( TESTING ) cout << "Getting coordinate inputs from user." << endl;
	char buffer[256];

	//get x coordinate
	printf ("\n\nEnter X coord: ");
	fgets (buffer, 256, stdin);
	rc.target_x = atof(buffer);

	//get y coordinate
	printf ("\n\nEnter Y coord: ");
	fgets (buffer, 256, stdin);
	rc.target_y = atof(buffer);
	
	//turn toward destination
	if( TESTING ) cout << "Turning towards destination" << endl;
	TurnTowardsTarget();

	//main loop, loops until destination is reached
	while(true)
	{
		//checking if destination is reached, will exit program if so
		if( TESTING ) cout << "Checking if destination has been reached" << endl;
		DestinationCheck();

		//manuever towards destination
		if(dm.detectCloseFront())
		{
			//need to go around an obstacle
			if( TESTING ) cout << "Obstacle detected in front" << endl;
			if(!dm.detectSide(false)) 
			{
				//left side clear, go around left
				if( TESTING ) cout << "Left side is clear, going around to the left" << endl;
				goAround(false);
			}
			else 
			{
				//left side obstructed, go around right
				if( TESTING ) cout << "Left side is obstructed, going around to the right" << endl;
				goAround(true);
			}

			//finished going around, turn back toward destination
			if( TESTING ) cout << "Finished manuevering around obstacle, turning toward destination" << endl;
			TurnTowardsTarget();
		}
		else
		{
			//front is clear, move foward
			if( TESTING ) cout << "Front is clear, moving forward" << endl;
			rc.moveForward(100);
		}
	}

	if( TESTING ) cout << "Got outside of main loop" << endl;
}

//turns the robot toward the destination position
void AI::TurnTowardsTarget()
{
	//calculate vector toward destination
	if( TESTING ) cout << "Calculating vector towards destination" << endl;
	double x = rc.target_x - rc.current_x;
	double y = rc.target_y - rc.current_y;
	theta = 0;

	if( y == 0 )
	{
		//y delta is 0, so we are going left or right
		if( TESTING ) cout << "delta y is 0, either right or left turn" << endl;
		if( x > 0 ) 
		{
			//need to go right
			if( TESTING ) cout << "delta x is positive, right turn required" << endl;
			theta = -90;
		}
		else if( x < 0 )
		{
			//need to go left
			if( TESTING ) cout << "delta x is negative, left turn required" << endl;
			theta = 90;
		}
	}
	else if( x == 0 )
	{
		//delta x is 0, need to go forward or backwards
		if( TESTING ) cout << "delta x is 0, no turn or turn around required" << endl;
		if( y < 0 )
		{
			//need to turn around
			if( TESTING ) cout << "delta y is negative, turn around required" << endl;
			theta = -180;
		}
	}
	else
	{
		//delta x and y are non-zero, use trig to calculate angle
		if( TESTING ) cout << "delta x and delta y are both non-zero, must calculate angle" << endl;
		theta = atan( x / y ) * -57.2957795;

		if( y < 0 )
		{
			//add 180 to put theta in correct quadrant
			if( TESTING ) cout << "delta y is negative, must adjust calculated angle for correct quadrant" << endl;
			theta += 180;
		}
	}

	//execute turn with calculated theta
	if( TESTING ) cout << "Turning robot toward destination" << endl;
	rc.setHeading(theta);
}

//check if robot has reached its destination, if so, exit the program
void AI::DestinationCheck()
{
	//check if destination has been reached
	if( TESTING ) cout << "Beginning destination check" << endl;
	if( abs(rc.target_x - rc.current_x ) < 50 && abs(rc.target_y - rc.current_y) < 50)
	{
		//Destination reached, end program
		if( TESTING ) cout << "Destination reached, ending program" << endl;
		std::exit(0);
	}

	if( TESTING ) cout << "Destination has not been reached" << endl;
}

//turns perpendicular to obstacle, drives until the obstacle is no longer in the way, then turns back
void AI::goAround(boolean right)
{
	if( TESTING ) cout << "Beginning go around instruction" << endl;
	
	//turn toward the direction passed in
	rc.moveTurn(right);

	//drive until obstacle no longer in the way
	int i = 0;
	while(true)
	{
		if( TESTING ) cout << "Driving until obstacle no longer in the way" << endl;
		i++;

		//check if destination was reached
		DestinationCheck();

		//check if obstacle in front
		if(dm.detectCloseFront())
		{
			//check what side to try and go around
			cout << "Obstacle detected in front, going around" << endl;
			if(!dm.detectSide(!right))
			{
				//left clear, go left
				cout << "Left side clear, going around left" << endl;
				goAround(!right);
			}
			else
			{
				//left blocked, go right
				cout << "Left side obstructed, going around right" << endl;
				goAround(right);
			}

			//reset i so that we don't check for the side being clear before we move a decent distance
			i = 0;
		}
		else
		{
			//move forward
			if( TESTING ) cout << "Front is clear, moving forward" << endl;
			rc.moveForward(100);
		}

		//if we've travelled a decent amount, start checking if side is clear
		if(i > 3)
		{
			//check if side clear
			if( TESTING ) cout << "Checking if side is clear" << endl;
			if(!right && !dm.detectSide(true))
			{
				if( TESTING ) cout << "Right side clear, moving slightly further then turning back" << endl;
				//move a bit then turn back toward where obstacle was blocking
				rc.moveForward(140);
				rc.moveTurn(true);
				return;
			}
			else if( right && !dm.detectSide(false))
			{
				if( TESTING ) cout << "Left side clear, moving slightly further then turning back" << endl;
				//move a bit then turn back toward where obstacle was blocking
				rc.moveForward(140);
				rc.moveTurn(false);
				return;
			}
		}
	}

	if( TESTING ) cout << "Got outside of go around loop" << endl;
}