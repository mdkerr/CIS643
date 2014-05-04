#include "AI.h"
#include "testing.h"

//starts robot AI
int main(int argc, char** argv)
{
	if( TESTING ) cout << "Entered main" << endl;
	//initialize and start AI
	AI ai(argc, argv);
	ai.start();
	if( TESTING ) cout << "Main finished" << endl;
}

