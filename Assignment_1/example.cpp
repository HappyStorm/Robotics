#include "Aria.h"
using namespace std;

// Packet Handlers:
// 	Server Information Packets (SIPs):
// 		They are packets sent by the robot server containing information updates about 
// 		the robot and its accessories. The standard SIP is sent by the robot to a connected
// 		client automatically every 100 milliseconds 
// 		(this frequency may be configured in the firmware parameters).
// 		It contains the robot's current position and estimates, current translational
// 		and rotational speeds, sonar reading updates, battery voltage, analog and
// 		digital I/O states, and more. These data are stored and used by
// 		ArRobot's State Reflection (see State Reflection below) and are accessible via
// 		methods of the ArRobot class. (Note, within the ArRobot source code
// 		the standard SIP is also called a "motor" packet.)

int main(int argc, char **argv)
{
	ArRobot robot;
	ArSonarDevice sonar;

	robot.addRangeDevice(&sonar);

	Aria::init();
	ArSimpleConnector connector(&argc,argv);

	if (!connector.connectRobot(&robot)){
		printf("Could not connect to robot... exiting\n");
		Aria::shutdown();
		Aria::exit(1);
	}

	robot.comInt(ArCommands::ENABLE, 1);

	robot.runAsync(false);

	// Used to perform actions when keyboard keys are pressed
	ArKeyHandler keyHandler;
	Aria::setKeyHandler(&keyHandler);

	// ArRobot contains an exit action for the Escape key. It also 
	// stores a pointer to the keyhandler so that other parts of the program can
	// use the same keyhandler.
	robot.attachKeyHandler(&keyHandler);
	printf("You may press escape to exit\n");

	// TODO: control the robot
	
	// Start of controling


	// 1. Lock the robot
	robot.lock();


	// 2. Write your control code here,
	//    e.g. robot.setVel(150);  
	robot.setVel(150);


	// 3. Unlock the robot
	robot.unlock();

	// 4. Sleep a while and let the robot move
	while(true){
		printf("%f %f %f\n", robot.getX(), robot.getY(), robot.getTh());
		ArUtil::sleep(300);
	}

	// End of controling


	Aria::shutdown();

	Aria::exit(0);
}
