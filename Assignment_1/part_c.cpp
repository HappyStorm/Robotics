//#include <cstdio>
//#include <cstdlib>
//#include <cmath>
//#include <algorithm>
//#include <map>
//#include <queue>
//#include <string>
//#include <vector>
//#include "Aria.h"
//#include "ArFunctor.h"
//#define EPI 0.001
//#define SONAR_DIS_ANGLE_10 666
//#define SONAR_DIS_ANGLE_30 333
//using namespace std;
//
//bool double_equals(double a, double b);
//bool isObstacleBySonar(ArRobot *robot);
//void SetRobotVelandRotVel(ArRobot *robot, double vel, double rot);
//void printFrontSonarRange(ArRobot *robot);
//void printSonarData(ArRobot *robot, int sonarID);
//
//class CallbackContainer
//{
//public:
//	void callback_up(ArRobot *robot);
//	void callback_down(ArRobot *robot);
//	void callback_left(ArRobot *robot);
//	void callback_right(ArRobot *robot);
//};
//void CallbackContainer::callback_up(ArRobot *robot)
//{
//	printf("===<UP Callback Function>===\n");
//	if (isObstacleBySonar(robot)) SetRobotVelandRotVel(robot, 0, robot->getRotVel());
//	else SetRobotVelandRotVel(robot, 500, robot->getRotVel());
//}
//void CallbackContainer::callback_down(ArRobot *robot)
//{
//	printf("===<Down Callback Function>===\n");
//	SetRobotVelandRotVel(robot, -500, robot->getRotVel());
//}
//void CallbackContainer::callback_left(ArRobot *robot)
//{
//	printf("===<Left Callback Function>===\n");
//	SetRobotVelandRotVel(robot, robot->getVel(), 25);
//}
//void CallbackContainer::callback_right(ArRobot *robot)
//{
//	printf("===<Right Callback Function>===\n");
//	SetRobotVelandRotVel(robot, robot->getVel(), -25);
//}
//
//
//int main(int argc, char **argv)
//{
//	ArRobot robot;
//	ArSonarDevice sonar;
//	CallbackContainer cb;
//
//	robot.addRangeDevice(&sonar);
//
//	Aria::init();
//	ArSimpleConnector connector(&argc, argv);
//
//	if (!connector.connectRobot(&robot)){
//		printf("Could not connect to robot... exiting\n");
//		Aria::shutdown();
//		Aria::exit(1);
//	}
//
//	robot.comInt(ArCommands::ENABLE, 1);
//	robot.runAsync(false);
//
//	ArKeyHandler keyHandler;
//	Aria::setKeyHandler(&keyHandler);
//
//	robot.attachKeyHandler(&keyHandler);
//	printf("You may press escape to exit\n");
//
//	ArFunctor1C<CallbackContainer, ArRobot*> functor_up(cb, &CallbackContainer::callback_up, &robot);
//	ArFunctor1C<CallbackContainer, ArRobot*> functor_down(cb, &CallbackContainer::callback_down, &robot);
//	ArFunctor1C<CallbackContainer, ArRobot*> functor_left(cb, &CallbackContainer::callback_left, &robot);
//	ArFunctor1C<CallbackContainer, ArRobot*> functor_right(cb, &CallbackContainer::callback_right, &robot);
//
//	keyHandler.addKeyHandler(keyHandler.UP, &functor_up);
//	keyHandler.addKeyHandler(keyHandler.DOWN, &functor_down);
//	keyHandler.addKeyHandler(keyHandler.LEFT, &functor_left);
//	keyHandler.addKeyHandler(keyHandler.RIGHT, &functor_right);
//
//	while (true){
//		if (isObstacleBySonar(&robot)) SetRobotVelandRotVel(&robot, 0, robot.getRotVel()); // robot.stop();
//		if (keyHandler.getKey() == -1) SetRobotVelandRotVel(&robot, 0, 0);
//
//		printFrontSonarRange(&robot);
//		printSonarData(&robot, 3);
//		printf("Robot(X, Y, Theta): (%6.3lf, %6.3lf, %6.3lf)\n\n", robot.getX(), robot.getY(), robot.getTh());
//		ArUtil::sleep(300);
//	}
//
//	Aria::shutdown();
//
//	Aria::exit(0);
//}
//
//bool double_equals(double a, double b)
//{
//	return abs(a - b) < EPI;
//}
//
//bool isObstacleBySonar(ArRobot *robot)
//{
//	if (robot->getSonarReading(3)->getRange() <= SONAR_DIS_ANGLE_10 || robot->getSonarReading(4)->getRange() <= SONAR_DIS_ANGLE_10 ||
//		robot->getSonarReading(2)->getRange() <= SONAR_DIS_ANGLE_30 || robot->getSonarReading(5)->getRange() <= SONAR_DIS_ANGLE_30)
//		return true;
//	else return false;
//}
//
//void SetRobotVelandRotVel(ArRobot *robot, double vel, double rot)
//{
//	robot->lock();
//	robot->setVel(vel), robot->setRotVel(rot);
//	robot->unlock();
//}
//
//void printFrontSonarRange(ArRobot *robot)
//{
//	printf("Sonar(2): %10d\n", robot->getSonarReading(2)->getRange());
//	printf("Sonar(3): %10d\n", robot->getSonarReading(3)->getRange());
//	printf("Sonar(4): %10d\n", robot->getSonarReading(4)->getRange());
//	printf("Sonar(5): %10d\n", robot->getSonarReading(5)->getRange());
//}
//
//void printSonarData(ArRobot *robot, int sonarID)
//{
//	printf("Sonar %2d:\n", sonarID);
//	printf("\tSensorDX: %8.3lf \t SensorDY: %12.3lf\n", robot->getSonarReading(sonarID)->getSensorDX(), robot->getSonarReading(sonarID)->getSensorDY());
//	printf("\tSensorX: %9.3lf \t SensorY: %13.3lf\n", robot->getSonarReading(sonarID)->getSensorX(), robot->getSonarReading(sonarID)->getSensorY());
//	printf("\tLocalX: %10.3lf \t LocalY: %14.3lf\n", robot->getSonarReading(sonarID)->getLocalX(), robot->getSonarReading(sonarID)->getLocalY());
//	printf("\tX: %15.3lf \t Y: %19.3lf\n", robot->getSonarReading(sonarID)->getX(), robot->getSonarReading(sonarID)->getY());
//}