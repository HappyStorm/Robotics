#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <algorithm>
#include <map>
#include <queue>
#include <string>
#include <vector>
#include "Aria.h"
#include "ArFunctor.h"
#define EPI 0.001
#define SONAR_DIS_ANGLE_10 666
#define SONAR_DIS_ANGLE_30 333
#define PI 3.1415926
#define ANGLEEPI ((2*PI)/360)*3
#define RADIAN2DEGREE 360/(2*PI)
using namespace std;

bool double_equals(double a, double b, double epi);
bool isObstacleBySonar(ArRobot *robot);
void setRobotVelandRotVel(ArRobot *robot, double vel, double rot);
void printFrontSonarRange(ArRobot *robot);
void printSonarData(ArRobot *robot, int sonarID);
void printRobotandTargetLocation(ArRobot *robot, ArPose *target);
void roateRobotToMove(ArRobot *robot, ArPose *target);
void rotateRobotToFinish(ArRobot *robot, ArPose *target);
void moveRobotFast(ArRobot *robot, ArPose *target);
void moveRobotSlow(ArRobot *robot, ArPose *target);


int main(int argc, char **argv)
{
	ArRobot robot;
	ArSonarDevice sonar;
	robot.addRangeDevice(&sonar);

	Aria::init();
	ArSimpleConnector connector(&argc, argv);
	if (!connector.connectRobot(&robot)){
		printf("Could not connect to robot... exiting\n");
		Aria::shutdown();
		Aria::exit(1);
	}
	robot.comInt(ArCommands::ENABLE, 1);
	robot.runAsync(false);

	ArKeyHandler keyHandler;
	Aria::setKeyHandler(&keyHandler);
	robot.attachKeyHandler(&keyHandler);
	
	printf("You may press escape to exit.\n");
	while (true){
		if (isObstacleBySonar(&robot)) setRobotVelandRotVel(&robot, 0, robot.getRotVel()); // robot.stop();
		double tx, ty, tth, dis2go, angle2go;
		scanf("%lf %lf %lf", &tx, &ty, &tth);
		ArPose *target = new ArPose(tx * 1000, ty * 1000, tth*RADIAN2DEGREE);
		ArPose initPost = robot.getPose();
		printRobotandTargetLocation(&robot, target);
		
		roateRobotToMove(&robot, target);
		moveRobotFast(&robot, target);
		// roateRobotToMove(&robot, target);
		moveRobotSlow(&robot, target);
		rotateRobotToFinish(&robot, target);

		printFrontSonarRange(&robot);
		printSonarData(&robot, 3);
		printf("Robot(X, Y, Theta): (%6.3lf, %6.3lf, %6.3lf)\n\n", robot.getX(), robot.getY(), robot.getTh());
		robot.moveTo(*new ArPose(0, 0, 0));
		ArUtil::sleep(300);
	}

	Aria::shutdown();
	Aria::exit(0);
}

bool double_equals(double a, double b, double epi)
{
	return abs(a - b) <= epi;
}

bool isObstacleBySonar(ArRobot *robot)
{
	if (robot->getSonarReading(3)->getRange() <= SONAR_DIS_ANGLE_10 || robot->getSonarReading(4)->getRange() <= SONAR_DIS_ANGLE_10 ||
		robot->getSonarReading(2)->getRange() <= SONAR_DIS_ANGLE_30 || robot->getSonarReading(5)->getRange() <= SONAR_DIS_ANGLE_30)
		return true;
	else return false;
}

void setRobotVelandRotVel(ArRobot *robot, double vel, double rot)
{
	robot->lock();
	robot->setVel(vel), robot->setRotVel(rot);
	robot->unlock();
}

void printFrontSonarRange(ArRobot *robot)
{
	printf("Sonar(2): %10d\n", robot->getSonarReading(2)->getRange());
	printf("Sonar(3): %10d\n", robot->getSonarReading(3)->getRange());
	printf("Sonar(4): %10d\n", robot->getSonarReading(4)->getRange());
	printf("Sonar(5): %10d\n", robot->getSonarReading(5)->getRange());
}

void printSonarData(ArRobot *robot, int sonarID)
{
	printf("Sonar %2d:\n", sonarID);
	printf("\tSensorDX: %8.3lf \t SensorDY: %12.3lf\n", robot->getSonarReading(sonarID)->getSensorDX(), robot->getSonarReading(sonarID)->getSensorDY());
	printf("\tSensorX: %9.3lf \t SensorY: %13.3lf\n",	robot->getSonarReading(sonarID)->getSensorX(),	robot->getSonarReading(sonarID)->getSensorY());
	printf("\tLocalX: %10.3lf \t LocalY: %14.3lf\n",	robot->getSonarReading(sonarID)->getLocalX(),	robot->getSonarReading(sonarID)->getLocalY());
	printf("\tX: %15.3lf \t Y: %19.3lf\n",				robot->getSonarReading(sonarID)->getX(),		robot->getSonarReading(sonarID)->getY());
}

void printRobotandTargetLocation(ArRobot *robot, ArPose *target)
{
	printf("Robot(X, Y, Theta): (%6.3lf, %6.3lf, %6.3lf)\n", robot->getX(), robot->getY(), robot->getTh());
	printf("Target(X, Y, Theta): (%6.3lf, %6.3lf, %6.3lf)\n", target->getX(), target->getY(), target->getTh());
}

void roateRobotToMove(ArRobot *robot, ArPose *target)
{
	double dis2go = robot->findDistanceTo(*target),
		angle2go = robot->findAngleTo(*target),
		anglediff = abs(robot->getTh() - angle2go),
		cw = abs(360 - angle2go),
		ccw = abs(robot->getTh() - angle2go);
	while (anglediff >= 0.8){
		printf("Angle to Rotate: %.3lf\n", angle2go);
		printf("R-Theta: %.3lf, T-Theta: %.3lf\n", robot->getTh(), target->getTh());
		double step = (anglediff >= 20) ? (anglediff<50 ? 20 : 50) : (anglediff<4 ? 1 : 10);
		if (ccw > cw) setRobotVelandRotVel(robot, 0, -step);
		else setRobotVelandRotVel(robot, 0, step);
		anglediff = abs(robot->getTh() - angle2go);
	}
	setRobotVelandRotVel(robot, 0, 0);
}

void rotateRobotToFinish(ArRobot *robot, ArPose *target)
{
	double anglediff = abs(robot->getTh() - target->getTh());
	while (anglediff >= 2){
		printf("R-Theta: %.3lf, T-Theta: %.3lf\n", robot->getTh(), target->getTh());
		double step = (anglediff >= 20) ? (anglediff<50 ? 20 : 50) : (anglediff<4 ? 1 : 10);
		if (robot->getTh() > target->getTh()) setRobotVelandRotVel(robot, 0, -step);
		else setRobotVelandRotVel(robot, 0, step);
		anglediff = abs(robot->getTh() - target->getTh());
	}
	setRobotVelandRotVel(robot, 0, 0);
}

void moveRobotFast(ArRobot *robot, ArPose *target)
{
	double dis2go = robot->findDistanceTo(*target)-600;
	while (dis2go >= 600){
		printf("Dis: %lf\n", dis2go);
		setRobotVelandRotVel(robot, 800, 0);
		dis2go = robot->findDistanceTo(*target)-600;
	}
	setRobotVelandRotVel(robot, 0, 0);
}

void moveRobotSlow(ArRobot *robot, ArPose *target)
{
	double dis2go = robot->findDistanceTo(*target);
	while (dis2go >= 150){
		printf("Dis: %lf\n", dis2go);
		setRobotVelandRotVel(robot, 200, 0);
		dis2go = robot->findDistanceTo(*target);
	}
	setRobotVelandRotVel(robot, 0, 0);
}