#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <ctime>
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
#define RADIAN2DEGREE 360/(2*PI)
using namespace std;

bool double_equals(double a, double b, double epi);
bool isObstacleBySonar(ArRobot *robot);
void setRobotVelandRotVel(ArRobot *robot, double vel, double rot);
void roateRobotToMove(ArRobot *robot, ArPose *target);
void rotateRobotToFinish(ArRobot *robot, ArPose *target);
void moveRobot(ArRobot *robot, ArPose *target);
void printErrorRange(ArRobot *robot, ArPose *target, double time);

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
	double start, end;
	while (true){
		if (isObstacleBySonar(&robot)) setRobotVelandRotVel(&robot, 0, robot.getRotVel());
		double tx, ty, tth, dis2go, angle2go;
		scanf("%lf %lf %lf", &tx, &ty, &tth);
		start = clock();
		ArPose *target = new ArPose(tx * 1000, ty * 1000, tth*RADIAN2DEGREE);
		ArPose initPost = robot.getPose();
		roateRobotToMove(&robot, target);
		moveRobot(&robot, target);
		rotateRobotToFinish(&robot, target);
		end = clock();
		printErrorRange(&robot, target, end - start);
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

void roateRobotToMove(ArRobot *robot, ArPose *target)
{
	double dis2go = robot->findDistanceTo(*target),
		angle2go = robot->findAngleTo(*target),
		anglediff = abs(robot->getTh() - angle2go),
		cw = abs(360 - angle2go),
		ccw = abs(robot->getTh() - angle2go);
	while (anglediff >= 0.8) {
		double step = (anglediff >= 25) ? (anglediff<50 ? 25 : 50) : (anglediff<6.25 ? 6.25 : 12.5);
		if (ccw > cw) setRobotVelandRotVel(robot, 0, -step);
		else setRobotVelandRotVel(robot, 0, step);
		anglediff = abs(robot->getTh() - angle2go);
	}
	setRobotVelandRotVel(robot, 0, -3.125);
	ArUtil::sleep(315);
	setRobotVelandRotVel(robot, 0, 0);
}

void rotateRobotToFinish(ArRobot *robot, ArPose *target)
{
	double anglediff = abs(robot->getTh() - target->getTh());
	while (anglediff >= 0.8){
		double step = (anglediff >= 25) ? (anglediff<50 ? 25 : 50) : (anglediff<6.25 ? 6.25 : 12.5);
		if (robot->getTh() > target->getTh()) setRobotVelandRotVel(robot, 0, -step);
		else setRobotVelandRotVel(robot, 0, step);
		anglediff = abs(robot->getTh() - target->getTh());
	}
	setRobotVelandRotVel(robot, 0, -3.125);
	ArUtil::sleep(315);
	setRobotVelandRotVel(robot, 0, 0);
}

void moveRobot(ArRobot *robot, ArPose *target)
{
	double dis2go = robot->findDistanceTo(*target);
	while (dis2go >= 225){
		setRobotVelandRotVel(robot, 1000, 0);
		dis2go = robot->findDistanceTo(*target);
	}
	setRobotVelandRotVel(robot, -66, 0);
	ArUtil::sleep(500);
	setRobotVelandRotVel(robot, 0, 0);
}

void printErrorRange(ArRobot *robot, ArPose *target, double time)
{
	printf("Robot(X, Y, Theta): (%6.3lf, %6.3lf, %6.3lf)\n", robot->getX(), robot->getY(), robot->getTh());
	printf("Target(X, Y, Theta): (%6.3lf, %6.3lf, %6.3lf)\n", target->getX(), target->getY(), target->getTh());
	printf("Distance Error: %6.3f\n", robot->findDistanceTo(*target));
	printf("Angle Error: %6.3f\n", abs(robot->getTh()-target->getTh()));
	printf("Total Time: %6.3lf\n", time / CLOCKS_PER_SEC);
}
