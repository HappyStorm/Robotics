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
void SetRobotVelandRotVel(ArRobot *robot, double vel, double rot);
void printFrontSonarRange(ArRobot *robot);
void printSonarData(ArRobot *robot, int sonarID);


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
	printf("You may press escape to exit\n");

	while (true){
		if (isObstacleBySonar(&robot)) SetRobotVelandRotVel(&robot, 0, robot.getRotVel()); // robot.stop();
		double tx, ty, tth, dis2go, angle2go;
		scanf("%lf %lf %lf", &tx, &ty, &tth);
		ArPose *targetPose = new ArPose(tx * 1000, ty * 1000, tth*RADIAN2DEGREE);
		ArPose initPost = robot.getPose();
		printf("Robot(X, Y, Theta): (%6.3lf, %6.3lf, %6.3lf)\n", robot.getX(), robot.getY(), robot.getTh());
		printf("Target(X, Y, Theta): (%6.3lf, %6.3lf, %6.3lf)\n", targetPose->getX(), targetPose->getY(), targetPose->getTh());
		dis2go = robot.findDistanceTo(*targetPose);
		angle2go = robot.findAngleTo(*targetPose);
		
		printf("Angle to Go: %lf\n", angle2go);
		while (!double_equals(robot.getTh(), angle2go, 2)){
			printf("1 RobotTheta: %.3lf, TargetTheta: %.31lf\n", robot.getTh(), targetPose->getTh());
			double step = (abs(robot.getTh() - targetPose->getTh()) <= 10) ? 2 : 15;
			if (robot.getTh() > targetPose->getTh()) SetRobotVelandRotVel(&robot, 0, -step);
			else SetRobotVelandRotVel(&robot, 0, step);
		}
		SetRobotVelandRotVel(&robot, robot.getVel(), 0);
		printf("Dis: %lf\n", dis2go);

		robot.move(dis2go-175);
		while (!robot.isMoveDone());

		dis2go = robot.findDistanceTo(*targetPose);
		printf("Left Dis = %.lf\n", dis2go);
		while (dis2go >= 300){
			printf("Dis: %lf\n", dis2go);
			SetRobotVelandRotVel(&robot, 25, 0);
			dis2go = robot.findDistanceTo(*targetPose);
		}
		SetRobotVelandRotVel(&robot, 0, 0);

		while (!double_equals(robot.getTh(), targetPose->getTh(), 2.5)){
			printf("2 RobotTheta: %.3lf, TargetTheta: %.31lf\n", robot.getTh(), targetPose->getTh());
			double step = (abs(robot.getTh() - targetPose->getTh()) <= 10) ? 2 : 15;
			if (robot.getTh() > targetPose->getTh()) SetRobotVelandRotVel(&robot, 0, -step);
			else SetRobotVelandRotVel(&robot, 0, step);
		}
		SetRobotVelandRotVel(&robot, 0, 0);
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

void SetRobotVelandRotVel(ArRobot *robot, double vel, double rot)
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