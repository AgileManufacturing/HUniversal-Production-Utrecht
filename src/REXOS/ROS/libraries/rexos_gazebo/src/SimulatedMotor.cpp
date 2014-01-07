#include "ros/ros.h"
#include "rexos_gazebo/SimulatedMotor.h"
#include <sstream>



SimulatedMotor::SimulatedMotor()
{
	
}

void SimulatedMotor::init(const char * jointName){
	name = jointName;
}

void SimulatedMotor::goToAngleDegrees(double degrees){
	double radiansAngle = degrees*0.0174532925;
	goToAngleRadians(radiansAngle);
}

void SimulatedMotor::goToAngleRadians(double radians){
	destinationAngle = radians;
}


void SimulatedMotor::update(){
	currentAngle = sdfController.getJointProperties(name).position;
	
	if((destinationAngle - currentAngle)/100 > 0.0001){
		double progressiveAngle = currentAngle;
		progressiveAngle += (destinationAngle - currentAngle)/30;
		if((destinationAngle - currentAngle)/100 < 0.0001){
			progressiveAngle = destinationAngle;
		}
		sdfController.rotateJoint(name, progressiveAngle);
	}else if((destinationAngle - currentAngle)/100 < -0.0001){
		double progressiveAngle = currentAngle;
		progressiveAngle += (destinationAngle - currentAngle)/30;
		if((destinationAngle - currentAngle)/100 > -0.0001){
			progressiveAngle = destinationAngle;
		}
		sdfController.rotateJoint(name, progressiveAngle);
			
	}
}
