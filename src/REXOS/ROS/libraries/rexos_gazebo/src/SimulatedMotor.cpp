#include "ros/ros.h"
#include "rexos_gazebo/SimulatedMotor.h"
#include <sstream>



SimulatedMotor::SimulatedMotor()
{
	
}

void SimulatedMotor::init(const char * jointName){
	jump = false;
	name = jointName;
}

void SimulatedMotor::goToAngleDegrees(double degrees){
	double radiansAngle = degrees*0.0174532925;
	goToAngleRadians(radiansAngle);
}

void SimulatedMotor::goToAngleRadians(double radians){
	radians *=-1;
	destinationAngle = radians;
}


void SimulatedMotor::update(){
	currentAngle = sdfController.getJointProperties(name).position;
		
	if(!jump){
		
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
	}else{
		if(destinationAngle != currentAngle){
			sdfController.rotateJoint(name, destinationAngle);
		}
	}
}
