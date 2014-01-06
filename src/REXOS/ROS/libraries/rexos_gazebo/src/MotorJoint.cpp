#include "ros/ros.h"

#include "rexos_gazebo/MotorJoint.h"
#include <sstream>



MotorJoint::MotorJoint()
{
	
}

void MotorJoint::run(const char * jointName, GazeboSDF sdfController){
	gazeboSDF = sdfController;
	name = jointName;
}

void MotorJoint::goToAngleDegrees(double degrees){
	double radiansAngle = degrees*0.0174532925;
	goToAngleRadians(radiansAngle);
}

void MotorJoint::goToAngleRadians(double radians){
	destinationAngle = radians;
}


void MotorJoint::update(){
	currentAngle = gazeboSDF.getJointProperties(name).position;
	
	if((destinationAngle - currentAngle)/100 > 0.0001){
		double progressiveAngle = currentAngle;
		progressiveAngle += (destinationAngle - currentAngle)/30;
		if((destinationAngle - currentAngle)/100 < 0.0001){
			progressiveAngle = destinationAngle;
		}
		gazeboSDF.rotateJoint(name, progressiveAngle);
	}else if((destinationAngle - currentAngle)/100 < -0.0001){
		double progressiveAngle = currentAngle;
		progressiveAngle += (destinationAngle - currentAngle)/30;
		if((destinationAngle - currentAngle)/100 > -0.0001){
			progressiveAngle = destinationAngle;
		}
		gazeboSDF.rotateJoint(name, progressiveAngle);
			
	}
}
