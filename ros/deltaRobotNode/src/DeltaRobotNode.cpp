/*
 * DeltaRobotNode.cpp
 *
 *  Created on: Sep 19, 2012
 *      Author: Dick van der Steen & Dennis Koole

// License:        newBSD
//
// Copyright © 2012, HU University of Applied Sciences Utrecht
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
// * Neither the name of the HU University of Applied Sciences Utrecht nor the
// names of its contributors may be used to endorse or promote products
// derived from this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//******************************************************************************
*/

#include "ros/ros.h"
#include <huniplacer/huniplacer.h> 
#include "deltaRobotNode/MoveToPoint.h"
#include "deltaRobotNode/MovePath.h"
#include "deltaRobotNode/MoveToRelativePoint.h"
#include "deltaRobotNode/MoveRelativePath.h"
#include "deltaRobotNode/Motion.h"

#define NODE_NAME "DeltaRobotNode"

 static huniplacer::deltarobot * deltarobot;

//callback function that gets called by the deltarobot thread when an exception occured in it
static void modbus_exhandler(std::exception& ex)
{
        std::stringstream ss;
        ss << "runtime error of type "<< typeid(ex).name()<<" in delta robot" << std::endl;
        ss <<"what(): " << ex.what()<<std::endl;
        std::cerr << ss;
}


/*
 *
 */
bool moveToPoint(deltaRobotNode::MoveToPoint::Request &req,
	deltaRobotNode::MoveToPoint::Response &res) {

	return true;
}

/*
 *
 */
bool movePath(deltaRobotNode::MovePath::Request &req,
	deltaRobotNode::MovePath::Response &res) {
	ROS_INFO("movePath called");
	res.succeeded = true;

	deltaRobotNode::Motion currentMotion;
	deltaRobotNode::Motion nextMotion;
	try
	{
		unsigned int n;
		for(n = 0; n < req.motion.size() -1; n++)
		{
			currentMotion = req.motion[n];			
			nextMotion = req.motion[n+1];
			if(!deltarobot->check_path(
				huniplacer::Point3D(currentMotion.x, currentMotion.y, currentMotion.z),
				huniplacer::Point3D(nextMotion.x, nextMotion.y, nextMotion.z)))
			{
				res.succeeded = false;
				return true;
			}
		}
		for(n = 0; n < req.motion.size(); n++)
		{	
			currentMotion = req.motion[n];
			ROS_INFO("moveTo: (%f, %f, %f) speed=%f", currentMotion.x, currentMotion.y,currentMotion.z, currentMotion.speed);
			deltarobot->moveto(huniplacer::Point3D(currentMotion.x, currentMotion.y, currentMotion.z),currentMotion.speed);
		}
		//deltaRobotNode::Motion msg;
		//msg = req.motion; 		
		//pubDeltaPos->publish(msg);
		deltarobot->wait_for_idle();
	}
	catch(std::runtime_error& ex)
	{
		//deltarobotnode::error msg;
		std::stringstream ss;
		ss << "runtime error of type "<< typeid(ex).name()<<" in delta robot" << std::endl;
		ss <<"what(): " << ex.what()<<std::endl;
		//msg.errorMsg = ss.str();
		//msg.errorType = 2;
		//pub->publish(msg);
		res.succeeded = false;
		ROS_ERROR("moveTo: %s", ss.str().c_str());
	}
	return true;
}

/*
 *
 */
bool moveToRelativePoint(deltaRobotNode::MoveToRelativePoint::Request &req,
	deltaRobotNode::MoveToRelativePoint::Response &res) {
	ROS_INFO("moveToRelativePoint called");
	deltaRobotNode::Motion currentMotion;
	try {
		currentMotion = req.motion;
		huniplacer::Point3D& effectorLocation = deltarobot->getEffectorLocation();
		ROS_INFO("Current effector location: x: %f y: %f z: %f", effectorLocation.x, effectorLocation.y, effectorLocation.z);
		double relativeX = effectorLocation.x + currentMotion.x;
		double relativeY = effectorLocation.y + currentMotion.y;
		double relativeZ = effectorLocation.z + currentMotion.z;
		ROS_INFO("Current motion z: %f", currentMotion.z);

		//std::cout << "relativeZ " << relativeZ << std::endl;
		//ROS_INFO("New effector location: x: %lf y: %lf z: %lf", relativeX, relativeY, relativeZ);
		if(!deltarobot->check_path(
				huniplacer::Point3D(effectorLocation.x, effectorLocation.y, effectorLocation.z),
				huniplacer::Point3D(relativeX, relativeY, relativeZ)))
		{
			res.succeeded = false;
			return true;
		}
		deltarobot->moveto(huniplacer::Point3D(relativeX, relativeY, relativeZ), currentMotion.speed);
		deltarobot->wait_for_idle();

	} catch(std::runtime_error& ex) {
		std::stringstream ss;
		ss << "runtime error of type "<< typeid(ex).name()<<" in delta robot" << std::endl;
		ss <<"what(): " << ex.what()<<std::endl;
		//msg.errorMsg = ss.str();
		//msg.errorType = 2;
		//pub->publish(msg);
		res.succeeded = false;
		ROS_ERROR("moveTo: %s", ss.str().c_str());
	}

    res.succeeded = true;
	return true;
}

/*
 *
 */
bool moveRelativePath(deltaRobotNode::MoveRelativePath::Request &req,
	deltaRobotNode::MoveRelativePath::Response &res) {

	return true;
}

/*
 *
 */
int main(int argc, char** argv) {
	ros::init(argc, argv, NODE_NAME);

    // Initialize modbus for IO controller
    modbus_t* modbus = modbus_new_tcp("192.168.0.2", 502);
    if(modbus == NULL)
    {
        throw std::runtime_error("Unable to allocate libmodbus context");
    }
    if(modbus_connect(modbus) == -1)
    {
        throw std::runtime_error("Connection failed");
    }
    assert(modbus != NULL);

    huniplacer::InverseKinematics kinematics(
    huniplacer::measures::BASE,
    huniplacer::measures::HIP,
    huniplacer::measures::EFFECTOR,
    huniplacer::measures::ANKLE,
    huniplacer::measures::HIP_ANKLE_ANGLE_MAX);


    modbus_t* modbus_rtu = modbus_new_rtu(
        "/dev/ttyS0",
        crd514_kd::rtu_config::BAUDRATE,
        crd514_kd::rtu_config::PARITY,
        crd514_kd::rtu_config::DATA_BITS,
        crd514_kd::rtu_config::STOP_BITS);

	double deviation[3] = {huniplacer::measures::MOTOR1_DEVIATION, huniplacer::measures::MOTOR2_DEVIATION, huniplacer::measures::MOTOR3_DEVIATION};
	huniplacer::steppermotor3 motors(modbus_rtu, huniplacer::measures::MOTOR_ROT_MIN, huniplacer::measures::MOTOR_ROT_MAX, modbus_exhandler, deviation);

	// power on the motors
	motors.power_on();

   
    deltarobot = new huniplacer::deltarobot(kinematics, motors);

    deltarobot->generate_boundaries(2);

    deltarobot->power_on();

    deltarobot->calibrateMotors(modbus);


	ros::NodeHandle nodeHandle;

	ros::ServiceServer moveToPointService =
		nodeHandle.advertiseService("moveToPoint", moveToPoint);

	ros::ServiceServer movePathService =
		nodeHandle.advertiseService("movePath", movePath);

	ros::ServiceServer moveToRelativePointService =
		nodeHandle.advertiseService("moveToRelativePoint", moveToRelativePoint);

	ros::ServiceServer moveRelativePathService =
		nodeHandle.advertiseService("moveRelativePath", moveRelativePath);



	ROS_INFO("DeltaRobotNode ready.");

	ros::spin();
	return 0;
}


