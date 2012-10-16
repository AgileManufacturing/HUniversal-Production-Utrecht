//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        grippernode
// File:           main.cpp
// Description:    Contains the gripper ROSnode
// Author:         Martijn Beek
// Notes:          ...
//
// License:        GNU GPL v3
//
// This file is part of grippernode.
//
// grippernode is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// grippernode is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with grippernode.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************

#include <gripper/gripper.h>
#include "ros/ros.h"
#include "grippernode/grippersrv.h"
#include "grippernode/error.h"
#include "grippernode/registerNode.h"

using namespace std;


static gripper * grip;
static ros::Publisher * pub;

static bool gripper_status = false;
static bool overheated = false;
bool enableGripper(grippernode::grippersrv::Request &req,
		grippernode::grippersrv::Response &res)
{
	try
	{
		res.succeeded = true;
		if(req.enabled)
		{
			if(!overheated)
			{
				gripper_status = true;
				//grip->grab();
			}
			else
			{
				ROS_WARN("Tried to turn on gripper, but it's valve is currently overheated. Ignoring request");
				res.succeeded = false;
			}
		} else {
			gripper_status = false;
			//grip->release();
		}

	}
	catch(std::runtime_error& ex)
	{
		res.succeeded = false;
		grippernode::error msg;
		std::stringstream ss;
		ss << "runtime error of type "<< typeid(ex).name()<<" in delta robot" << std::endl;
		ss <<"what(): " << ex.what()<<std::endl;
		msg.errorMsg = ss.str();
		msg.errorType = 3;
		pub->publish(msg);
	}
	return true;
}


int main(int argc, char** argv)
{
	grip = new gripper("192.168.0.2", 502);
	grip->connect();

	ros::init(argc, argv, "Gripper");
	ros::NodeHandle n;
	ros::ServiceServer service2 = n.advertiseService("enableGripper", enableGripper);
	ros::Publisher pub = n.advertise<grippernode::registerNode>("NodeRegistration", 1000);
	
	ros::Rate loop_rate(1000);

	
	ros::Time gripper_went_on;
	ros::Time got_overheated;
	bool previous_gripper_status = gripper_status;
	static const int MAX_GRIPPER_ON = 60; //sec
	static const int COOLDOWN_DURATION = 3*60; //sec
	
	while(pub.getNumSubscribers()==0)
	{
	   ROS_ERROR("Waiting for subscibers");
	   sleep(10);
	}
		
	grippernode::registerNode rn;
	rn.action = "GRIPPER";
	rn.form = "enableGripper";
	pub.publish(rn);	
	ros::spinOnce();
	loop_rate.sleep();
	

	while(ros::ok())
	{
		//prevent the watchdog to trigger by sending the same command again
		if(gripper_status)
			grip->grab();
		else
			grip->release();

		if(!previous_gripper_status && gripper_status)
		{
			gripper_went_on = ros::Time::now();
		}
		else if(previous_gripper_status && gripper_status && (ros::Time::now() - gripper_went_on).toSec() > MAX_GRIPPER_ON)
		{
			ROS_WARN("Gripper valve was turned on for longer than %d seconds. Gripper will be forced to turn off now to prevent overheating", MAX_GRIPPER_ON);
			overheated = true;
			got_overheated = ros::Time::now();
			grip->release();
			previous_gripper_status = gripper_status = false;
		}
		else if(overheated && (ros::Time::now() - got_overheated).toSec() > COOLDOWN_DURATION)
		{
			ROS_WARN("Gripper valve cooled down");
			overheated = false;
		}

		previous_gripper_status = gripper_status;
		ros::spinOnce();

	}

	grip->release();
	grip->disconnect();

	
}
