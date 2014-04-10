#include "ros/ros.h"

#include "stewart_gough_simulation_node/StewartGoughSimulationNode.h"
#include <sstream>
#include <cstdlib>
#include "gazebo_msgs/LinkState.h"
#include "stewart_gough_simulation_node/SixAxisMath.h"


StewartGoughSimulationNode::StewartGoughSimulationNode()
{
	
}



void StewartGoughSimulationNode::run(int argc, char **argv){
	ros::start();
	ros::Rate loop_rate(60);

	motors[0].init("upperArmToMotor1");
	motors[1].init("upperArmToMotor2");
	motors[2].init("upperArmToMotor3");
	motors[3].init("upperArmToMotor4");
	motors[4].init("upperArmToMotor5");
	motors[5].init("upperArmToMotor6");
	
	motors[0].goToAngleDegrees(0);
	motors[1].goToAngleDegrees(0);
	motors[2].goToAngleDegrees(0);
	motors[3].goToAngleDegrees(0);
	motors[4].goToAngleDegrees(0);
	motors[5].goToAngleDegrees(0);





double angles[10][6] = {

{12.5454, 12.5454, 12.5454, 12.5454, 12.5454, 12.5454},

	{11.6507, 14.1764, 11.6507, 14.1764, 11.6507, 14.1764},

	{11.4274, 15.3098, 11.4274, 15.3098, 11.4274, 15.3098},

	{11.3349, 16.6566, 11.3349, 16.6566, 11.3349, 16.6566},

	{11.3714, 18.1977, 11.3714, 18.1977, 11.3714, 18.1977},

	{12.5454, 12.5454, 12.5454, 12.5454, 12.5454, 12.5454},

	{14.1764, 11.6507, 14.1764, 11.6507, 14.1764, 11.6507},

	{15.3098, 11.4274, 15.3098, 11.4274, 15.3098, 11.4274},

	{16.6566, 11.3349, 16.6566, 11.3349, 16.6566, 11.3349},

	{18.1977, 11.3714, 18.1977, 11.3714, 18.1977, 11.3714}};


	
	ros::NodeHandle nodeHandle;
	ros::ServiceServer service = nodeHandle.advertiseService("rotate_motor_to_degrees", & StewartGoughSimulationNode::rotateMotorToDegrees, this);
	int timer = 0;
	int patternIndex = 0;
	bool pattern = true;
	ROS_INFO("=====Simulation is running======");
	
	while (ros::ok()){
		
		for(int i = 0; i < 6; i++){
			motors[i].update();
		}
		ROS_INFO("I iz inside1");
	
		if(pattern == true){
			if(timer > 100){
				for(int i = 0; i < 6; i++){
					motors[i].goToAngleDegrees(angles[patternIndex][i]);
				}
				ROS_INFO("I iz inside2");
	
				patternIndex++;
				if(patternIndex > 9){patternIndex = 0;}
				timer = 0;
			}
			timer++;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
}

bool StewartGoughSimulationNode::rotateMotorToDegrees(stewart_gough_simulation_node::rotate::Request  & req,
									 stewart_gough_simulation_node::rotate::Response  & res){
  motors[0].goToAngleDegrees(req.motor1);
  motors[1].goToAngleDegrees(req.motor2);
  motors[2].goToAngleDegrees(req.motor3);
  motors[3].goToAngleDegrees(req.motor4);
  motors[4].goToAngleDegrees(req.motor5);
  motors[5].goToAngleDegrees(req.motor6);
  return true;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "stewart_gough_simulation_node_client");
	
	StewartGoughSimulationNode stewartGoughSimulation;
	stewartGoughSimulation.run(argc, argv);

	return 0;
}

/*

*/
