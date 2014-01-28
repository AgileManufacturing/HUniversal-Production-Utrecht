#include "ros/ros.h"

#include "stewart_gough_simulation_node/StewartGoughSimulationNode.h"
#include <sstream>
#include <cstdlib>
#include "gazebo_msgs/LinkState.h"


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
	
	//motor1.goToAngleDegrees(10);
	//motor2.goToAngleDegrees(10);
	//motor3.goToAngleDegrees(10);
	//motor4.goToAngleDegrees(10);
	//motor5.goToAngleDegrees(0);
	//motor6.goToAngleDegrees(0);
	
	ros::NodeHandle nodeHandle;

	ros::ServiceServer service = nodeHandle.advertiseService("rotate_motor_to_degrees", & StewartGoughSimulationNode::rotateMotorToDegrees, this);
	
	int timer = 0;
	double degrees[11][6]{
		{30,30,30,30,30,30},
		{-40,-40,-40,-40,-40,-40},
		{-10,10,-10,10,-10,10},
		{10,-10,10,-10,10,-10},
		{-50,-50,20,20,20,20},
		{20,20,-50,-50,20,20},
		{-50,-50,20,20,-50,-50},
		{15,05,-30,-40,-30,-40},
		{-40,-30,15,5,-30,-40},
		{30,30,-40,-40,-40,-40},
		{0,0,0,0,0,0}
	};
	int patternIndex = 0;
	bool pattern = true;
	ROS_INFO("=====Simulation is running======");
	while (ros::ok())
	{
		ROS_INFO("effector X: %f",sdfController.getLinkState("platform").pose.position.x);
		ROS_INFO("effector Y: %f",sdfController.getLinkState("platform").pose.position.y);
		ROS_INFO("effector Z: %f",sdfController.getLinkState("platform").pose.position.z);
		for(int i = 0; i < 6; i++){
			motors[i].update();
		}
		
		if(pattern == true){
			ROS_INFO("in pattern");
			if(timer > 100){
				for(int i = 0; i < 6; i++){
					motors[i].goToAngleDegrees(degrees[patternIndex][i]);
				}
				
				patternIndex++;
				if(patternIndex > 10){patternIndex = 0;}
				timer = 0;
			}
			timer++;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	//ros::spin();
}

bool StewartGoughSimulationNode::rotateMotorToDegrees(stewart_gough_simulation_node::rotate::Request  & req,
									 stewart_gough_simulation_node::rotate::Response  & res){
  motors[req.motorIndex].goToAngleDegrees(req.degrees);
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
