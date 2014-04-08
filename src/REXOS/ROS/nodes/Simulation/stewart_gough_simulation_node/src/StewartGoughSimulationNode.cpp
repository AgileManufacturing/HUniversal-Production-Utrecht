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

	SixAxisMath math(10,35, 10.13);
	SixAxisMath::Point3D moveTo(0,0,-43.50);

	double angleA = math.getAngleForMotor(moveTo, SixAxisMath::MOTOR_A_POS_ON_CIRCLE);
	double angleB = math.getAngleForMotor(moveTo, SixAxisMath::MOTOR_B_POS_ON_CIRCLE);
	double angleC = math.getAngleForMotor(moveTo, SixAxisMath::MOTOR_C_POS_ON_CIRCLE);
	
	motors[0].goToAngleDegrees(15.2558);
	motors[1].goToAngleDegrees(15.4123);
	motors[2].goToAngleDegrees(18.8276);
	motors[3].goToAngleDegrees(24.1848);
	motors[4].goToAngleDegrees(24.3017);
	motors[5].goToAngleDegrees(18.7872);


	ros::NodeHandle nodeHandle;
	ros::ServiceServer service = nodeHandle.advertiseService("rotate_motor_to_degrees", & StewartGoughSimulationNode::rotateMotorToDegrees, this);
	

	ROS_INFO("=====Simulation is running======");
	
	while (ros::ok()){
		for(int i = 0; i < 6; i++){
			motors[i].update();
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
