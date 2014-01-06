#include "ros/ros.h"
#include "rexos_gazebo/GazeboSDF.h"
#include <sstream>

GazeboSDF::GazeboSDF(){
	
}

void GazeboSDF::rotateJoint(const char * jointName, double speed){
	gazebo_msgs::ODEJointProperties jointConfig;
	jointConfig.hiStop.push_back(speed+0.0001);
	jointConfig.loStop.push_back(speed);
	setJointProperties(jointName,jointConfig);
}

void GazeboSDF::setLinkState(const char * linkName, gazebo_msgs::LinkState state){
	ros::NodeHandle nodeHandle;
	ros::ServiceClient client = nodeHandle.serviceClient<gazebo_msgs::SetLinkState>("gazebo/set_link_state", 1);
	gazebo_msgs::SetLinkState service;
	service.request.link_state = state;
	
	if (client.call(service)){
		//ROS_INFO("new link state was sent");
	}else{
		ROS_ERROR("Failed to call service set_link_state in setLinkState() %s" , service.response.status_message.c_str());
	}
}


gazebo_msgs::LinkState GazeboSDF::getLinkState(const char * linkName){
	ros::NodeHandle nodeHandle;
	ros::ServiceClient client = nodeHandle.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
	gazebo_msgs::GetLinkState service;
	service.request.link_name = linkName;
	
	//ros::Publisher publisher = n.advertise<gazebo_msgs::LinkState>("gazebo/set_link_state", 10);
	if (client.call(service)){
		return service.response.link_state;
	}else{
		ROS_ERROR("Failed to call service get_link_state in getLinkState() %s" , service.response.status_message.c_str());
		return service.response.link_state;
	}
}

void GazeboSDF::setJointProperties(const char * jointName, gazebo_msgs::ODEJointProperties jointConfig){
	ros::NodeHandle nodeHandle;
	ros::ServiceClient client = nodeHandle.serviceClient<gazebo_msgs::SetJointProperties>("/gazebo/set_joint_properties");
	gazebo_msgs::SetJointProperties service;
	service.request.joint_name = jointName;
	service.request.ode_joint_config = jointConfig;
	
	if (client.call(service)){
		//ROS_INFO("sent joint properties of %s success", jointName);
	}else{
		ROS_ERROR("Failed to call service set_joint_properties in setJointProperties() %s" , service.response.status_message.c_str());
	}
	
}

JointProperties GazeboSDF::getJointProperties(const char * jointName){
	ros::NodeHandle nodeHandle;
	ros::ServiceClient client = nodeHandle.serviceClient<gazebo_msgs::GetJointProperties>("/gazebo/get_joint_properties");
	gazebo_msgs::GetJointProperties service;
	service.request.joint_name = jointName;
	
	JointProperties jointProperties(0,0,0,0);
	
	if (client.call(service)){
		//ROS_INFO("got joint properties %s", service.response.status_message.c_str());
		if(service.response.damping.size() > 0){jointProperties.damping = service.response.damping[0];}
		if(service.response.position.size() > 0){jointProperties.position = service.response.position[0];}
		if(service.response.rate.size() > 0){jointProperties.rate = service.response.rate[0];}
		jointProperties.type = service.response.type;
		return jointProperties;
	}else{
		ROS_ERROR("Failed to call service get_joint_properties in getJointProperties() %s" , service.response.status_message.c_str());
		return jointProperties;
	}
	
}

void GazeboSDF::applyForceToJoint(const char * jointName, double force, double durationTime){
	ros::NodeHandle nodeHandle;
	ros::ServiceClient client = nodeHandle.serviceClient<gazebo_msgs::ApplyJointEffort>("/gazebo/apply_joint_effort");
	gazebo_msgs::ApplyJointEffort service;
	service.request.joint_name = jointName;
	service.request.effort = force;
	ros::Time time(0.0);
	service.request.start_time = time;
	ros::Duration duration(durationTime);
	service.request.duration = duration;

	if (client.call(service)){
		//ROS_INFO("force added");
	}
	else{
		ROS_ERROR("Failed to apply force to joint int applyForceToJoint() %s" , service.response.status_message.c_str());
	}
}
