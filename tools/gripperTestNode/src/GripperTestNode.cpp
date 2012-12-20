#include "GripperTestNode.h"
#include <Utilities/Utilities.h>

#define NODE_NAME "GripperTestNode"

/**
 * Constructor 
 * @param equipletID identifier for the equiplet
 * @param moduleID identifier for the gripper
 **/
GripperTestNode::GripperTestNode(int equipletID, int moduleID): rosMast::StateMachine(equipletID, moduleID)
{
	gripper = new Gripper(this, WrapperForGripperError);
	ros::NodeHandle nodeHandle;
	// Advertise the services
	gripService = nodeHandle.advertiseService(GripperTestNodeServices::GRIP, &GripperTestNode::grip, this);
	releaseService = nodeHandle.advertiseService(GripperTestNodeServices::RELEASE, &GripperTestNode::release, this);
}

GripperTestNode::~GripperTestNode()
{
	delete gripper;
}

/** 
 * A wrapper for the gripper error handler so that we can use a member function
 * @param gripperNodeObject pointer to the gripperTestNode object
 **/
void GripperTestNode::WrapperForGripperError(void* gripperNodeObject) {
	GripperTestNode* myself = (GripperTestNode*) gripperNodeObject;
	myself->error();
}

/** 
 * Sends error message to equipletNode with an errorcode
 **/
void GripperTestNode::error(){
	sendErrorMessage(-1);
}

/**
 * Transition from Safe to Standby state
 * @return 0 if everything went OK else error
 **/
int GripperTestNode::transitionSetup() {
	ROS_INFO("Setup transition called");
	
	setState(rosMast::setup); 
	return 0; 
}

/**
 * Transition from Standby to Safe state
 * @return 0 if everything went OK else error
 **/
int GripperTestNode::transitionShutdown() {	
	ROS_INFO("Shutdown transition called");	

	setState(rosMast::shutdown);
	return 0;
}

/**
 * Transition from Standby to Normal state
 * @return 0 if everything went OK else error
 **/
int GripperTestNode::transitionStart() {
	ROS_INFO("Start transition called");   

	// Set currentState to start
	setState(rosMast::start);	
	return 0;
}
/**
 * Transition from Normal to Standby state
 * @return 0 if everything went OK else error
 **/
int GripperTestNode::transitionStop() {
	ROS_INFO("Stop transition called");
	gripper->release();
	// Set currentState to stop
	setState(rosMast::stop);
	return 0;
}

/**
 * Set gripper on
 *
 * @param req The request for this service as defined in Grip.srv 
 * @param res The response for this service as defined in Grip.srv
 *
 * @return true if gripper is put on else return false.
 **/
bool GripperTestNode::grip(gripperTestNode::Grip::Request &req, gripperTestNode::Grip::Response &res)
{
	res.succeeded = false;
	if(getState() == rosMast::normal)
	{
		gripper->grab();
		res.succeeded = true;
	}
	else
	{
		res.succeeded = false;
	}
	return true;
}	

/**
 * Set gripper off
 *
 * @param req The request for this service as defined in Grip.srv 
 * @param res The response for this service as defined in Grip.srv
 *
* @return true if gripper is put off else return false.
 **/
bool GripperTestNode::release(gripperTestNode::Release::Request &req, gripperTestNode::Release::Response &res)
{
	res.succeeded = false;
	if(getState() == rosMast::normal)
	{
		gripper->release();
		res.succeeded = true;
		ROS_INFO("Gripper released");
	}
	else
	{
		res.succeeded = false;
		ROS_INFO("Gripper not released, state was not normal");
	}
	return true;
}



int main(int argc, char** argv){
	
	ros::init(argc, argv, NODE_NAME);
	int equipletID = 0;
	int moduleID = 0;
	if(argc < 3 || !(Utilities::stringToInt(equipletID, argv[1]) == 0 && Utilities::stringToInt(moduleID, argv[2]) == 0))
	{ 	 	
    	std::cerr << "Cannot read equiplet id and/or moduleId from commandline please use correct values." << std::endl;
 		return -1;
  	} 

	GripperTestNode gripperTestNode(equipletID, moduleID);    

	ros::spin();
	return 0;
}