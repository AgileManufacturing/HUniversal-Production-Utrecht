#include "GripperNode/GripperNode.h"
#include "Utilities/Utilities.h"

#define NODE_NAME "GripperNode"

/**
 * The IP of the modbus we are connecting to
 **/
#define MODBUS_IP "192.168.0.2"
/**
 * The port we are connecting to
 **/
#define MODBUS_PORT 502

/**
 * Constructor
 **/
GripperNode::GripperNode(int equipletID, int moduleID): rosMast::StateMachine(equipletID, moduleID)
{

	// Initialize modbus for IO controller
	std::cout << "[DEBUG] Opening modbus connection" << std::endl;
	modbusContext = modbus_new_tcp(MODBUS_IP, MODBUS_PORT);

	if (modbusContext == NULL) {
		throw std::runtime_error("Unable to allocate libmodbus context");
	}

	if (modbus_connect(modbusContext) == -1) {
		throw std::runtime_error("Modbus connection to IO controller failed");
	}
	assert(modbusContext != NULL);

	modbus = new ModbusController::ModbusController(modbusContext);

	std::cout << "[DEBUG] Opening IO Controller" << std::endl;
	InputOutput::InputOutputController controller(modbus);

	std::cout << "[DEBUG] Starting gripper" << std::endl;
	gripper = new InputOutput::OutputDevices::Gripper(controller, this, wrapperForGripperError);
	ros::NodeHandle nodeHandle;
	// Advertise the services
	gripService = nodeHandle.advertiseService(GripperNodeServices::GRIP, &GripperNode::grip, this);
	releaseService = nodeHandle.advertiseService(GripperNodeServices::RELEASE, &GripperNode::release, this);
}

GripperNode::~GripperNode()
{
	delete gripper;
	delete modbus;
    modbus_close(modbusContext);
    modbus_free(modbusContext);
}

/**
 * A wrapper for the gripper error handler so that we can use a member function
 * @param Pointer to the gripperTestNode object
 **/
void GripperNode::wrapperForGripperError(void* gripperNodeObject) {
	GripperNode* myself = (GripperNode*) gripperNodeObject;
	myself->error();
}

/**
 * Sends error message to equipletNode with an errorcode
 **/
void GripperNode::error(){
	sendErrorMessage(-1);
}

/**
 * Transition from Safe to Standby state
 * @return 0 if everything went OK else error
 **/
int GripperNode::transitionSetup() {
	ROS_INFO("Setup transition called");

	setState(rosMast::setup);
	return 0;
}

/**
 * Transition from Standby to Safe state
 * @return 0 if everything went OK else error
 **/
int GripperNode::transitionShutdown() {
	ROS_INFO("Shutdown transition called");

	setState(rosMast::shutdown);
	return 0;
}

/**
 * Transition from Standby to Normal state
 * @return 0 if everything went OK else error
 **/
int GripperNode::transitionStart() {
	ROS_INFO("Start transition called");

	// Set currentState to start
	setState(rosMast::start);
	return 0;
}
/**
 * Transition from Normal to Standby state
 * @return 0 if everything went OK else error
 **/
int GripperNode::transitionStop() {
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
bool GripperNode::grip(gripperNode::Grip::Request &req, gripperNode::Grip::Response &res)
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
bool GripperNode::release(gripperNode::Release::Request &req, gripperNode::Release::Response &res)
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
	if(argc != 2 || (Utilities::stringToInt(equipletID, argv[1]) != 0 && Utilities::stringToInt(moduleID, argv[2]) != 0))
	{
    	std::cerr << "Cannot read equiplet id and/or moduleId from commandline please use correct values." <<std::endl;
 		return 0;
  	}


	GripperNode gripperNode(equipletID, moduleID);

	gripperNode.startStateMachine();
	return 0;
}
