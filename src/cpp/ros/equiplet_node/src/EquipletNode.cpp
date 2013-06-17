/**
 * @file EquipletNode.cpp
 * @brief Symbolizes an entire EquipletNode.
 * @date Created: 2012-10-12
 *
 * @author Dennis Koole
 *
 * @section LICENSE
 * License: newBSD
 *
 * Copyright © 2012, HU University of Applied Sciences Utrecht.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 * - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/

#include <equiplet_node/EquipletNode.h>

/**
 * Create a new EquipletNode
 * @param id The unique identifier of the Equiplet
 **/
EquipletNode::EquipletNode(int id): 
	equipletId(id),
	operationState(),
	safetyState(), 
	moduleTable(),
	moduleErrorService(),
	stateUpdateService(),
	modulePackageNodeMap(),
	blackboardClient(NULL){
	// Create the map with moduleType mapped to package name and node name
	modulePackageNodeMap = std::map< int, std::pair<std::string, std::string> >();
	modulePackageNodeMap[1] = std::pair< std::string, std::string > ("deltaRobotNode", "DeltaRobotNode");
	modulePackageNodeMap[2] = std::pair< std::string, std::string > ("gripperTestNode", "GripperTestNode");

	//blackboardClient = new BlackboardCppClient("localhost", "REXOS", "blackboard", this);
	//blackboardClient->subscribe("instruction");
        
        blackboardClient = new BlackboardCppClient("localhost", "CollectiveDb", "EquipletStepBB", this);
        //blackboardClient = new BlackboardCppClient("145.89.191.131", 22, "CollectiveDb", "EquipletStepBB", this);
        blackboardClient->subscribe("equipletSteps");

	std::cout << "Connected!" << std::endl;
	
	ros::NodeHandle nodeHandle;
	std::stringstream stringStream;
	stringStream << equipletId;
	std::string str = stringStream.str();
	moduleErrorService = nodeHandle.advertiseService("ModuleError_" + str, &EquipletNode::moduleError, this);
	stateUpdateService = nodeHandle.advertiseService("StateUpdate_" + str, &EquipletNode::stateChanged, this);
} 

/**
 * Destructor for the EquipletNode
 **/
EquipletNode::~EquipletNode(){
	delete blackboardClient;
}

/**
 * This function is called when a new message on the Blackboard is received,
 * The command, destination and payload are read from the message, and the 
 * service specified in the message is called
 *
 * @param json The message parsed in the json format
 **/
void EquipletNode::blackboardReadCallback(std::string json){
        //lets parse a root node from the bb msg.
    //We might want to check the type of msg first.
    JSONNode n = libjson::parse(json);
    rexos_datatypes::EquipletStep * step = new rexos_datatypes::EquipletStep(n);
    callLookupHandler(step->getInstructionData().getLook_up(), step->getInstructionData().getLook_up(), step->getInstructionData().getPayload());
}

/**
 * Callback function that is called when a message is received on the equiplet_statechanged topic
 * It updates the state of a hardware module.
 * 
 * @param request Contains the data required for a state transition
 * @param response Says if update was succesfull
 **/
bool EquipletNode::stateChanged(rexos_mast::StateUpdate::Request &request, rexos_mast::StateUpdate::Response &response){
	ROS_INFO("State changed message received");
	if(updateModuleState(request.state.moduleID, rexos_mast::StateType(request.state.newState))){
		response.succeeded = true;
	} else{
		response.succeeded = false;
	}	
	return true;
}

/**
 * Callback for when a error occurs in a module
 *
 * @param request Contains the errorCode and the ID of the module were the error occured
 * @param response Will contain the new state after error occured
 **/
bool EquipletNode::moduleError(rexos_mast::ErrorInModule::Request &request, rexos_mast::ErrorInModule::Response &response){
	int moduleID = request.moduleError.moduleID;
	ROS_INFO("Error message received from module %d", moduleID);

	// TODO: Lookup errorcode in the DB and decide accordingly

	// Lookup current state of the module
	rexos_mast::StateType currentModuleState = getModuleState(moduleID);

	// This will be changed to a proper way to decide what state should be entered on error
	response.state.moduleID = moduleID;
	response.state.newState = rexos_mast::StateType(currentModuleState - 3);
	return true;
}

/**
 * Send a StateChange request to a specific module
 * 
 * @param moduleID the unique identifier for the module which state needs to change
 * @param newState the new state for the module
 **/
void EquipletNode::sendStateChangeRequest(int moduleID, rexos_mast::StateType newState){
	rexos_mast::StateChange msg;
	msg.request.desiredState = newState;

	ros::NodeHandle nodeHandle;
	std::stringstream stringStream;
	stringStream << equipletId + "_" << moduleID;
	std::string str = stringStream.str();
	ros::ServiceClient stateChangeRequestClient = nodeHandle.serviceClient<rexos_mast::StateChange>("RequestStateChange_" + str);

	stateChangeRequestClient.call(msg);
}

/**
 * Update the safetyState of the Equiplet
 **/
void EquipletNode::updateSafetyState(){
	std::vector<HardwareModuleProperties>::iterator it;
	rexos_mast::StateType newSafetyState = rexos_mast::safe;
	for(it = moduleTable.begin(); it < moduleTable.end(); it++){
		if((*it).actor && (*it).currentState > newSafetyState){
			newSafetyState = (*it).currentState;
		}
	}
	safetyState = newSafetyState;
}

/**
 * Update the operation state of the Equiplet
 **/
void EquipletNode::updateOperationState(){
	std::vector<HardwareModuleProperties>::iterator it;
	bool operationStateSet = false;
	// first set the operation state to the highest state possible 
	rexos_mast::StateType newOperationState = rexos_mast::normal;

	for(it = moduleTable.begin(); it < moduleTable.end(); it++){
		// Set the new operation state if the hardware module is an actor, required for
		// the current service and if its state is lower than the new operation state as
		// Initialized above.
		if((*it).actor && (*it).needed){
			newOperationState = std::min((*it).currentState, newOperationState);
			operationStateSet = true;
		}
	}
	// If the operation state is not set, it means that there are no actor modules suited
	// for the current service and so the operation state is equal to the lowest state possible
	// the safe state.
	if(!operationStateSet){
		newOperationState = rexos_mast::safe;
	}
	operationState = newOperationState;
}

/**
 * Add a hardware module to the module table
 * 
 * @param module The hardware to add to the table
 *
 * @return true if the module has a unique id, otherwise false
 **/
bool EquipletNode::addHardwareModule(HardwareModuleProperties module){
	// First check if the module already exists
	std::vector<HardwareModuleProperties>::iterator it;
	for(it = moduleTable.begin(); it < moduleTable.end(); it++){
		if(module.id == (*it).id){
			return false;
		}
	}

	// Create the string that is used to start another ROS node
	std::pair< std::string, std::string > packageNodeName = modulePackageNodeMap[module.type];
	std::stringstream ss(std::stringstream::in | std::stringstream::out);
	ss << "rosrun " << packageNodeName.first << " " << packageNodeName.second << " " << equipletId << " " << module.id;
	std::cout << ss.str() << std::endl;
	int pid = -1;
	switch(pid = fork()){
		case 0:
			fclose(stderr);
			fclose(stdout);
			fclose(stdin);
			execl("/bin/sh", "/bin/sh", "-c", ss.str().c_str(), NULL);
		case -1: 
			std::cerr << "Cannot start node for hardware module " << module.id << std::endl;
			return false;
		default:
			break;
	}

	
	// Add the module to the table and update the safety state and operation state
	moduleTable.push_back(module);
	updateSafetyState();
	updateOperationState(); 
	
	return true;
}

/**
 * Remove a hardware module from the module table
 *
 * @param id The identifier that is used to identify the hardware module that needs to be removed
 *
 * @return true if the hardware module is removed, false if the module could not be found in the table
 **/
bool EquipletNode::removeHardwareModule(int id){
	std::vector<HardwareModuleProperties>::iterator it;
	for(it = moduleTable.begin(); it < moduleTable.end(); it++){
		if((*it).id == id){
			moduleTable.erase(it);
			updateSafetyState();
			updateOperationState();
			return true;
		}
	}
	return false;
}

/**
 * Print all hardware modules in the table
 **/
void EquipletNode::printHardwareModules(){
	std::vector<HardwareModuleProperties>::iterator it;
	for(it = moduleTable.begin(); it < moduleTable.end(); it++){
		std::cout << *it << std::endl;
	}
}

/**
 * Get the state of the module corresponding to the moduleID
 *
 * @param moduleID the unique identifier for a module
 *
 * @return the State of the module
 **/
rexos_mast::StateType EquipletNode::getModuleState(int moduleID){
	std::vector<HardwareModuleProperties>::iterator it;
	for(it = moduleTable.begin(); it < moduleTable.end(); it++){
		if((*it).id == moduleID){
			return (*it).currentState;
		}
	}
	return rexos_mast::nostate;
}

/**
 * Update the state of a module in the module table. Also automatically updates the operationState and
 * safe state of the Equiplet
 *
 * @param moduleID the id of the module
 * @param state the new state of the module 
 * 
 * @return true if the module is found and the state is updated, false if the module is not found in module table
 **/
bool EquipletNode::updateModuleState(int moduleID, rexos_mast::StateType state){
	std::vector<HardwareModuleProperties>::iterator it;
	for(it = moduleTable.begin(); it < moduleTable.end(); it++){
		if((*it).id == moduleID){
			(*it).currentState = state;
			updateSafetyState();
			updateOperationState();
			return true;
		}
	}
	return false;
}

/**
 * Call the lookuphandler with the data from the blackboard to get data
 *
 * @param lookupType the type of the lookup
 * @param lookupID the ID of the lookup
 * @param payload the payload, contains data that will get combined with environmentcache data
 **/
void EquipletNode::callLookupHandler(std::string lookupType, std::string lookupID, std::map<std::string, std::string> payload){
 	
    environment_communication_msgs::Map enviromentPayload = createEnviromentMap(payload);
    lookup_handler::LookupServer msg;
    msg.request.lookupMsg.lookupType = lookupType;
    msg.request.lookupMsg.lookupID = lookupID;
    msg.request.lookupMsg.payLoad = enviromentPayload;
    
    ros::NodeHandle nodeHandle;
    ros::ServiceClient lookupClient = nodeHandle.serviceClient<lookup_handler::LookupServer>("LookupHandler/lookup");
    if(lookupClient.call(msg)){
        
    } else{
            ROS_ERROR("Error in calling lookupHandler/lookup service");
    }
}

/**
 * Convert a map to a vector
 *
 **/
environment_communication_msgs::Map EquipletNode::createEnviromentMap(const std::map<std::string, std::string> Map){
    environment_communication_msgs::Map * tempMap = new environment_communication_msgs::Map();  
    std::vector<environment_communication_msgs::KeyValuePair> * tempVector = new std::vector<environment_communication_msgs::KeyValuePair>();
    
    for(map<string, string>::const_iterator i= Map.begin(); i!= Map.end(); ++i)
    {
       //tempMap->map.insert(i, std::pair<std::string,std::string>((*i).first,(*i).second));
    }
    
    tempMap->map.vector(tempVector);
    
    return *tempMap;
}

/** 
 * Main that creates the equipletNode and adds hardware modules
 **/
int main(int argc, char **argv){

	// Check if an equiplet id is given at the command line	 
	int equipletId = 1;
	if(argc != 2 || rexos_utilities::stringToInt(equipletId, argv[1]) != 0){
		std::cerr << "Cannot read equiplet id from commandline. Assuming equiplet id is 1" <<std::endl;
	}

	// Set the id of the Equiplet
	std::ostringstream ss;
	ss << "Equiplet" << equipletId;
	const char* equipletName = ss.str().c_str();
	
	ros::init(argc, argv, equipletName);
	EquipletNode equipletNode(equipletId);

	// Add some hardware modules to this equiplet
	// This should change to modules being created in the Node itself after commands on blackboard
	HardwareModuleProperties deltaRobot(1, 1, rexos_mast::safe, true, true);
	HardwareModuleProperties gripper(2, 2, rexos_mast::safe, true, true);
        
	equipletNode.addHardwareModule(deltaRobot);
	equipletNode.addHardwareModule(gripper);

	// print the hardware modules that are currently added to the Equiplet
	equipletNode.printHardwareModules();

	ros::Rate poll_rate(10);
	while(ros::ok()){
		poll_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
