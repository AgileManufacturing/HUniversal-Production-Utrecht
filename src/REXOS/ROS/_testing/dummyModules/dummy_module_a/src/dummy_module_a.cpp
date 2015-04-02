#include "dummy_module_a/dummy_module_a.h"
#include <actionlib/client/simple_action_client.h>
#include <equiplet_node/HumanInteractionAction.h>


DummyModuleA::DummyModuleA(std::string equipletName, rexos_datatypes::ModuleIdentifier moduleIdentifier, bool isSimulated, bool isShadow) :
		equipletName(equipletName),
		rexos_module::Module(equipletName, moduleIdentifier, isSimulated, isShadow)
{
}

void DummyModuleA::run() {
	REXOS_INFO("running");
	ros::spin();
}
bool DummyModuleA::transitionInitialize() {
	REXOS_INFO("Initialize transition called");
	return true;
}

bool DummyModuleA::transitionDeinitialize() {
	REXOS_INFO("Deinitialize transition called");
	ros::shutdown();
	return true;
}


bool DummyModuleA::transitionSetup(){
	REXOS_INFO("Setup transition called");
	
	ros::Duration(5.0).sleep();
	
	rexos_module::TransitionGoal goal;
	goal.gainedSupportedMutations.push_back("move");
	
	transitionActionClient.sendGoal(goal);
	
	actionlib::SimpleActionClient<equiplet_node::HumanInteractionAction> humanInteraction(nodeHandle, equipletName + "/humanInteraction/");
	//REXOS_INFO_STREAM("client started at " << (equipletName + "/humanInteraction/"));
	
	equiplet_node::HumanInteractionGoal humanInteractionGoal;
	humanInteractionGoal.humanInteractionFormJson = "{ \"a\" : 1 }";
	REXOS_INFO_STREAM(humanInteraction.isServerConnected());
	humanInteraction.waitForServer();
	REXOS_INFO("sending data");
	humanInteraction.sendGoal(humanInteractionGoal);
	humanInteraction.waitForResult();
	
	REXOS_INFO("done");
	return true;
}
bool DummyModuleA::transitionShutdown(){
	REXOS_INFO("Shutdown transition called");
	return true;
}
bool DummyModuleA::transitionStart(){
	REXOS_INFO("Start transition called");
	return true;
}
bool DummyModuleA::transitionStop(){
	REXOS_INFO("Stop transition called");
	return true;
}

int main(int argc, char* argv[]) {
	if(argc < 5){
		REXOS_ERROR("Usage: dummy_module_a (--isSimulated | --isShadow) equipletName manufacturer typeNumber serialNumber");
		return -1;
	}
	
	bool isSimulated = false;
	bool isShadow = false;
	
	for (int i = 0; i < argc; i++) {
		std::string arg = argv[i];
		if (arg == "--isSimulated") {
			isSimulated = true;
		} else if (arg == "--isShadow") {
			isShadow = true;
			isSimulated = true;
		}
	}
	
	std::string equipletName = std::string(argv[argc - 4]);
	rexos_datatypes::ModuleIdentifier moduleIdentifier(argv[argc - 3], argv[argc - 2], argv[argc - 1]);
	
	// set up node namespace and name
	if(isShadow == true) {
		if(setenv("ROS_NAMESPACE", "shadow", 1) != 0) {
			REXOS_ERROR("Unable to set environment variable");
		}
	}
	std::string nodeName = equipletName + "_" + moduleIdentifier.getManufacturer() + "_" + 
			moduleIdentifier.getTypeNumber() + "_" + moduleIdentifier.getSerialNumber();
	ros::init(argc, argv, nodeName);
	
	DummyModuleA node(equipletName, moduleIdentifier, isSimulated, isShadow);
	
	ros::spin();
	return 0;
}
