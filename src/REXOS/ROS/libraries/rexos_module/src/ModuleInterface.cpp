#include <rexos_module/ModuleProxy.h>
#include <jsoncpp/json/writer.h>

namespace rexos_module {
	ModuleInterface::ModuleInterface(std::string equipletName, rexos_datatypes::ModuleIdentifier identifier) :
			AbstractModule(equipletName, identifier),
			moduleInterfaceListener(NULL),
			executeHardwareStepClient(nodeHandle, advertisementPath + "executeHardwareStep")
	{
	}
	ModuleInterface::ModuleInterface(std::string equipletName, rexos_datatypes::ModuleIdentifier identifier, ModuleInterfaceListener* moduleInterfaceListener) :
			AbstractModule(equipletName, identifier),
			moduleInterfaceListener(moduleInterfaceListener),
			executeHardwareStepClient(nodeHandle, advertisementPath + "executeHardwareStep")
	{
	}
	void ModuleInterface::executeHardwareStep(rexos_datatypes::HardwareStep hardwareStep) {
		ROS_INFO_STREAM("Sending instruction to module: " << identifier);
		if(executeHardwareStepClient.waitForServer(ros::Duration(30)) == false) {
			throw std::runtime_error("setInstructionActionServer took too long to respond");
		}
		rexos_module::ExecuteHardwareStepGoal goal;
		Json::StyledWriter writer;
		goal.json = writer.write(hardwareStep.toJSON());
		goal.OID = hardwareStep.getId();
		
		hardwareSteps[hardwareStep.getId()] = hardwareStep;
		
		executeHardwareStepClient.sendGoal(goal, boost::bind(&ModuleInterface::onExecuteHardwareStepCallback, this, _1, _2), NULL, NULL);
	}
	void ModuleInterface::onExecuteHardwareStepCallback(const actionlib::SimpleClientGoalState& state, 
			const rexos_module::ExecuteHardwareStepResultConstPtr& result) {
		ROS_INFO_STREAM(result->OID);
		
		if(moduleInterfaceListener != NULL) {
			rexos_datatypes::HardwareStep hardwareStep = hardwareSteps[result->OID];
			hardwareSteps.erase(result->OID);
			
			if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {
				hardwareStep.setStatus(rexos_datatypes::HardwareStep::DONE);
			} else {
				hardwareStep.setStatus(rexos_datatypes::HardwareStep::FAILED);
			}
			moduleInterfaceListener->onHardwareStepCompleted(this, hardwareStep);
		}
	}
}
