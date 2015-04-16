#include <rexos_module/ModuleProxy.h>
#include <jsoncpp/json/writer.h>

namespace rexos_module {
	ModuleInterface::ModuleInterface(std::string equipletName, rexos_datatypes::ModuleIdentifier identifier) :
			AbstractModule(equipletName, identifier),
			moduleInterfaceListener(NULL),
			setInstructionActionClient(nodeHandle, advertisementPath + "set_instruction")
	{
	}
	ModuleInterface::ModuleInterface(std::string equipletName, rexos_datatypes::ModuleIdentifier identifier, ModuleInterfaceListener* moduleInterfaceListener) :
			AbstractModule(equipletName, identifier),
			moduleInterfaceListener(moduleInterfaceListener),
			setInstructionActionClient(nodeHandle, advertisementPath + "set_instruction")
	{
	}
	void ModuleInterface::setInstruction(std::string OID, Json::Value n) {
		ROS_INFO_STREAM("Sending instruction to module: " << identifier);
		if(setInstructionActionClient.waitForServer(ros::Duration(30)) == false) {
			throw std::runtime_error("setInstructionActionServer took too long to respond");
		}
		rexos_module::SetInstructionGoal goal;
		Json::StyledWriter writer;
		goal.json = writer.write(n);
		goal.OID = OID;
		
		setInstructionActionClient.sendGoal(goal, boost::bind(&ModuleInterface::onInstructionServiceCallback, this, _1, _2), NULL, NULL);
	}
	void ModuleInterface::onInstructionServiceCallback(const actionlib::SimpleClientGoalState& state, 
			const rexos_module::SetInstructionResultConstPtr& result) {
		if(moduleInterfaceListener != NULL) {
			if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {
				moduleInterfaceListener->onHardwareStepCompleted(this, result->OID, true);
			} else {
				moduleInterfaceListener->onHardwareStepCompleted(this, result->OID, false);
			}
		}
	}
}
