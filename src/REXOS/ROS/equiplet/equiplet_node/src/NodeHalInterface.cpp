#include <equiplet_node/NodeHalInterface.h>

#include <rexos_logger/rexos_logger.h>

namespace equiplet_node {
	NodeHalInterface::NodeHalInterface(std::string equipletName, bool isShadow, HalInterfaceListener* listener) : 
			HalInterface(equipletName, isShadow, listener), nh(), jsonReader(), jsonWriter() {
		std::string path = equipletName + "/";
		hardwareStepStatusChangedPublisher = nh.advertise<std_msgs::String>(path + "hardwareStepStatus", 10);
		equipletCommandStatusChangedPublisher = nh.advertise<std_msgs::String>(path + "equipletCommandStatus", 10);
		stateChangedPublisher = nh.advertise<std_msgs::String>(path + "stateChanged", 10);
		modeChangedPublisher = nh.advertise<std_msgs::String>(path + "modeChanged", 10);
		violationOccuredPublisher = nh.advertise<std_msgs::String>(path + "violationOccured", 10);
		
		hardwareStepSubscriber = nh.subscribe(path + "hardwareSteps", 10, &NodeHalInterface::onHardwareStepMessage, this);
		equipletCommandSubscriber = nh.subscribe(path + "equipletCommands", 10, &NodeHalInterface::onEquipletCommandMessage, this);
	}
	void NodeHalInterface::postHardwareStepStatus(rexos_datatypes::HardwareStep hardwareStep) {
		Json::Value messageJson;
		messageJson["id"] = hardwareStep.getId();
		messageJson["status"] = hardwareStep.getStatusAsString();
		
		std_msgs::String message;
		message.data = jsonWriter.write(messageJson);
		hardwareStepStatusChangedPublisher.publish(message);
	}
	void NodeHalInterface::postEquipletCommandStatus(rexos_datatypes::EquipletCommand equipletCommand) {
		Json::Value messageJson;
		messageJson["id"] = equipletCommand.getId();
		messageJson["status"] = equipletCommand.getStatusAsString();
		
		std_msgs::String message;
		message.data = jsonWriter.write(messageJson);
		equipletCommandStatusChangedPublisher.publish(message);
	}
	void NodeHalInterface::postStateChange(rexos_datatypes::ModuleIdentifier identifier, rexos_statemachine::State state) {
		Json::Value messageJson;
		messageJson["moduleIdentifier"]["manufacturer"] = identifier.getManufacturer();
		messageJson["moduleIdentifier"]["typeNumber"] = identifier.getTypeNumber();
		messageJson["moduleIdentifier"]["serialNumber"] = identifier.getSerialNumber();
		messageJson["state"] = rexos_statemachine::state_txt[state];
		
		std_msgs::String message;
		message.data = jsonWriter.write(messageJson);
		stateChangedPublisher.publish(message);
	}
	void NodeHalInterface::postModeChange(rexos_datatypes::ModuleIdentifier identifier, rexos_statemachine::Mode mode) {
		Json::Value messageJson;
		messageJson["moduleIdentifier"]["manufacturer"] = identifier.getManufacturer();
		messageJson["moduleIdentifier"]["typeNumber"] = identifier.getTypeNumber();
		messageJson["moduleIdentifier"]["serialNumber"] = identifier.getSerialNumber();
		messageJson["mode"] = rexos_statemachine::mode_txt[mode];
		
		std_msgs::String message;
		message.data = jsonWriter.write(messageJson);
		modeChangedPublisher.publish(message);
	}
	void NodeHalInterface::postStateChange(rexos_statemachine::State state) {
		ROS_INFO("State changed");
		Json::Value messageJson;
		messageJson["state"] = rexos_statemachine::state_txt[state];
		
		std_msgs::String message;
		message.data = jsonWriter.write(messageJson);
		stateChangedPublisher.publish(message);
	}
	void NodeHalInterface::postModeChange(rexos_statemachine::Mode mode) {
		Json::Value messageJson;
		messageJson["mode"] = rexos_statemachine::mode_txt[mode];
		
		std_msgs::String message;
		message.data = jsonWriter.write(messageJson);
		modeChangedPublisher.publish(message);
	}
	void NodeHalInterface::postViolation(std::string type, std::string message) {
		Json::Value messageJson;
		messageJson["type"] = type;
		messageJson["message"] = message;
		
		std_msgs::String blackBoardMessage;
		blackBoardMessage.data = jsonWriter.write(messageJson);
		violationOccuredPublisher.publish(blackBoardMessage);
	}
	void NodeHalInterface::onHardwareStepMessage(const std_msgs::StringConstPtr& message) {
		Json::Value hardwareStepJson;
		if(jsonReader.parse(message->data, hardwareStepJson) == true) {
			rexos_datatypes::HardwareStep step(hardwareStepJson);
			step.setId(hardwareStepJson["id"].asString());
			listener->onHardwareStep(step);
		} else {
			REXOS_ERROR("Reading hardware step failed");
		}
	}
	void NodeHalInterface::onEquipletCommandMessage(const std_msgs::StringConstPtr& message) {
		Json::Value equipletCommandJson;
		if(jsonReader.parse(message->data, equipletCommandJson) == true) {
			rexos_datatypes::EquipletCommand command(equipletCommandJson);
			command.setId(equipletCommandJson["id"].asString());
			listener->onEquipletCommand(command);
		} else {
			REXOS_ERROR("Reading equiplet command failed");
		}
	}
}
