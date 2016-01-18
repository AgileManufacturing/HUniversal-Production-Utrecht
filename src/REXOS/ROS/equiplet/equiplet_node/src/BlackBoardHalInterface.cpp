#include <equiplet_node/BlackBoardHalInterface.h>

#include <rexos_logger/rexos_logger.h>
#include <rexos_configuration/Configuration.h>
#include <rexos_blackboard_cpp_client/BasicOperationSubscription.h>
#include <rexos_blackboard_cpp_client/OplogEntry.h>

namespace equiplet_node {
	BlackBoardHalInterface::BlackBoardHalInterface(std::string equipletName, bool isShadow, HalInterfaceListener* listener, std::string blackboardIp) : 
			HalInterface(equipletName, isShadow, listener) {
		std::string dbName = equipletName;
		if(isShadow == true) {
			dbName = "shadow_" + equipletName;
		}
		
		bool useCustomIp = false;
		if(blackboardIp.length() != 0) useCustomIp = true;
		
		REXOS_DEBUG("Subscribing to HardwareStepsBlackBoard");
		hardwareStepsBlackboardClient = new Blackboard::BlackboardCppClient(
				useCustomIp ? blackboardIp : rexos_configuration::Configuration::getProperty("rosInterface/hardwareSteps/ip", equipletName).asString(), 
				dbName, 
				rexos_configuration::Configuration::getProperty("rosInterface/hardwareSteps/blackboardName", equipletName).asString());
		
		hardwareStepsSubscription = new Blackboard::BasicOperationSubscription(Blackboard::INSERT, *this);
		hardwareStepsBlackboardClient->subscribe(*hardwareStepsSubscription);
		subscriptions.push_back(hardwareStepsSubscription);
		sleep(1);

		REXOS_DEBUG("Subscribing to equipletCommands");
		equipletCommandsBlackboardClient = new Blackboard::BlackboardCppClient(
				useCustomIp ? blackboardIp : rexos_configuration::Configuration::getProperty("rosInterface/equipletCommands/ip", equipletName).asString(), 
				dbName, 
				rexos_configuration::Configuration::getProperty("rosInterface/equipletCommands/blackboardName", equipletName).asString());
		equipletCommandsSubscription = new Blackboard::BasicOperationSubscription(Blackboard::INSERT, *this);
		equipletCommandsBlackboardClient->subscribe(*equipletCommandsSubscription);
		subscriptions.push_back(equipletCommandsSubscription);
		sleep(1);

		REXOS_DEBUG("Subscribing to state");
		stateBlackboardClient = new Blackboard::BlackboardCppClient(
				useCustomIp ? blackboardIp : rexos_configuration::Configuration::getProperty("rosInterface/equipletState/ip", equipletName).asString(), 
				dbName, 
				rexos_configuration::Configuration::getProperty("rosInterface/equipletState/blackboardName", equipletName).asString());
		sleep(1);
		
		REXOS_DEBUG("Subscribing to violations");
		violationBlackboardClient = new Blackboard::BlackboardCppClient(
				useCustomIp ? blackboardIp : rexos_configuration::Configuration::getProperty("rosInterface/violations/ip", equipletName).asString(), 
				dbName, 
				rexos_configuration::Configuration::getProperty("rosInterface/violations/blackboardName", equipletName).asString());
		sleep(1);
	}
	BlackBoardHalInterface::~BlackBoardHalInterface() {
		for (std::vector<Blackboard::BlackboardSubscription *>::iterator iter = subscriptions.begin() ; iter != subscriptions.end() ; iter++) {
			delete *iter;
		}
		subscriptions.clear();
		
		delete hardwareStepsBlackboardClient;
		delete equipletCommandsBlackboardClient;
		delete stateBlackboardClient;
	}
	void BlackBoardHalInterface::postHardwareStepStatus(rexos_datatypes::HardwareStep hardwareStep) {
		hardwareStepsBlackboardClient->updateDocumentById(mongo::OID(hardwareStep.getId()), 
				"{$set : {status: \"" + hardwareStep.getStatusAsString() + "\"} } ");
	}
	void BlackBoardHalInterface::postEquipletCommandStatus(rexos_datatypes::EquipletCommand equipletCommand) {
		equipletCommandsBlackboardClient->updateDocumentById(mongo::OID(equipletCommand.getId()), 
				"{$set : {status: \"" + equipletCommand.getStatusAsString() + "\"} } ");
	}
	void BlackBoardHalInterface::postEquipletCommandReply(rexos_datatypes::EquipletCommand equipletCommand){
		
	}
	void BlackBoardHalInterface::postStateChange(rexos_datatypes::ModuleIdentifier identifier, rexos_statemachine::State state) {
		Json::Value messageJson;
		messageJson["moduleIdentifier"]["manufacturer"] = identifier.getManufacturer();
		messageJson["moduleIdentifier"]["typeNumber"] = identifier.getTypeNumber();
		messageJson["moduleIdentifier"]["serialNumber"] = identifier.getSerialNumber();
		messageJson["state"] = rexos_statemachine::state_txt[state];
		
		std::string message;
		message = jsonWriter.write(messageJson);
		stateBlackboardClient->insertDocument(message);
	}
	void BlackBoardHalInterface::postModeChange(rexos_datatypes::ModuleIdentifier identifier, rexos_statemachine::Mode mode) {
		Json::Value messageJson;
		messageJson["moduleIdentifier"]["manufacturer"] = identifier.getManufacturer();
		messageJson["moduleIdentifier"]["typeNumber"] = identifier.getTypeNumber();
		messageJson["moduleIdentifier"]["serialNumber"] = identifier.getSerialNumber();
		messageJson["mode"] = rexos_statemachine::mode_txt[mode];
		
		std::string message;
		message = jsonWriter.write(messageJson);
		stateBlackboardClient->insertDocument(message);
	}
	void BlackBoardHalInterface::postStateChange(rexos_statemachine::State state) {
		Json::Value messageJson;
		messageJson["state"] = rexos_statemachine::state_txt[state];
		
		std::string message;
		message = jsonWriter.write(messageJson);
		stateBlackboardClient->insertDocument(message);
	}
	void BlackBoardHalInterface::postModeChange(rexos_statemachine::Mode mode) {
		Json::Value messageJson;
		messageJson["mode"] = rexos_statemachine::mode_txt[mode];
		
		std::string message;
		message = jsonWriter.write(messageJson);
		stateBlackboardClient->insertDocument(message);
	}
	void BlackBoardHalInterface::postViolation(std::string type, std::string message) {
		Json::Value messageJson;
		messageJson["type"] = type;
		messageJson["message"] = message;
		
		std::string blackBoardMessage;
		blackBoardMessage = jsonWriter.write(messageJson);
		violationBlackboardClient->insertDocument(blackBoardMessage);
	}
	
	void BlackBoardHalInterface::onMessage(Blackboard::BlackboardSubscription & subscription, const Blackboard::OplogEntry & oplogEntry) {
		mongo::OID targetObjectId;
		oplogEntry.getTargetObjectId(targetObjectId);

		if(&subscription == hardwareStepsSubscription) {
			std::string hardwareStepString = hardwareStepsBlackboardClient->findDocumentById(targetObjectId).jsonString();
			Json::Value hardwareJson;
			if(jsonReader.parse(hardwareStepString, hardwareJson)) {
				rexos_datatypes::HardwareStep step = rexos_datatypes::HardwareStep(hardwareJson);
				step.setId(targetObjectId.toString());
				listener->onHardwareStep(step);
			} else {
				REXOS_ERROR("Reading hardware step failed");
			}
		} else if(&subscription == equipletCommandsSubscription) {
			std::string equipletCommandString = equipletCommandsBlackboardClient->findDocumentById(targetObjectId).jsonString();
			Json::Value equipletCommandJson;
			if(jsonReader.parse(equipletCommandString, equipletCommandJson) == true) {
				rexos_datatypes::EquipletCommand command(equipletCommandJson);
				command.setId(targetObjectId.toString());
				listener->onEquipletCommand(command);
			} else {
				REXOS_ERROR("Reading equiplet command failed");
			}
		}
	}
}
