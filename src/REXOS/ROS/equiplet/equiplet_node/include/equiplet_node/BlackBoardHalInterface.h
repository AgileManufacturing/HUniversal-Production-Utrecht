#pragma once

#include <equiplet_node/HalInterface.h>
#include <equiplet_node/HalInterfaceListener.h>

#include <rexos_blackboard_cpp_client/BlackboardCppClient.h>
#include <rexos_blackboard_cpp_client/BlackboardSubscriber.h>

#include <jsoncpp/json/reader.h>
#include <jsoncpp/json/writer.h>

namespace equiplet_node {
	class BlackBoardHalInterface : public HalInterface, public Blackboard::BlackboardSubscriber {
	public:
		BlackBoardHalInterface(std::string equipletName, HalInterfaceListener* listener, std::string blackboardIp);
		~BlackBoardHalInterface();
		
		virtual void postHardwareStepStatus(rexos_datatypes::HardwareStep hardwareStep);
		virtual void postEquipletCommandStatus(rexos_datatypes::EquipletCommand equipletCommand);
		virtual void postStateChange(rexos_datatypes::ModuleIdentifier identifier, rexos_statemachine::State state);
		virtual void postModeChange(rexos_datatypes::ModuleIdentifier identifier, rexos_statemachine::Mode mode);
		virtual void postStateChange(rexos_statemachine::State state);
		virtual void postModeChange(rexos_statemachine::Mode mode);
	private:
		HalInterfaceListener* listener;
		
		Blackboard::BlackboardCppClient* hardwareStepsBlackboardClient;
		Blackboard::BlackboardSubscription* hardwareStepsSubscription;

		Blackboard::BlackboardCppClient* equipletCommandsBlackboardClient;
		Blackboard::BlackboardSubscription* equipletCommandsSubscription; 

		Blackboard::BlackboardCppClient* stateBlackboardClient;
		
		std::vector<Blackboard::BlackboardSubscription*> subscriptions; 

		Json::Reader jsonReader;
		Json::StyledWriter jsonWriter;
		
		void onMessage(Blackboard::BlackboardSubscription & subscription, const Blackboard::OplogEntry & oplogEntry);
	};
}
