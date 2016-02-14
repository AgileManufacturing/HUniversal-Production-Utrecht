#pragma once

#include <equiplet_node/HalInterface.h>
#include <equiplet_node/HalInterfaceListener.h>

#include <ros/ros.h>
#include <jsoncpp/json/reader.h>
#include <jsoncpp/json/writer.h>
#include <std_msgs/String.h>

namespace equiplet_node {
	class NodeHalInterface : public HalInterface {
	public:
		NodeHalInterface(std::string equipletName, bool isShadow, HalInterfaceListener* listener);
		virtual void postHardwareStepStatus(rexos_datatypes::HardwareStep hardwareStep);
		virtual void postEquipletCommandStatus(rexos_datatypes::EquipletCommand equipletCommand);
		virtual void postEquipletCommandReply(rexos_datatypes::EquipletCommand equipletCommand);
		virtual void postStateChange(rexos_datatypes::ModuleIdentifier identifier, rexos_statemachine::State state);
		virtual void postModeChange(rexos_datatypes::ModuleIdentifier identifier, rexos_statemachine::Mode mode);
		virtual void postStateChange(rexos_statemachine::State state);
		virtual void postModeChange(rexos_statemachine::Mode mode);
		virtual void postViolation(std::string type, std::string message);
	private:
		ros::NodeHandle nh;
		
		ros::Publisher hardwareStepStatusChangedPublisher;
		ros::Publisher equipletCommandStatusChangedPublisher;
		ros::Publisher equipletCommandReplyPublisher;
		ros::Publisher stateChangedPublisher;
		ros::Publisher modeChangedPublisher;
		ros::Publisher violationOccuredPublisher;
		
		ros::Subscriber hardwareStepSubscriber;
		ros::Subscriber equipletCommandSubscriber;
		
		Json::Reader jsonReader;
		Json::StyledWriter jsonWriter;
		
		void onHardwareStepMessage(const std_msgs::StringConstPtr& message);
		void onEquipletCommandMessage(const std_msgs::StringConstPtr& message);
		void onChangeStateMessage(const std_msgs::StringConstPtr& message);
		void onChangeModeMessage(const std_msgs::StringConstPtr& message);
	};
}
