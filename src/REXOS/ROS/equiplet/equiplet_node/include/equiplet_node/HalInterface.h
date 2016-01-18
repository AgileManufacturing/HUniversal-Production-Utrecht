#pragma once

#include <equiplet_node/HalInterfaceListener.h>
#include <rexos_datatypes/ModuleIdentifier.h>
#include <rexos_datatypes/EquipletCommand.h>
#include <rexos_datatypes/HardwareStep.h>
#include <rexos_statemachine/State.h>
#include <rexos_statemachine/Mode.h>

namespace equiplet_node {
	class HalInterface {
	protected:
		HalInterface(std::string equipletName, bool isShadow, HalInterfaceListener* listener) :
				equipletName(equipletName), isShadow(isShadow), listener(listener) {}
		std::string equipletName;
		bool isShadow;
		HalInterfaceListener* listener;
	public:
		virtual ~HalInterface(){ }; // virtual destructor must be implemented
		virtual void postHardwareStepStatus(rexos_datatypes::HardwareStep hardwareStep) = 0;
		virtual void postEquipletCommandStatus(rexos_datatypes::EquipletCommand equipletCommand) = 0;
		//virtual void postEquipletCommandReply(rexos_datatypes::EquipletCommand equipletCommand);
		virtual void postStateChange(rexos_datatypes::ModuleIdentifier identifier, rexos_statemachine::State state) = 0;
		virtual void postModeChange(rexos_datatypes::ModuleIdentifier identifier, rexos_statemachine::Mode mode) = 0;
		virtual void postStateChange(rexos_statemachine::State state) = 0;
		virtual void postModeChange(rexos_statemachine::Mode mode) = 0;
		virtual void postViolation(std::string type, std::string message) = 0;
	};
}
