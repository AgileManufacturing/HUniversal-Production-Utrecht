#pragma once

#include <rexos_datatypes/ModuleIdentifier.h>
#include <rexos_datatypes/EquipletCommand.h>
#include <rexos_datatypes/HardwareStep.h>
#include <rexos_statemachine/State.h>
#include <rexos_statemachine/Mode.h>

namespace equiplet_node {
	class HalInterface {
	public:
		virtual void postHardwareStepStatus(rexos_datatypes::HardwareStep hardwareStep) = 0;
		virtual void postEquipletCommandStatus(rexos_datatypes::EquipletCommand equipletCommand) = 0;
		virtual void postStateChange(rexos_datatypes::ModuleIdentifier identifier, rexos_statemachine::State state) = 0;
		virtual void postModeChange(rexos_datatypes::ModuleIdentifier identifier, rexos_statemachine::Mode mode) = 0;
		virtual void postStateChange(rexos_statemachine::State state) = 0;
		virtual void postModeChange(rexos_statemachine::Mode mode) = 0;
	};
}
