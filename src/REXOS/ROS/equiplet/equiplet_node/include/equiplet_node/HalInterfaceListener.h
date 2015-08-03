#pragma once

#include <rexos_datatypes/ModuleIdentifier.h>
#include <rexos_datatypes/EquipletCommand.h>
#include <rexos_datatypes/HardwareStep.h>
#include <rexos_statemachine/State.h>
#include <rexos_statemachine/Mode.h>

namespace equiplet_node {
	class HalInterfaceListener {
	public:
		virtual void onHardwareStep(rexos_datatypes::HardwareStep hardwareStep) = 0;
		virtual void onEquipletCommand(rexos_datatypes::EquipletCommand equipletCommand) = 0;
	};
}
