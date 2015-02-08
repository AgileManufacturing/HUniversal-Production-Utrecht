/**
 * @file KeyBoardControlNode.cpp
 * @brief Control the robot with the WASD UP and Down keys.
 * @date Created: 2014-11-05
 *
 * @author Dick van der Steen
 * @author Dennis Koole
 * @author Tommas Bakker
 * @author Peter Markotic
 *
 * @section LICENSE
 * License: newBSD
 * 
 * Copyright © 2012, HU University of Applied Sciences Utrecht.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 * - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/

#include <hardcoded_mover_node/HardcodedMoverNode.h>
#include <jsoncpp/json/value.h>
#include <jsoncpp/json/writer.h>
#include <rexos_datatypes/EquipletStep.h>

// @cond HIDE_NODE_NAME_FROM_DOXYGEN
#define NODE_NAME "HardcodedMoverNode"
// @endcond

using namespace hardcoded_mover_node;


HardcodedMoverNode::HardcodedMoverNode(std::string equipletName, rexos_datatypes::ModuleIdentifier moduleIdentifier, 
		std::vector<Movement> movements, bool loop) :
	equipletName(equipletName), identifier(moduleIdentifier), loop(loop), movements(movements), 
	moduleInterface(equipletName, identifier, this)
{
	REXOS_INFO("Constructing");
	movementIterator = this->movements.begin();
	
	if(this->movements.size() == 0) {
		throw std::runtime_error("Require at least one movement to be specified");
	}
	
	startNextMovement("1");
	
	ros::spin();
}

HardcodedMoverNode::~HardcodedMoverNode() {
	REXOS_INFO("Destructing");
}

void HardcodedMoverNode::onHardwareStepCompleted(rexos_module::ModuleInterface* moduleInterface, std::string id, bool completed) {
	REXOS_INFO("done with hardware step");
	
	movementIterator++;
	if(movementIterator == movements.end() && loop == true) {
		movementIterator = movements.begin();
		REXOS_INFO("looping movements");
	} else if(movementIterator == movements.end() && loop == false) {
		REXOS_INFO("done with movements, shutting down");
		ros::shutdown();
		return;
	}
	startNextMovement(id);
}

void HardcodedMoverNode::startNextMovement(string id) {
	rexos_datatypes::EquipletStep hardwareStep;
	hardwareStep.setId(id);
	hardwareStep.setModuleIdentifier(identifier);
	
	Movement movement = *movementIterator;
	hardwareStep.setOriginPlacement(movement.originPlacement);
	
	Json::Value instructionData;
	Json::Value move;
	move["x"] = movement.moveX;
	move["y"] = movement.moveY;
	move["z"] = movement.moveZ;
	move["acceleration"] = movement.acceleration;
	instructionData["move"] = move;
	Json::Value rotate;
	rotate["x"] = movement.rotateX;
	rotate["y"] = movement.rotateY;
	rotate["z"] = movement.rotateZ;
	instructionData["rotate"] = rotate;
	hardwareStep.setInstructionData(instructionData);
	
	moduleInterface.setInstruction(id, hardwareStep.toJSON());
}

/**
* Starting method for the KeyBoardControlNode.
*
* @param argc Argument count.
* @param argv Argument vector.
* 
* @return 1 on keyboard read error.
**/
int main(int argc, char** argv){
	// Ros init.
	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle nodeHandle;
	
	std::vector<HardcodedMoverNode::Movement> movements;
	
	HardcodedMoverNode::Movement movement;
	movement.originPlacement.setOriginPlacementType(rexos_datatypes::OriginPlacement::RELATIVE_TO_MODULE_ORIGIN);
	movement.acceleration = 5.0;
	
	movement.moveX = 50.0;
	movement.moveY = 0.0;
	movement.moveZ = -200.0;
	movements.push_back(movement);
	
	movement.moveX = -50.0;
	movement.moveY = 0.0;
	movement.moveZ = -200.0;
	movements.push_back(movement);

	std::cout << "This node enables you to send hardcoded movement commands to a module. Please select the module:" << std::endl;
	
	std::string equipletName;
	std::cout << "equipletName: ";
	std::cout.flush();
	std::cin >> equipletName;
	
	std::string manufacturer;
	std::cout << "manufacturer: ";
	std::cout.flush();
	std::cin >> manufacturer;
	
	std::string typeNumber;
	std::cout << "typeNumber: ";
	std::cout.flush();
	std::cin >> typeNumber;
	
	std::string serialNumber;
	std::cout << "serialNumber: ";
	std::cout.flush();
	std::cin >> serialNumber;
	
	rexos_datatypes::ModuleIdentifier moduleIdentifier(manufacturer, typeNumber, serialNumber);
	
	HardcodedMoverNode HardcodedMoverNode(equipletName, moduleIdentifier, movements, false);
	return 0;
}
