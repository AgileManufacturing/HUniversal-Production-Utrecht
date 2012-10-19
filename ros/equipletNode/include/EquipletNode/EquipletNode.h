/**
* @file EquipletNode.h
* @brief Symbolizes an entire EquipletNode.
* @date Created: 2012-10-12
*
* @author Dennis Koole
*
* @section LICENSE
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

#pragma once

#include <string>
#include <vector>
#include <Mast/HardwareModuleProperties.h>
#include "rosMast/States.h"
#include <BlackboardClient/BlackboardClientUtils.h>
#include <BlackboardClient/PostIt.pb.h>


class EquipletNode {
public:
	EquipletNode(int id = 1);

	virtual ~EquipletNode();

	bool addHardwareModule(Mast::HardwareModuleProperties module);
	bool removeHardwareModule(int id);
	void updateOperationState();
	void updateSafetyState();
	bool updateModuleState(int moduleID, rosMast::StateType state);
	void printHardwareModules();
	void readFromBlackboard();
private:
	/**
	 * @var int equipletId
	 * The id of the equiplet
	 **/
	int equipletId;

	/**
	 * @var Mast::state operationState
	 * The minimal operation state is equal to the lowest state of all modules that are actors
	 **/
	rosMast::StateType operationState;
	/**
	 * @var Mast::state safetyState
	 * The safety state of the Equiplet. This is equal to the highest state of the actor modules
	 **/
	rosMast::StateType safetyState;
	/**
	 * @var std::vector<Mast::HardwareModuleProperties> moduleTable
	 * The table that holds all information about the modules currently attached to this Equiplet  
	 **/
	std::vector<Mast::HardwareModuleProperties> moduleTable;
	/**
	 *
	 *
	 **/
	BlackboardClient::BlackboardClientUtils bbUtils;
	/**
	 * @var PostItBox postItBox
	 * The postIt box where the messages are stored that are read from the blackboard
	 **/
	PostItBox *postItBox;
	/**
	 * @var std::map<int, pair> modulePackageNodeMap
	 * A map with the moduleType as key and a pair of package name and node name as value.
	 * This is used to find the name of the node that has to be started when a
	 * module is added, and the package name where the node can be find. This is a TEMPORARY!!
	 * solution. Better is to store this in some kind of database.
	 **/
	 std::map<int, std::pair< std::string, std::string> > modulePackageNodeMap;
};