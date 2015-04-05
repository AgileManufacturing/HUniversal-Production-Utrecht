/**
 * @file StepperMotorProperties.cpp
 * @brief Contains the properties of a stepper motor.
 * @date Created: 2012-10-02
 *
 * @author Tommas Bakker
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

#include <rexos_node_spawner/NodeSpawner.h>

#include <unistd.h>
#include <zip.h>
#include <fstream>
#include <unistd.h>

#include <boost/filesystem/operations.hpp>
#include <boost/algorithm/string/replace.hpp>

#include <rexos_zip/ZipExtractor.h>

#include "ros/ros.h"

namespace rexos_node_spawner {
	NodeSpawner::NodeSpawner(std::string equipletName, bool isSimulated, bool isShadow) :
		equipletName(equipletName),
		isSimulated(isSimulated),
		isShadow(isShadow)
	{
		
	}
	void NodeSpawner::spawnNode(rexos_datatypes::ModuleIdentifier moduleIdentifier) {
		int pid = fork();
		if(pid == 0) {
			// we are the new child
			rexos_knowledge_database::RosSoftware rosSoftware = rexos_knowledge_database::RosSoftware(moduleIdentifier);
			extractRosSoftware(rosSoftware);
			
			// start the new node
			std::string command = rosSoftware.getCommand();
			std::string baseDir = ZIP_ARCHIVE_PATH + boost::lexical_cast<std::string>(rosSoftware.getId()) + "/";
			boost::algorithm::replace_all(command, "{baseDir}", baseDir);
			if(isSimulated == true) {
				boost::algorithm::replace_all(command, "{isSimulated}", "--isSimulated");
			} else {
				boost::algorithm::replace_all(command, "{isSimulated}", "");
			}
			if(isShadow == true) {
				boost::algorithm::replace_all(command, "{isShadow}", "--isShadow");
			} else {
				boost::algorithm::replace_all(command, "{isShadow}", "");
			}
			
			boost::algorithm::replace_all(command, "{equipletName}", equipletName);
			boost::algorithm::replace_all(command, "{manufacturer}", moduleIdentifier.getManufacturer());
			boost::algorithm::replace_all(command, "{typeNumber}", moduleIdentifier.getTypeNumber());
			boost::algorithm::replace_all(command, "{serialNumber}", moduleIdentifier.getSerialNumber());
		
			REXOS_INFO_STREAM("Spawning node with command " << rosSoftware.getCommand());
			execl("/bin/bash", "/bin/bash", "-c", command.c_str(), NULL);
			REXOS_ERROR("Unable to execl");
			throw std::runtime_error("Unable to execl");
		} else {
			// we are the old parent
		}
	}
	void NodeSpawner::spawnEquipletNode() {
		int pid = fork();
		if(pid == 0) {
			// we are the new child
			rexos_knowledge_database::RosSoftware rosSoftware = rexos_knowledge_database::RosSoftware(equipletName);
			extractRosSoftware(rosSoftware);
			
			// start the new node
			std::string command = rosSoftware.getCommand();
			std::string baseDir = ZIP_ARCHIVE_PATH + boost::lexical_cast<std::string>(rosSoftware.getId()) + "/";
			boost::algorithm::replace_all(command, "{baseDir}", baseDir);
			if(isSimulated == true) {
				boost::algorithm::replace_all(command, "{isSimulated}", "--isSimulated");
			} else {
				boost::algorithm::replace_all(command, "{isSimulated}", "");
			}
			if(isShadow == true) {
				boost::algorithm::replace_all(command, "{isShadow}", "--isShadow");
			} else {
				boost::algorithm::replace_all(command, "{isShadow}", "");
			}
			
			boost::algorithm::replace_all(command, "{equipletName}", equipletName);
			
			REXOS_INFO_STREAM("Spawning node with command " << rosSoftware.getCommand());
			execl("/bin/bash", "/bin/bash", "-c", command.c_str(), NULL);
			REXOS_ERROR("Unable to execl");
			throw std::runtime_error("Unable to execl");
		} else {
			// we are the old parent
			REXOS_INFO("node has been spawned");
		}
	}
	void NodeSpawner::extractRosSoftware(rexos_knowledge_database::RosSoftware& rosSoftware) {
		std::string baseName = boost::lexical_cast<std::string>(rosSoftware.getId());
		rexos_zip::ZipExtractor::extractZipArchive(rosSoftware.getRosFile(), baseName, boost::filesystem::path(ZIP_ARCHIVE_PATH));
	}
}
