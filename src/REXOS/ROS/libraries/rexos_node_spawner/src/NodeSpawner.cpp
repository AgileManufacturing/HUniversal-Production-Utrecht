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
#include <iostream>
#include <fstream>
#include <unistd.h>

#include <boost/filesystem/operations.hpp>
#include <boost/algorithm/string/replace.hpp>

#include "ros/ros.h"

namespace rexos_node_spawner {
	NodeSpawner::NodeSpawner(std::string equipletName) :
		equipletName(equipletName)
	{
		
	}
	void NodeSpawner::spawnNode(rexos_knowledge_database::ModuleIdentifier moduleIdentifier) {
		ROS_INFO("Spawning new node");
		int pid = fork();
		if(pid == 0) {
			// we are the new child
			rexos_knowledge_database::RosSoftware rosSoftware = rexos_knowledge_database::RosSoftware(moduleIdentifier);
			extractZipArchive(rosSoftware);
			
			// start the new node
			std::string command = rosSoftware.getCommand();
			boost::algorithm::replace_all(command, "{equipletName}", equipletName);
			boost::algorithm::replace_all(command, "{manufacturer}", moduleIdentifier.getManufacturer());
			boost::algorithm::replace_all(command, "{typeNumber}", moduleIdentifier.getTypeNumber());
			boost::algorithm::replace_all(command, "{serialNumber}", moduleIdentifier.getSerialNumber());
		
			ROS_INFO_STREAM("Spawning node with command " << rosSoftware.getCommand());
			execl("/bin/bash", "/bin/bash", "-c", command.c_str(), NULL);
			ROS_ERROR("Unable to execl");
			throw std::runtime_error("Unable to execl");
		} else {
			// we are the old parent
			ROS_INFO("node has been spawned");
		}
	}
	void NodeSpawner::spawnEquipletNode() {
		ROS_INFO("Spawning new node");
		int pid = fork();
		if(pid == 0) {
			// we are the new child
			rexos_knowledge_database::RosSoftware rosSoftware = rexos_knowledge_database::RosSoftware(equipletName);
			extractZipArchive(rosSoftware);
			
			// start the new node
			std::string command = rosSoftware.getCommand();
			boost::algorithm::replace_all(command, "{equipletName}", equipletName);
			
			ROS_INFO_STREAM("Spawning node with command " << rosSoftware.getCommand());
			execl("/bin/bash", "/bin/bash", "-c", command.c_str(), NULL);
			ROS_ERROR("Unable to execl");
			throw std::runtime_error("Unable to execl");
		} else {
			// we are the old parent
			ROS_INFO("node has been spawned");
		}
	}
	void NodeSpawner::extractZipArchive(rexos_knowledge_database::RosSoftware& rosSoftware) {
		char buf[100];
			
		std::istream* rosFile = rosSoftware.getRosFile();
		std::ofstream zipFileOutputStream;
		std::string zipArchiveFileName = boost::lexical_cast<std::string>(rosSoftware.getId()) + ".zip";
		std::string zipArchivePath = std::string("/tmp/rexos_node_spawner/");
		zipFileOutputStream.open(zipArchivePath + zipArchiveFileName, 
				std::ios::out | std::ios::binary); 
		
		while(rosFile->good() == true) {
			rosFile->read(buf, sizeof(buf));
			zipFileOutputStream.write(buf, rosFile->gcount());
		}
		zipFileOutputStream.close();
		ROS_INFO_STREAM("zip archive has been written at " << zipArchiveFileName);
		
		int err = 0;
		zip* zipArchive = zip_open((zipArchivePath + zipArchiveFileName).c_str(), 0, &err);
		ROS_INFO_STREAM("zip archive opened with " << err);
		
		struct zip_stat zipStat;
		for (int i = 0; i < zip_get_num_entries(zipArchive, 0); i++) {
			if (zip_stat_index(zipArchive, i, 0, &zipStat) == 0) {
				// is directory or file entry?
				if (zipStat.name[strlen(zipStat.name) - 1] == '/') {
					boost::filesystem::create_directories(zipStat.name);
				} else {
					// files could be specified before the upper directories. create these directories
					boost::filesystem3::path path = boost::filesystem3::path(zipStat.name);
					boost::filesystem::create_directories(path.parent_path());
					
					struct zip_file* zipFile;
					zipFile = zip_fopen_index(zipArchive, i, 0);
					if (!zipFile) {
						throw std::runtime_error("Unable to open zipFile in zipArchive");
					}
					
					std::ofstream fs;
					fs.open((zipArchivePath + std::string(zipStat.name)).c_str(), std::ios::out | std::ios::binary); 
					if (fs.good() != true) {
						throw std::runtime_error("Unable to open fstream");
					}
					
					int sum = 0;
					while (sum != zipStat.size) {
						int len = zip_fread(zipFile, buf, 100);
						fs.write(buf, sizeof(buf));
						sum += len;
					}
					
					fs.close();
					zip_fclose(zipFile);
				}
			}
		}
		ROS_INFO("zipArchive has been extracted");
	}
}
