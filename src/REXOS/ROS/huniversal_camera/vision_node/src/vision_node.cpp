/**
 * @file CameraNode.cpp
 * @brief Camera node
 * @date Created: 2013-10-23
 * 
 * @author Tommas Bakker
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
 *
 **/

#include "vision_node/VisionNode.h"
#include "rexos_utilities/Utilities.h"

#include <camera/unicap_cv_bridge.h>

#include <stdexcept>

int main(int argc, char* argv[]) {
	REXOS_INFO("This node must be run with ROOT priveleges or the device must be added to the correct group when using the DFK 22AUC03");

	if(argc < 7){
		REXOS_ERROR("Usage: part_locator_node (--isSimulated | --isShadow) equipletName manufacturer typeNumber serialNumber deviceNumber formatNumber");
		return -1;
	}
	int argumentOffset = 1;
	
	bool isSimulated = false;
	bool isShadow = false;
	
	for (int i = 0; i < argc; i++) {
		std::string arg = argv[i];
		if (arg == "--isSimulated") {
			isSimulated = true;
			argumentOffset++;
		} else if (arg == "--isShadow") {
			isShadow = true;
			isSimulated = true;
			argumentOffset++;
		}
	}
	
	
	std::vector<std::string> devices;
	devices = camera::unicap_cv_bridge::listDevices();
	REXOS_INFO_STREAM("Devices:" << std::endl);
	for(uint i = 0; i < devices.size(); i++){
		REXOS_INFO_STREAM("\t" << devices[i] << std::endl);
	}
	
	int deviceNumber;
	try{
		deviceNumber = rexos_utilities::stringToInt(argv[argumentOffset + 4]);
	} catch(std::runtime_error ex){
		REXOS_ERROR("Device number is not a valid number.");
		exit(-2);
	}
	REXOS_WARN_STREAM("deviceNumber" << deviceNumber);
	
	try{
		std::vector<std::string> formats;
		formats = camera::unicap_cv_bridge::listFormats(deviceNumber);
		REXOS_INFO_STREAM("Formats:" << std::endl);
		for(uint i = 0; i < formats.size(); i++){
			REXOS_INFO_STREAM("\t" << formats[i] << std::endl);
		}
	} catch (camera::unicap_cv_bridge::UnicapCvException ex) {
		// ignore
	}

	int formatNumber;
	try{
		formatNumber = rexos_utilities::stringToInt(argv[argumentOffset + 5]);
	} catch(std::runtime_error ex){
		REXOS_ERROR("Format number is not a valid number.");
		exit(-3);
	}
	REXOS_WARN_STREAM("formatNumber" << formatNumber);
	
	std::string equipletName = std::string(argv[argumentOffset + 0]);
	rexos_datatypes::ModuleIdentifier moduleIdentifier(argv[argumentOffset + 1], argv[argumentOffset + 2], "1");
	rexos_datatypes::ModuleIdentifier moduleIdentifier2(argv[argumentOffset + 1], argv[argumentOffset + 2], "2");
	std::vector<rexos_datatypes::ModuleIdentifier> cameraModules = {moduleIdentifier,moduleIdentifier2};
	// set up node namespace and name
	if(isShadow == true) {
		if(setenv("ROS_NAMESPACE", "shadow", 1) != 0) {
			REXOS_ERROR("Unable to set environment variable");
		}
	}
	std::string nodeName = equipletName + "_" + moduleIdentifier.getManufacturer() + "_" + 
			moduleIdentifier.getTypeNumber() + "_" + moduleIdentifier.getSerialNumber();
	ros::init(argc, argv, nodeName);
	
	REXOS_WARN_STREAM(nodeName);

	
	VisionNode visionNode(equipletName, cameraModules, isSimulated, deviceNumber, formatNumber);
	visionNode.run();

	return 0;
}
