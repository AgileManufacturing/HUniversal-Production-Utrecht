/**
 * @file DatabaseConnection.h
 * @brief Coordinate system for communication between nodes
 * @date Created: 2012-01-??  TODO: Date
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

#pragma once

#include <string>
#include <vector>

#include <rexos_knowledge_database/ModuleType.h>

#include "mysql_connection.h"

namespace rexos_knowledge_database {
	class Module{
	public:
		Module(std::string manufacturer, std::string typeNumber, std::string serialNumber);
		
		ModuleType* getModuleType();
		std::string getModuleProperties();
		void setModuleProperties(std::string jsonProperties);
		Module* getParentModule();
		std::vector<Module*> getChildModules();
		
		std::string getCalibrationDataForModuleOnly();
		std::string getCalibrationDataForModuleAndChilds();
		std::string getCalibrationDataForModuleAndOtherModules(std::vector<Module*> modules);
		void setCalibrationDataForModuleOnly(std::string properties);
		void setCalibrationDataForModuleAndChilds(std::string properties);
		void setCalibrationDataForModuleAndOtherModules(std::vector<Module*> modules, std::string properties);
	private:
		int getCalibrationGroupForModuleAndOtherModules(std::vector<Module*> modules);
		
	private:
		std::string manufacturer, typeNumber, serialNumber;
		std::auto_ptr<sql::Connection> connection;
	};
}