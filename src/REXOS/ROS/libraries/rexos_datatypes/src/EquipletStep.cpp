/**
 * @file EquipletStep.cpp
 * @brief Container class of an equipletstep. 
 * @date Created: 2012-09-19
 *
 * @author Alexander Streng
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

#include "rexos_datatypes/EquipletStep.h"
#include "rexos_utilities/Utilities.h"

#include <jsoncpp/json/writer.h>

namespace rexos_datatypes{

    EquipletStep::EquipletStep(Json::Value n) :
			moduleIdentifier("", "", "") // shut up the complaining compiler. TODO: nicer solution
	{
		setValues(n);
    }

    std::string EquipletStep::getId(){
        return this->id;
    }

    void EquipletStep::setId(std::string id){
        this->id = id;    
    }

    rexos_knowledge_database::ModuleIdentifier EquipletStep::getModuleIdentifier(){
        return this->moduleIdentifier;
    }

    void EquipletStep::setModuleIdentifier(rexos_knowledge_database::ModuleIdentifier moduleIdentifier){
        this->moduleIdentifier = moduleIdentifier;
    }

    void EquipletStep::setModuleIdentifier(const Json::Value & n){
		std::string manufacturer = n["manufacturer"].asString();
		std::string typeNumber = n["typeNumber"].asString();
		std::string serialNumber = n["serialNumber"].asString();
       this->moduleIdentifier = rexos_knowledge_database::ModuleIdentifier(manufacturer, typeNumber, serialNumber);
	 }

    InstructionData EquipletStep::getInstructionData(){
        return this->instructionData;
    }

    void EquipletStep::setInstructionData(InstructionData instructionData){
        this->instructionData = instructionData;
    }

    std::string EquipletStep::getStatus(){
        return this->status;
    }

    void EquipletStep::setStatus(std::string status){
        this->status = status;
    }

    EquipletStep::~EquipletStep() {
        //std::cout << "Delete Equipletstep called." std::endl;
    }

    Json::Value EquipletStep::getJsonNode(){
        return this->jsonNode;
    }
    
    void EquipletStep::setValues(const Json::Value & n){
		setModuleIdentifier(n["moduleIdentifier"]);
		setInstructionData(InstructionData(n["instructionData"]));
		setStatus(n["status"].asString());
    }

    std::string EquipletStep::toJSONString(){
		Json::Value output;
		output["moduleIdentifier"] = moduleIdentifier.toString();
		output["instructionData"] = instructionData.getJsonNode();
		output["status"] = status;
		
		Json::StyledWriter styledWriter;
		return styledWriter.write(output);
    }
}
