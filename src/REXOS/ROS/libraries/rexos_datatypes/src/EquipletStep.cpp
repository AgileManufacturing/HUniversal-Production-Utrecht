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

namespace rexos_datatypes{

    EquipletStep::EquipletStep(JSONNode n) :
			moduleIdentifier("", "", "") // shut up the complaining compiler. TODO: nicer solution
	{
		setValues(n);
    }

    std::string EquipletStep::getId(){
        return this->_id;
    }

    void EquipletStep::setId(std::string id){
        this->_id = id;    
    }

    std::string EquipletStep::getServiceStepID(){
        return this->serviceStepID;
    }

    void EquipletStep::setServiceStepID(std::string serviceStepID){
        this->serviceStepID = serviceStepID;
    }

    std::string EquipletStep::getNextStep(){
        return this->nextStep;
    }

    void EquipletStep::setNextStep(std::string nextStep){
        this->nextStep = nextStep;
    }

    rexos_knowledge_database::ModuleIdentifier EquipletStep::getModuleIdentifier(){
        return this->moduleIdentifier;
    }

    void EquipletStep::setModuleIdentifier(rexos_knowledge_database::ModuleIdentifier moduleIdentifier){
        this->moduleIdentifier = moduleIdentifier;
    }

    void EquipletStep::setModuleIdentifier(const JSONNode & n){
		std::string manufacturer;
		std::string typeNumber;
		std::string serialNumber;
		
       //Iterate them nodes.
       JSONNode::const_iterator i = n.begin();

       while (i != n.end()){
			const char * node_name = i -> name().c_str();
			
           if (strcmp(node_name, "manufacturer") == 0){
               manufacturer = i -> as_string();
           } else if (strcmp(node_name, "typeNumber") == 0){
               typeNumber = i -> as_string();
           } else if (strcmp(node_name, "serialNumber") == 0){
               serialNumber = i -> as_string();
           }
		    i++;
	   }
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

    std::map<std::string, std::string> EquipletStep::getStatusData(){
        return this->statusData;
    }

    void EquipletStep::setStatusData(std::map<std::string, std::string> statusData){
        this->statusData = statusData;
    }

    TimeData EquipletStep::getTimeData(){
        return this->timeData;
    }
    
    void EquipletStep::setTimeData(TimeData timeData){
        this->timeData = timeData;
    }

    EquipletStep::~EquipletStep() {
        //std::cout << "Delete Equipletstep called." std::endl;
    }

    JSONNode EquipletStep::getJsonNode(){
        return this-> jsonNode;
    }
    
    void EquipletStep::setValues(const JSONNode & n){
        //Iterate them nodes.
        JSONNode::const_iterator i = n.begin();

        while (i != n.end()){
            
            const char * node_name = i -> name().c_str();
            
            if (strcmp(node_name, "serviceStepID") == 0){
                setServiceStepID(i -> as_string());
            }
            else if (strcmp(node_name, "nextStep") == 0){
                setNextStep(i -> as_string());
            }
            else if (strcmp(node_name, "moduleIdentifier") == 0){
                setModuleIdentifier(i -> as_node());
            }
            else if (strcmp(node_name, "instructionData") == 0){
                setInstructionData(InstructionData(*i));
            }
            else if (strcmp(node_name, "status") == 0){
                setStatus(i -> as_string());
            }
            else if (strcmp(node_name, "statusData") == 0){
                setStatusData(rexos_utilities::setMapFromNode(*i));
            }
            else if (strcmp(node_name, "timeData") == 0){
                setTimeData(setTimeDataFromNode(*i));
            }
            ++i;
        }        
    }

    std::string EquipletStep::toJSONString(){

        std::stringstream ss;

        ss << "{ ";
        ss << "\"serviceStepID\" : \"" << this->serviceStepID << "\", ";
        ss << "\"nextStep\" : \"" << this->nextStep << "\", ";
        ss << "\"moduleIdentifier\" : " << this->moduleIdentifier.toString() << ", ";
        ss << "\"instructionData\" : " << this->instructionData.toJSONString() << ", ";
        ss << "\"status\" : \"" << this->status << "\" ";
        ss << " } ";

        return ss.str();
    }
    
    TimeData EquipletStep::setTimeDataFromNode(const JSONNode & n){
        TimeData * timeData = new TimeData();
        //For now its only 1 value. This might be more in the future
        JSONNode::const_iterator i = n.begin();
        
        while (i != n.end()){
            timeData->setDuration(i-> as_int());
            i++;
        }
        return *timeData;
    }
   
}
