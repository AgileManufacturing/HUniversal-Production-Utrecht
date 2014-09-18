/* 
 * File:   EquipletStep.h
 * Author: alexander-ubuntu
 *
 * Created on June 16, 2013, 3:33 PM
 */
#ifndef EQUIPLETSTEP_H
#define	EQUIPLETSTEP_H

#include <string.h>
#include <iostream>
#include <map>
#include <utility>
#include "rexos_datatypes/InstructionData.h"
#include "rexos_datatypes/OriginPlacement.h"
#include "rexos_datatypes/TimeData.h"

#include <rexos_knowledge_database/ModuleIdentifier.h>

#include <jsoncpp/json/value.h>

namespace rexos_datatypes{
    
    class EquipletStep {
    public:
        EquipletStep(Json::Value n);
        virtual ~EquipletStep();

        std::string getId();
        void setId(std::string id);

        rexos_knowledge_database::ModuleIdentifier getModuleIdentifier();
		 void setModuleIdentifier(rexos_knowledge_database::ModuleIdentifier moduleIdentifier);
		 void setModuleIdentifier(const Json::Value & n);

        OriginPlacement getOriginPlacement();
        void setOriginPlacement(OriginPlacement originPlacement);

        Json::Value getInstructionData();
        void setInstructionData(Json::Value instructionData);

        std::string getStatus();
        void setStatus(std::string status);

        Json::Value toJSON();
    private:
		std::string id;
		rexos_knowledge_database::ModuleIdentifier moduleIdentifier;
		Json::Value instructionData;
		OriginPlacement originPlacement;
		// TODO enum
		std::string status;
		void setValues(const Json::Value & n);
    };
}
#endif	/* EQUIPLETSTEP_H */

