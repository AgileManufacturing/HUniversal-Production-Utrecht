/* 
 * File:   EquipletStep.h
 * Author: alexander-ubuntu
 *
 * Created on June 16, 2013, 3:33 PM
 */
#ifndef EQUIPLETSTEP_H
#define	EQUIPLETSTEP_H

#include <string.h>
#include <rexos_datatypes/OriginPlacement.h>
#include <rexos_datatypes/ModuleIdentifier.h>

#include <jsoncpp/json/value.h>

namespace rexos_datatypes{
    
    class EquipletStep {
    public:
        EquipletStep();
        EquipletStep(Json::Value n);
        virtual ~EquipletStep();

        std::string getId();
        void setId(std::string id);

        rexos_datatypes::ModuleIdentifier getModuleIdentifier();
        
        void setModuleIdentifier(rexos_datatypes::ModuleIdentifier moduleIdentifier);
        void setModuleIdentifier(const Json::Value & n);

        OriginPlacement getOriginPlacement();
        void setOriginPlacement(OriginPlacement originPlacement);

        Json::Value getInstructionData();
        void setInstructionData(Json::Value instructionData);

        std::string getStatus();
        void setStatus(std::string status);

        /**
         * [getReloadEquiplet - WIP LARS]
         * @return [description]
         */
        std::string getReloadEquiplet();
        /**
         * [setReloadEquiplet - WIP LARS]
         * @param status [description]
         */
        void setReloadEquiplet(std::string status);

        Json::Value toJSON();
    private:
		std::string id;
		rexos_datatypes::ModuleIdentifier moduleIdentifier;
		Json::Value instructionData;
		OriginPlacement originPlacement;
		// TODO enum
		std::string status;
        std::string reloadEquiplet;
		void setValues(const Json::Value & n);
    };
}
#endif	/* EQUIPLETSTEP_H */

