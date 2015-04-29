#pragma once

#include <string.h>
#include <rexos_datatypes/OriginPlacement.h>
#include <rexos_datatypes/ModuleIdentifier.h>

#include <jsoncpp/json/value.h>

namespace rexos_datatypes {
	class HardwareStep {
	public:
		enum Status {
			WAITING, IN_PROGRESS, DONE, FAILED, COUNT
		};
		static const char* const statusTxt[];
		
		HardwareStep();
		HardwareStep(Json::Value n);
		
		std::string getId();
		void setId(std::string id);
		
		rexos_datatypes::ModuleIdentifier getModuleIdentifier();
		
		void setModuleIdentifier(rexos_datatypes::ModuleIdentifier moduleIdentifier);
		void setModuleIdentifier(const Json::Value & n);
		
		OriginPlacement getOriginPlacement();
		void setOriginPlacement(OriginPlacement originPlacement);
		
		Json::Value getInstructionData();
		void setInstructionData(Json::Value instructionData);
		
		Status getStatus();
		std::string getStatusAsString();
		void setStatus(Status status);
		void setStatus(std::string status);
		
		Json::Value toJSON();
	private:
		std::string id;
		rexos_datatypes::ModuleIdentifier moduleIdentifier;
		Json::Value instructionData;
		OriginPlacement originPlacement;
		Status status;
	};
}
