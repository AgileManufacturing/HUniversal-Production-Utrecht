#pragma once

#include <string.h>
#include <rexos_datatypes/OriginPlacement.h>
#include <rexos_datatypes/ModuleIdentifier.h>

#include <jsoncpp/json/value.h>

namespace rexos_datatypes {
	class EquipletCommand {
	public:
		enum Status {
			WAITING, IN_PROGRESS, DONE, FAILED, COUNT
		};
		static const char* const statusTxt[];
		
		EquipletCommand();
		EquipletCommand(Json::Value n);
		
		std::string getId();
		void setId(std::string id);
		
		std::string getCommand();
		void setCommand(std::string command);
		
		Json::Value getParameters();
		void setParameters(Json::Value parameters);
		
		Status getStatus();
		std::string getStatusAsString();
		void setStatus(Status status);
		void setStatus(std::string status);
		
		Json::Value toJSON();
	private:
		std::string id;
		std::string command;
		Json::Value parameters;
		Status status;
	};
}
