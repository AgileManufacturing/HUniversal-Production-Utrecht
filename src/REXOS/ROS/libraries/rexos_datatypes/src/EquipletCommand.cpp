#include <rexos_datatypes/EquipletCommand.h>
namespace rexos_datatypes {
	const char* const EquipletCommand::statusTxt[] = {"WAITING", "IN_PROGRESS", "DONE", "FAILED"};
	EquipletCommand::EquipletCommand() {
	}
	EquipletCommand::EquipletCommand(Json::Value n) {
		setCommand(n["command"].asString());
		setParameters(n["parameters"]);
		setStatus(n["status"].asString());
	}
		
	std::string EquipletCommand::getId(){
		return this->id;
	}

	void EquipletCommand::setId(std::string id){
		this->id = id;	
	}

	std::string EquipletCommand::getCommand() {
		return this->command;
	}
	
	void EquipletCommand::setCommand(std::string command) {
		this->command = command;
	}
	
	Json::Value EquipletCommand::getParameters(){
		return this->parameters;
	}

	void EquipletCommand::setParameters(Json::Value parameters) {
		this->parameters = parameters;
	}
	
	EquipletCommand::Status EquipletCommand::getStatus(){
		return this->status;
	}

	std::string EquipletCommand::getStatusAsString() {
		return statusTxt[status];
	}

	void EquipletCommand::setStatus(EquipletCommand::Status status){
		this->status = status;
	}
	void EquipletCommand::setStatus(std::string status) {
		for(int i = 0; i < Status::COUNT; i++) {
			if(status == statusTxt[i]) {
				status = static_cast<Status>(i);
				return;
			}
		}
		throw std::runtime_error("EquipletCommand::Status is of unknown type");
	}
	
	Json::Value EquipletCommand::toJSON(){
		Json::Value output;
		output["command"] = command;
		output["parameters"] = parameters;
		output["status"] = status;
		return output;
	}
}
