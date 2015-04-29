#include "rexos_datatypes/HardwareStep.h"
#include "rexos_utilities/Utilities.h"

#include <jsoncpp/json/writer.h>

namespace rexos_datatypes{
	const char* const HardwareStep::statusTxt[] = {"WAITING", "IN_PROGRESS", "DONE", "FAILED"};

	HardwareStep::HardwareStep() :
			moduleIdentifier("", "", "") // shut up the complaining compiler. TODO: nicer solution
	{
	}

	HardwareStep::HardwareStep(Json::Value n) :
			moduleIdentifier("", "", "") // shut up the complaining compiler. TODO: nicer solution
	{
		setModuleIdentifier(n["moduleIdentifier"]);
		setInstructionData(n["instructionData"]);
		setOriginPlacement(OriginPlacement(n["originPlacement"]));
		setStatus(n["status"].asString());
	}

	std::string HardwareStep::getId(){
		return this->id;
	}

	void HardwareStep::setId(std::string id){
		this->id = id;
	}

	rexos_datatypes::ModuleIdentifier HardwareStep::getModuleIdentifier(){
		return this->moduleIdentifier;
	}

	void HardwareStep::setModuleIdentifier(rexos_datatypes::ModuleIdentifier moduleIdentifier){
		this->moduleIdentifier = moduleIdentifier;
	}

	void HardwareStep::setModuleIdentifier(const Json::Value & n){
		std::string manufacturer = n["manufacturer"].asString();
		std::string typeNumber = n["typeNumber"].asString();
		std::string serialNumber = n["serialNumber"].asString();
	   this->moduleIdentifier = rexos_datatypes::ModuleIdentifier(manufacturer, typeNumber, serialNumber);
	 }

	Json::Value HardwareStep::getInstructionData(){
		return this->instructionData;
	}

	void HardwareStep::setInstructionData(Json::Value instructionData){
		this->instructionData = instructionData;
	}
	
	OriginPlacement HardwareStep::getOriginPlacement(){
		return this->originPlacement;
	}
	
	void HardwareStep::setOriginPlacement(OriginPlacement originPlacement){
		this->originPlacement = originPlacement;
	}
	
	HardwareStep::Status HardwareStep::getStatus(){
		return this->status;
	}
	
	std::string HardwareStep::getStatusAsString() {
		return statusTxt[status];
	}

	void HardwareStep::setStatus(HardwareStep::Status status){
		this->status = status;
	}
	void HardwareStep::setStatus(std::string status) {
		for(int i = 0; i < Status::COUNT; i++) {
			if(status == statusTxt[i]) {
				status = static_cast<Status>(i);
				return;
			}
		}
		throw std::runtime_error("HardwareStep::Status is of unknown type");
	}

	Json::Value HardwareStep::toJSON(){
		Json::Value output;
		output["moduleIdentifier"] = moduleIdentifier.toJSONObject();
		output["instructionData"] = instructionData;
		output["originPlacement"] = originPlacement.toJSON();
		output["status"] = status;
		return output;
	}
}
