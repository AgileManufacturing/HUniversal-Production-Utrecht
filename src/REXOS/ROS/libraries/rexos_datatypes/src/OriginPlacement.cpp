 /**
 * @file InstructionData.cpp
 * @brief Represents the instructiondata of an equipletstep.
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

#include "rexos_datatypes/OriginPlacement.h"
#include "rexos_utilities/Utilities.h"

#include <jsoncpp/json/writer.h>

namespace rexos_datatypes{

    OriginPlacement::OriginPlacement(){}

    OriginPlacement::OriginPlacement(Json::Value n){
		setOriginPlacementType(n["originPlacementType"].asString());
		parameters = n["parameters"];
		lookupResult = n["lookupResult"];
    }

	OriginPlacement::OriginPlacementType OriginPlacement::getOriginPlacementType() {
		return originPlacementType;
	}
	std::string OriginPlacement::getOriginPlacementTypeAsString() {
		if(originPlacementType == OriginPlacementType::RELATIVE_TO_IDENTIFIER) {
			return "relativeToIdentifier";
		} else if(originPlacementType == OriginPlacementType::RELATIVE_TO_CURRENT_POSITION) {
			return "relativeToCurrentPosition";
		} else if(originPlacementType == OriginPlacementType::RELATIVE_TO_MODULE_ORIGIN) {
			return "relativeToModuleOrigin";
		} else if(originPlacementType == OriginPlacementType::RELATIVE_TO_EQUIPLET_ORIGIN) {
			return "relativeToEquipletOrigin";
		} else {
			throw std::runtime_error("OriginPlacementType is of unknown type");
		}
	}
	void OriginPlacement::setOriginPlacementType(OriginPlacementType type) {
		originPlacementType = type;
	}
	void OriginPlacement::setOriginPlacementType(std::string type) {
		if(type == "relativeToIdentifier") {
			originPlacementType = OriginPlacementType::RELATIVE_TO_IDENTIFIER;
		} else if (type == "relativeToCurrentPosition") {
			originPlacementType = OriginPlacementType::RELATIVE_TO_CURRENT_POSITION;
		} else if (type == "relativeToModuleOrigin") {
			originPlacementType = OriginPlacementType::RELATIVE_TO_MODULE_ORIGIN;
		} else if (type == "relativeToEquipletOrigin") {
			originPlacementType = OriginPlacementType::RELATIVE_TO_EQUIPLET_ORIGIN;
		}
	}

	Json::Value OriginPlacement::getLookupResult() {
		return lookupResult;
	}
	void OriginPlacement::setLookupResult(Json::Value result) {
		this->lookupResult = result;
	}
	
	Json::Value OriginPlacement::getParameters() {
		return parameters;
	}
	void OriginPlacement::setParameters(Json::Value parameters) {
		this->parameters = parameters;
	}
	
    Json::Value OriginPlacement::toJSON(){
		Json::Value output;
		output["originPlacementType"] = getOriginPlacementTypeAsString();
		output["parameters"] = parameters;
		output["lookupResult"] = lookupResult;
		return output;
    }
}