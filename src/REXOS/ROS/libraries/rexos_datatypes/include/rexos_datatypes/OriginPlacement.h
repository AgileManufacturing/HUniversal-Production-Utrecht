/* 
 * File:   InstructionData.h
 * Author: alexander-ubuntu
 *
 * Created on June 16, 2013, 3:38 PM
 */

#pragma once

#include <string.h>
#include <iostream>
#include <map>
#include <utility>

#include <jsoncpp/json/value.h>
 

using namespace std;

namespace rexos_datatypes {
	class OriginPlacement {
	public:
		enum OriginPlacementType {
			RELATIVE_TO_IDENTIFIER,
			RELATIVE_TO_CURRENT_POSITION,
			RELATIVE_TO_MODULE_ORIGIN,
			RELATIVE_TO_EQUIPLET_ORIGIN,
			UNDEFINED,
			COUNT
		};
		static const char* const originPlacementTypeTxt[];
		
		OriginPlacement(); 
		OriginPlacement(Json::Value n); 
		
		OriginPlacementType getOriginPlacementType();
		std::string getOriginPlacementTypeAsString();
		void setOriginPlacementType(OriginPlacementType type);
		void setOriginPlacementType(std::string type);

		Json::Value getLookupResult();
		void setLookupResult(Json::Value jsonNode);

		Json::Value getParameters();
		void setParameters(Json::Value parameters);

		Json::Value toJSON();
	private:
		Json::Value parameters;
		Json::Value lookupResult;
		OriginPlacementType originPlacementType;
	};
}
