/* 
 * File:   InstructionData.h
 * Author: alexander-ubuntu
 *
 * Created on June 16, 2013, 3:38 PM
 */

#ifndef INSTRUCTIONDATA_H
#define	INSTRUCTIONDATA_H


#include <string.h>
#include <iostream>
#include <map>
#include <utility>

#include <jsoncpp/json/value.h>
 

using namespace std;

namespace rexos_datatypes{
	class InstructionData {
	public:
		InstructionData();
		InstructionData(Json::Value n); 
		
		Json::Value getJsonNode();
		void setJsonNode(Json::Value jsonNode);

		std::string toJSONString();
	private:
		Json::Value jsonNode;
	};
}

#endif	/* INSTRUCTIONDATA_H */

