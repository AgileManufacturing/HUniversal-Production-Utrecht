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

// GCC system header to suppress libjson warnings
#pragma GCC system_header
#include <libjson/libjson.h>
 

using namespace std;

namespace rexos_datatypes{
    class InstructionData {
    public:
        InstructionData();  
        InstructionData(std::string command, std::string destination, std::string look_up, 
            std::map<std::string, std::string> look_up_parameters, std::map<std::string, std::string> payload); 

        std::string getCommand();
        void setCommand(std::string command);

        std::string getDestination();
        void setDestination(std::string destination);

        std::string getLook_up();
        void setLook_up(std::string look_up);

        std::map<std::string, std::string> getLook_up_parameters();
        void setLook_up_parameters(map<std::string, std::string> look_up_parameters);

        std::map<std::string, std::string> getPayload();
        void setPayload(map<std::string, std::string> payload);

        JSONNode getJsonNode();
        void setJsonNode(JSONNode jsonNode);

        std::string toJSONString();
    private:
        JSONNode jsonNode;
        std::string command;
        std::string destination;
        std::string look_up;
        std::map<std::string, std::string> look_up_parameters;
        std::map<std::string, std::string> payload;
        std::string mapToJsonString(std::map<std::string, std::string> map);
    };
}

#endif	/* INSTRUCTIONDATA_H */

