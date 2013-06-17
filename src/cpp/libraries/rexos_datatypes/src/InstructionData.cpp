/* 
 * File:   InstructionData.cpp
 * Author: alexander-ubuntu
 * 
 * Created on June 16, 2013, 3:38 PM
 */

#include "rexos_datatypes/InstructionData.h"

namespace rexos_datatypes{
    InstructionData::InstructionData() {
    }

    std::string InstructionData::getCommand(){
        return this->command;
    }
    void InstructionData::setCommand(std::string command){
        this->command = command;
    }

    std::string InstructionData::getDestination(){
        return this->destination;
    }
    void InstructionData::setDestination(std::string destination){
        this->destination = destination;
    }

    std::string InstructionData::getLook_up(){
        return this->look_up;
    }
    void InstructionData::setLook_up(std::string look_up){
        this->look_up = look_up;
    }

    std::map<std::string, std::string> InstructionData::getLook_up_parameters(){
        return this->look_up_parameters;
    }
    void InstructionData::setLook_up_parameters(map<std::string, std::string> look_up_parameters){
        this->look_up_parameters = look_up_parameters;
    }

    std::map<std::string, std::string> InstructionData::getPayload(){
        return this->payload;    
    }
    void InstructionData::setPayload(std::map<std::string, std::string> payload){
        this->payload = payload;
    }
}