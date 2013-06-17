/* 
 * File:   EquipletStep.cpp
 * Author: alexander-ubuntu
 * 
 * Created on June 16, 2013, 3:33 PM
 */

#include "rexos_datatypes/EquipletStep.h"

namespace rexos_datatypes{

    std::string EquipletStep::getId(){
        return this->_id;
    }
    void EquipletStep::setId(std::string id){
        this->_id = id;    
    }

    std::string EquipletStep::getServiceStepID(){
        return this->serviceStepID;
    }
    void EquipletStep::setServiceStepID(std::string serviceStepID){
        this->serviceStepID = serviceStepID;
    }

    std::string EquipletStep::getNextStep(){
        return this->nextStep;
    }
    void EquipletStep::setNextStep(std::string nextStep){
        this->nextStep = nextStep;
    }

    int EquipletStep::getModuleId(){
        return this->moduleId;
    }
    void EquipletStep::setModuleId(int id){
        this->moduleId = id;
    }

    InstructionData EquipletStep::getInstructionData(){
        return this->instructionData;
    }
    void EquipletStep::setInstructionData(InstructionData instructionData){
        this->instructionData = instructionData;
    }

    int EquipletStep::getStatus(){
        return this->status;
    }
    void EquipletStep::setStatus(int status){
        this->status = status;
    }

    std::map<std::string, std::string> EquipletStep::getStatusData(){
        return this->statusData;
    }
    void EquipletStep::setStatusData(std::map<std::string, std::string> statusData){
        this->statusData = statusData;
    }

    TimeData EquipletStep::getTimeData(){
        return this->timeData;
    }
    void EquipletStep::setTimeData(TimeData timeData){
        this->timeData = timeData;
    }
    
    EquipletStep::EquipletStep(JSONNode n) {
        std::cout << "Equipletstep called." << std::endl;
        setValues(n);
    }

    EquipletStep::~EquipletStep() {
        //std::cout << "Delete Equipletstep called." std::endl;
    }
    
    void EquipletStep::setValues(const JSONNode & n){
        //Iterate them nodes.
        JSONNode::const_iterator i = n.begin();

        while (i != n.end()){
            
            const char * node_name = i -> name().c_str();
            
            if (strcmp(node_name, "serviceStepID") == 0){
                setServiceStepID(i -> as_string());
            }
            else if (strcmp(node_name, "nextStep") == 0){
                setNextStep(i -> as_string());
            }
            else if (strcmp(node_name, "moduleId") == 0){
                setModuleId(i -> as_int());
            }
            else if (strcmp(node_name, "instructionData") == 0){
                setInstructionData(setInstructionDataFromNode(*i));
            }
            else if (strcmp(node_name, "status") == 0){
                setStatus(i -> as_int());
            }
            else if (strcmp(node_name, "statusData") == 0){
                setStatusData(setMapFromNode(*i));
            }
            else if (strcmp(node_name, "timeData") == 0){
                setTimeData(setTimeDataFromNode(*i));
            }
            //increment the iterator
            ++i;
        }
        //first check if we got an instructiondata node
        
    }
    
    InstructionData EquipletStep::setInstructionDataFromNode(const JSONNode & n){
         //Iterate them nodes.
        JSONNode::const_iterator i = n.begin();
        InstructionData * instructData = new InstructionData();
        
        while (i != n.end()){
            
            const char * node_name = i -> name().c_str();
            
            if (strcmp(node_name, "command") == 0){
                instructData->setCommand(i -> as_string());
            }
            else if (strcmp(node_name, "destination") == 0){
                instructData->setDestination(i -> as_string());
            }
            else if (strcmp(node_name, "look_up") == 0){
                instructData->setLook_up(i -> as_string());
            }
            else if (strcmp(node_name, "look_up_parameters") == 0){
                instructData->setLook_up_parameters(setMapFromNode(*i));
            }
            else if (strcmp(node_name, "payload") == 0){
                instructData->setPayload(setMapFromNode(*i));
            }
            //increment the iterator
            ++i;
        }
        return *instructData;
    }
    
    TimeData EquipletStep::setTimeDataFromNode(const JSONNode & n){
        TimeData * timeData = new TimeData();
        //For now its only 1 value. This might be more in the future
        JSONNode::const_iterator i = n.begin();
        
        while (i != n.end()){
            timeData->setDuration(i-> as_int());
            i++;
        }
        
        return *timeData;
    }
    
    std::map<std::string, std::string> EquipletStep::setMapFromNode(const JSONNode & n)
    {
        std::map<std::string, std::string> * newMap = new std::map<std::string, std::string>();
        
        JSONNode::const_iterator i = n.begin();
         while (i != n.end()){
             newMap->insert(std::pair<std::string,std::string>(i->name(),i->as_string()));
             i++;
         }
        
        return *newMap;
    }
    
}